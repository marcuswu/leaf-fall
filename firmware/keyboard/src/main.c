#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <string.h>

#include <nrfx_power.h>
#include <nrf_gzll.h>
#include <gzll_glue.h>
#include "leaf_fold.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG); // Use DBG for RTT logs

#define WORKQ_PRIORITY K_PRIO_PREEMPT(0)
#define WORKQ_STACK_SIZE 1024

#define DEBOUNCE_TICKS 5

/* Define the number of buttons we are polling. */
#define NUM_BUTTONS 22
#define HALF_NUM_BITS NUM_BUTTONS + 1 // Total bits needed (22 buttons + 1 power alert bit)

/* Number of bytes needed to represent NUM_BUTTONS bits */
#define TX_PAYLOAD_LENGTH (HALF_NUM_BITS + 7) / 8

/* Maximum number of transmission attempts */
#define MAX_TX_ATTEMPTS 100

/* Sleep settings for power saving */
#define INACTIVITY_TIMEOUT_LIGHT_SLEEP_TICKS 1500  // ~1.5 seconds
#define INACTIVITY_TIMEOUT_HEAVY_SLEEP_TICKS 10000 // ~10 seconds

#define KEEPALIVE_TICKS 1250 // 8Hz ticks for keepalive packets, but implemented as debounce ticks

/* CR2032 low-power threshold and power alert state */
#define LOW_POWER_POF_THRESHOLD NRF_POWER_POFTHR_V19
static atomic_t low_power_alert = ATOMIC_INIT(0);
// static bool low_power_alert = false;

const struct device *port = DEVICE_DT_GET(DT_NODELABEL(gpio0));

const struct device *counter_dev = DEVICE_DT_GET(DT_NODELABEL(timer0));
struct gpio_callback gpio_callback_struct;

/* Power state machine */
enum power_mode {
    POWER_MODE_HIGH,
    POWER_MODE_MEDIUM,
    POWER_MODE_SLEEP
};
static enum power_mode current_power_mode = POWER_MODE_HIGH;
static atomic_t inactivity_counter_ticks = ATOMIC_INIT(0);
// static int inactivity_counter_ticks = 0;

static uint32_t debounce_ticks = 0;
static atomic_t keepalive_ticks = ATOMIC_INIT(0);
// static int keepalive_ticks = 0;
static volatile bool debouncing = false;

/*
 * Create an array of gpio_dt_spec structs, initialized using the
 * aliases from the app.overlay file (sw0, sw1, ... sw16).
 */
static const struct gpio_dt_spec buttons[NUM_BUTTONS] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw4), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw5), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw6), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw7), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw8), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw9), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw10), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw11), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw12), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw13), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw14), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw15), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw16), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw17), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw18), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw19), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw20), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(sw21), gpios),
};

/* State variables to hold the current and debounce state of all buttons. */
static uint32_t current_key_states = 0;
static uint32_t debounce_key_states = 0;
static struct k_spinlock key_state_lock;
static struct k_spinlock key_read_lock;

/* Gazell Link Layer TX result structure */
struct gzll_tx_result {
    bool success;
    uint32_t pipe;
    nrf_gzll_device_tx_info_t info;
};

/* Placeholder for received ACK payloads from Host. */
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

// struct gzll_tx_result_node {
//     void *fifo_reserved;
//     struct gzll_tx_result tx_result;
// };

// K_FIFO_DEFINE(gzll_tx_result_fifo);
// K_FIFO_DEFINE(gzll_tx_result_free_fifo);
/* Gazell Link Layer TX result message queue */
K_MSGQ_DEFINE(gzll_rx_msgq,
          sizeof(struct gzll_tx_result),
          1,
          sizeof(uint32_t));

// static struct gzll_tx_result_node gzll_tx_result_nodes[4];

/* Work items */
static K_THREAD_STACK_DEFINE(worker_stack, WORKQ_STACK_SIZE);
static struct k_thread worker_thread;
static struct k_sem main_sem;
static struct k_sem counter_sem;
static struct k_work gzll_results_work;
static struct k_work send_packet_work;

/* Gazell Link Layer function prototypes */
static void gzll_tx_result_handler(struct gzll_tx_result *tx_result);
static void gzll_results_work_handler(struct k_work *work);
static void send_packet_work_handler(struct k_work *work);
static void worker_thread_fn(void *p1, void *p2, void *p3);

// Interrupt handler prototypes
void button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void power_warn_event_handler(void);
void counter_handler(const struct device *dev, void *user_data);
void debounce_work_handler(struct k_work *work);
void inactivity_work_handler(struct k_work *work);
void keepalive_work_handler(struct k_work *work);

// power failure warning configuration
nrfx_power_pofwarn_config_t pof_config = {
    .handler = power_warn_event_handler,
    .thr = LOW_POWER_POF_THRESHOLD
};

static inline uint32_t read_keys(bool print_keys)
{
    k_spinlock_key_t key = k_spin_lock(&key_read_lock);
    // nrf_gpio_port_in_read() can read the entire port of GPIOs at once
    // We can use this to read all buttons in one go, then update the keystate buffer
    uint32_t raw_input = 0;
    int result = gpio_port_get(port, &raw_input);
    if (result != 0) {
        LOG_ERR("Failed to read GPIO port: %d", result);
        return 0; // Return all keys unpressed on error
    }

    if (print_keys) {
        LOG_INF("Raw GPIO before mask: 0x%08X; INPUT_MASK: 0x%08X", raw_input, INPUT_MASK);
    }
    result = raw_input & INPUT_MASK;
    if (print_keys) {
        LOG_INF("Masked read key states: 0x%08X", result);
    }
    k_spin_unlock(&key_read_lock, key);
    return result;
}

// Initialize pof event configuration
static void power_failure_init(void)
{
    nrfx_power_pof_init(&pof_config);
    nrfx_power_pof_enable(&pof_config);
}

// Initialize gazell link layer configuration
void gazell_init() {
    bool result_value;

    k_work_init(&gzll_results_work, gzll_results_work_handler);
    k_work_init(&send_packet_work, send_packet_work_handler);

    result_value = gzll_glue_init();
    if (!result_value) {
        LOG_ERR("Cannot initialize GZLL glue code");
        return;
    }

    result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    if (!result_value) {
        LOG_ERR("Cannot initialize GZLL");
        return;
    }

    // for (int i = 0; i < (int)(sizeof(gzll_tx_result_nodes) / sizeof(gzll_tx_result_nodes[0])); i++) {
    //     k_fifo_put(&gzll_tx_result_free_fifo, &gzll_tx_result_nodes[i]);
    // }

    // --- Set Base Addresses (from our previous fix) ---
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);
    // This sender (left) uses Pipe 0, which defaults to base_address_0
    
    nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);

    result_value = nrf_gzll_enable();
    if (!result_value) {
        LOG_ERR("Cannot enable GZLL");
        return;
    }

    // --- NEW: Set initial power mode ---
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM); // 0dBm is max power
    current_power_mode = POWER_MODE_HIGH;
    LOG_INF("Gzll device started in HIGH POWER mode (Pipe %d).", PIPE_NUMBER);
}

// Initialize counter to handle key state changes, debouncing, and inactivity timeouts
void counter_config(void)
{
    int err;
    k_sem_init(&counter_sem, 0, 1000);
    k_thread_create(&worker_thread, worker_stack, WORKQ_STACK_SIZE,
                    worker_thread_fn, NULL, NULL, NULL,
                    WORKQ_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&worker_thread, "key_worker");
    LOG_INF("Worker thread created with priority %d", WORKQ_PRIORITY);
    k_work_init(&send_packet_work, send_packet_work_handler);

    struct counter_top_cfg counter_config = {
        .callback = counter_handler,
        .ticks = 16000, // 16Mhz clock so 16000 ticks = 1 ms
        .user_data = NULL,
        .flags = COUNTER_TOP_CFG_RESET_WHEN_LATE | COUNTER_CONFIG_INFO_COUNT_UP
    };

    if (!device_is_ready(counter_dev)) {
        LOG_ERR("Counter device is not ready");
        return;
    }

    err = counter_set_top_value(counter_dev, &counter_config);
    if (err < 0) {
        LOG_ERR("Counter device failed to set: %d", err);
        return;
    }
    err = counter_start(counter_dev);
    if (err < 0) {
        LOG_ERR("Counter device failed to start: %d", err);
        return;
    }

    LOG_INF("Counter started successfully");
}

// Initialize GPIOTE to wake up on button presses if we are in sleep mode
void gpio_config(void)
{
    if (!device_is_ready(port)) {
        LOG_ERR("GPIO port is not ready");
    }
    // New implementation using Zephyr's GPIO API with interrupts for simplicity and reliability
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (!gpio_is_ready_dt(&buttons[i])) {
            LOG_ERR("GPIO device for button %d is not ready", i);
            continue;
        }

        int err = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
        if (err != 0) {
            LOG_ERR("Failed to configure GPIO pin %d: %d", buttons[i].pin, err);
            continue;
        }
        LOG_INF("Configured button %d pin %d flags=0x%08X", i, buttons[i].pin, buttons[i].dt_flags);
        // Set up the interrupt for every pin. NRF chips can only do this on 6 pins unless using edge sense,
        // so be sure to set this in your overlay for your gpio (replacing the mask with the pins you need):
        // sense-edge-mask = < 0xF01FC7FF >;
        err = gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_BOTH);
        if (err < 0) {
            LOG_ERR("Failed to configure interrupt for GPIO pin %d: %d", buttons[i].pin, err);
            continue;
        }
    }
    // Set the same button_handler for all button interrupts
    gpio_init_callback(&gpio_callback_struct, button_handler, INPUT_MASK);
    gpio_add_callback(port, &gpio_callback_struct);
}

int main(void)
// void thread0(void)
{
    // Initialize pof, gazell, rtc, and gpiote subsystems
    LOG_INF("Starting Leaf Fall Keyboard...");
    LOG_INF("using input mask: 0x%08X", INPUT_MASK);
    LOG_INF("Setting up POF warning");
    power_failure_init();
    k_sem_init(&main_sem, 0, 1000);
    LOG_INF("Initializing Gazell");
    gazell_init();
    LOG_INF("Configuring Counter");
    counter_config();
    LOG_INF("Configuring GPIOTE");
    gpio_config();
    printk("Main loop initialized.\n");
    // manual_isr_setup();

    // k_work_submit(&send_packet_work);
    // send_packet_work_handler(NULL);

    /*
     * The main loop just waits for events
     */
    LOG_INF("Entering main loop, waiting for events...");
    while (true) {
        k_sem_take(&main_sem, K_FOREVER);
        k_cpu_idle();
        // k_cpu_idle(); // Use Zephyr's idle function to allow for power management and event handling
        // k_msleep(10);
    }
    return 0;
}

/* ---- Interrupt Handlers ---- */
void power_warn_event_handler(void)
{
    // We'll leave this on unconditionally.
    // If the battery is replaced, the flag will be false again on power up
    atomic_set(&low_power_alert, 1);
    // low_power_alert = true;
}

static inline void wake_up_from_sleep()
{
    if (current_power_mode != POWER_MODE_SLEEP && current_power_mode != POWER_MODE_MEDIUM) {
        return;
    }
    if (current_power_mode == POWER_MODE_SLEEP) {
        // Rise from full sleep to medium power
        nrf_gzll_enable();
        counter_start(counter_dev);
    }
    // Rise from medium power to high power
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
    current_power_mode = POWER_MODE_HIGH;
}

void button_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    LOG_INF("Button event detected on pins 0x%08X", pins);
    wake_up_from_sleep();
    k_spinlock_key_t key = k_spin_lock(&key_state_lock);
    debouncing = true;
    debounce_ticks = 0;
    k_spin_unlock(&key_state_lock, key);
}

/* ---- Tick Handlers ---- */

// Send key states at 8hz keepalive interval even w/o key changes
void keepalive_handler()
{
    // send_packet_work_handler(NULL);
    k_work_submit(&send_packet_work);
}

// Handle debouncing key presses and sleep logic for inactivity
void counter_handler(const struct device *dev, void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);
    // LOG_DBG("counter_handler fired");
    k_sem_give(&counter_sem);
}

void debounce_work_handler(struct k_work *work)
{
    k_spinlock_key_t key = k_spin_lock(&key_state_lock);
    // LOG_INF("debounce work handler");
    if (debouncing) {
        uint32_t current_read_keys = read_keys(false);
        if (debounce_key_states != current_read_keys) {
            debounce_key_states = current_read_keys;
            debounce_ticks = 0;
            LOG_INF("Detected key change, resetting debounce. key states: 0x%08X", debounce_key_states);
        } else {
            debounce_ticks++;
            if (debounce_ticks >= DEBOUNCE_TICKS) {
                debouncing = false;
                debounce_ticks = 0;
                current_key_states = debounce_key_states;
                LOG_INF("Debounce complete. key states: 0x%08X", current_key_states);
                // send_packet_work_handler(NULL);
                k_work_submit(&send_packet_work);
            }
        }
    }
    k_spin_unlock(&key_state_lock, key);

    // check_inactivity_timeout();
}

// void check_inactivity_timeout()
void inactivity_work_handler(struct k_work *work)
{

    if (read_keys(false) != 0) {
        // If any key is pressed, reset the inactivity counter and ensure we're in high power mode
        atomic_set(&inactivity_counter_ticks, 0);
        // inactivity_counter_ticks = 0;
        // LOG_INF("Activity detected. Resetting inactivity counter.");
        return;
    }

    atomic_inc(&inactivity_counter_ticks);
    // inactivity_counter_ticks++;

    if (atomic_get(&inactivity_counter_ticks) <= INACTIVITY_TIMEOUT_LIGHT_SLEEP_TICKS) {
    // if (inactivity_counter_ticks <= INACTIVITY_TIMEOUT_LIGHT_SLEEP_TICKS) {
        return;
    }
    atomic_set(&inactivity_counter_ticks, 0);
    // inactivity_counter_ticks = 0;

    /*
    if (current_power_mode == POWER_MODE_HIGH) {
        // Reduce transmit power to save battery
        LOG_INF("Inactivity detected. Setting MEDIUM power mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_N8_DBM); // Reduce power to -4dBm
        current_power_mode = POWER_MODE_MEDIUM;
    } else if (current_power_mode == POWER_MODE_MEDIUM && atomic_get(&inactivity_counter_ticks) >= INACTIVITY_TIMEOUT_HEAVY_SLEEP_TICKS) {
        // Reduce power to minimum -- wakes up on GPIO event
        LOG_INF("Extended inactivity detected. Setting SLEEP mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_N8_DBM);
        nrf_gzll_disable();
        current_power_mode = POWER_MODE_SLEEP;
        counter_stop(counter_dev);
    }
    */
}

void keepalive_work_handler(struct k_work *work)
{
    atomic_inc(&keepalive_ticks);
    // keepalive_ticks++;
    // if (keepalive_ticks >= KEEPALIVE_TICKS) {
    //     keepalive_ticks = 0;
    //     keepalive_handler();
    // }
    if (atomic_get(&keepalive_ticks) >= KEEPALIVE_TICKS) {
        atomic_set(&keepalive_ticks, 0);
        keepalive_handler();
    }
}
// Helper function to get the state of a key from the data payload
// Used for logging the state of keys based on the payload we're sending
// inline the function to avoid function call overhead since this will be called for every key in the sent payload
/*inline bool get_keystate(uint8_t *data_payload, size_t key_index)
{
    size_t byte_index = key_index / 8;
    size_t bit_index = key_index % 8;

    return (data_payload[byte_index] & (1 << bit_index)) != 0;
}*/


static void send_packet_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    bool result_value;
    /* Payload to send to Host is now NUM_BUTTONS bytes long. */
    uint8_t data_payload[TX_PAYLOAD_LENGTH] = {0}; // Initialize all bytes to 0

    // If we're asleep, wake up and set to high power
    wake_up_from_sleep();

    k_spinlock_key_t key = k_spin_lock(&key_state_lock);
    uint32_t key_states = current_key_states;
    k_spin_unlock(&key_state_lock, key);
    bool low_power = atomic_get(&low_power_alert);
    // bool low_power = false;
    // Update the payload based on the current key states.
    data_payload[0] = ((key_states & (1<<S00)) ? 1 : 0) << 7 |
                       ((key_states & (1<<S01)) ? 1 : 0) << 6 | 
                       ((key_states & (1<<S02)) ? 1 : 0) << 5 | 
                       ((key_states & (1<<S03)) ? 1 : 0) << 4 |
                       ((key_states & (1<<S04)) ? 1 : 0) << 3 | 
                       ((key_states & (1<<S05)) ? 1 : 0) << 2 |
                       ((key_states & (1<<S06)) ? 1 : 0) << 1 | 
                       ((key_states & (1<<S07)) ? 1 : 0);
    data_payload[1] = ((key_states & (1<<S08)) ? 1 : 0) << 7 | 
                       ((key_states & (1<<S09)) ? 1 : 0) << 6 | 
                       ((key_states & (1<<S10)) ? 1 : 0) << 5 |
                       ((key_states & (1<<S11)) ? 1 : 0) << 4 |
                       ((key_states & (1<<S12)) ? 1 : 0) << 3 |
                       ((key_states & (1<<S13)) ? 1 : 0) << 2 |
                       ((key_states & (1<<S14)) ? 1 : 0) << 1 |
                       ((key_states & (1<<S15)) ? 1 : 0);
    data_payload[2] = ((key_states & (1<<S16)) ? 1 : 0) << 7 |
                       ((key_states & (1<<S17)) ? 1 : 0) << 6 |
                       ((key_states & (1<<S18)) ? 1 : 0) << 5 |
                       ((key_states & (1<<S19)) ? 1 : 0) << 4 |
                       ((key_states & (1<<S20)) ? 1 : 0) << 3 |
                       ((key_states & (1<<S21)) ? 1 : 0) << 2 |
                       ((low_power) ? 1 : 0) << 1 | // Power alert bit
                       (0); // Padding bit

    /* Build the log string based on the (correct) packet data */
    // char state_string[NUM_BUTTONS + 1];
    // for (int i = 0; i <= NUM_BUTTONS; i++) {
    //     state_string[i] = (get_keystate(data_payload, i)) ? '1' : '0';
    // }
    // state_string[NUM_BUTTONS] = '\0';

    LOG_INF("Sending 3 bytes: [0x%02x 0x%02x 0x%02x]", data_payload[0], data_payload[1], data_payload[2]);

    /* Send the entire data_payload (NUM_BUTTONS bytes). */
    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER,
                            data_payload,
                            NUM_BUTTONS);
    if (!result_value) {
        LOG_ERR("TX fifo error");
    }
}

static void gzll_device_report_tx(bool success,
                  uint32_t pipe,
                  nrf_gzll_device_tx_info_t *tx_info)
{
    int err;
    struct gzll_tx_result tx_result = {
        .success = success,
        .pipe = pipe,
        .info = *tx_info
    };

    err = k_msgq_put(&gzll_rx_msgq, &tx_result, K_NO_WAIT);
    if (err != 0) {
        LOG_ERR("Failed to put TX result in message queue: %d", err);
        return;
    }
    k_work_submit(&gzll_results_work);
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    gzll_device_report_tx(true, pipe, &tx_info);
}

void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    LOG_WRN("TX failed from callback!");
    gzll_device_report_tx(false, pipe, &tx_info);
}

void nrf_gzll_disabled(void)
{
}

void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
}

static void gzll_results_work_handler(struct k_work *work)
{
    struct gzll_tx_result tx_result;

    while ((k_msgq_get(&gzll_rx_msgq, &tx_result, K_NO_WAIT) == 0)) {
        gzll_tx_result_handler(&tx_result);
    }

    k_sem_give(&main_sem);
}

static void worker_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Worker thread starting...");
    while (true) {
        k_sem_take(&counter_sem, K_FOREVER);
        // LOG_INF("Worker thread processing...");
        debounce_work_handler(NULL);
        inactivity_work_handler(NULL);
        keepalive_work_handler(NULL);
    }
}

static void gzll_tx_result_handler(struct gzll_tx_result *tx_result)
{
    bool result_value;
    uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    if (tx_result->success) {
        LOG_INF("Gazell transmission successful.");
        if (tx_result->info.payload_received_in_ack) {
            result_value = nrf_gzll_fetch_packet_from_rx_fifo(tx_result->pipe,
                                          ack_payload,
                                          &ack_payload_length);
            if (!result_value) {
                LOG_ERR("RX fifo error");
            } else if (ack_payload_length > 0) {
                LOG_INF("ACK payload received. LED state would be: 0x%02x", ack_payload[0]);
            }
        }
    } else {
        LOG_ERR("Gazell transmission failed.");
    }
}