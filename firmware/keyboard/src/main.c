#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <nrfx_power.h>
#include <nrfx_gpiote.h>
#include <nrfx_rtc.h>
#include <nrf_gzll.h>
#include <gzll_glue.h>
#include "leaf_fold.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF); // Use INF for RTT logs

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

/* CR2032 low-power threshold and power alert state */
#define LOW_POWER_POF_THRESHOLD NRF_POWER_POFTHR_V19
static volatile bool low_power_alert = false;

static nrfx_rtc_t rtc_debounce = NRFX_RTC_INSTANCE(0);
static nrfx_rtc_t rtc_keepalive = NRFX_RTC_INSTANCE(1);
static nrfx_gpiote_t gpiote_instance = NRFX_GPIOTE_INSTANCE(0);

/* Power state machine */
enum power_mode {
    POWER_MODE_HIGH,
    POWER_MODE_MEDIUM,
    POWER_MODE_SLEEP
};
static enum power_mode current_power_mode = POWER_MODE_HIGH;
static uint32_t inactivity_counter_ticks = 0;

static uint32_t debounce_ticks;
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

/* Gazell Link Layer TX result structure */
struct gzll_tx_result {
    bool success;
    uint32_t pipe;
    nrf_gzll_device_tx_info_t info;
};

/* Payload to send to Host is now NUM_BUTTONS bytes long. */
static uint8_t data_payload[TX_PAYLOAD_LENGTH];

/* Placeholder for received ACK payloads from Host. */
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

/* Gazell Link Layer TX result message queue */
K_MSGQ_DEFINE(gzll_msgq,
          sizeof(struct gzll_tx_result),
          1,
          sizeof(uint32_t));

/* Work items */
static struct k_work gzll_results_work;
static struct k_work send_packet_work;

/* Gazell Link Layer function prototypes */
static void gzll_tx_result_handler(struct gzll_tx_result *tx_result);
static void gzll_results_work_handler(struct k_work *work);
static void send_packet_work_handler(struct k_work *work);

// Interrupt handler prototypes
void wake_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context);
void power_warn_event_handler(void);
void keepalive_handler(nrfx_rtc_int_type_t int_type);
void debounce_handler(nrfx_rtc_int_type_t int_type);
void check_inactivity_timeout(void);

// power failure warning configuration
static nrfx_power_pofwarn_config_t pof_config = {
    .handler = power_warn_event_handler,
    .thr = LOW_POWER_POF_THRESHOLD
};

// Read the state of all buttons and return as a bitfield
static inline uint32_t read_keys(void)
{
    // nrf_gpio_port_in_read() can read the entire port of GPIOs at once
    // We can use this to read all buttons in one go, then update the keystate buffer
    return nrf_gpio_port_in_read(NRF_GPIO) & INPUT_MASK;
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

// Initialize Real Time Counter to handle key state changes, debouncing, and inactivity timeouts
void rtc_config(void)
{
    nrfx_rtc_config_t rtc_debounce_config = {
        .prescaler = NRF_RTC_FREQ_TO_PRESCALER(1000), // 1000 Hz to optimize response time
        .interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY,
        .tick_latency = NRFX_RTC_US_TO_TICKS(2000, 32768), // 2ms max latency
        .reliable = false
    };
    nrfx_rtc_config_t rtc_keepalive_config = {
        .prescaler = NRF_RTC_FREQ_TO_PRESCALER(8), // 8 Hz
        .interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY,
        .tick_latency = NRFX_RTC_US_TO_TICKS(2000, 32768), // 2ms max latency
        .reliable = false
    };

    // debounce handler handles recognizing and sending key state changes and inactivity management
    nrfx_rtc_init(&rtc_debounce, &rtc_debounce_config, &debounce_handler);
    nrfx_rtc_enable(&rtc_debounce);

    // keepalive handler handles sending periodic keepalive key state updates
    nrfx_rtc_init(&rtc_keepalive, &rtc_keepalive_config, &keepalive_handler);
    nrfx_rtc_enable(&rtc_keepalive);
}

// Initialize GPIOTE to wake up on button presses if we are in sleep mode
void gpiote_config(void)
{
    nrfx_gpiote_init(&gpiote_instance, 0);
    // Wake up on any button press using GPIOTE events
    // pull pin up since buttons connect to GND when pressed
    nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLUP;
    // Send GPIOTE event on high-to-low transition (button press)
    nrfx_gpiote_trigger_config_t trigger_config =  {
        .trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
        .p_in_channel = NULL
    };
    // Any button press will trigger the wake handler
    nrfx_gpiote_handler_config_t handler_config = {
        .handler = wake_handler,
        .p_context = NULL
    };
    nrfx_gpiote_input_pin_config_t input_config = {
        .p_pull_config = &pull_config,
        .p_trigger_config = &trigger_config,
        .p_handler_config = &handler_config
    };
    for (int i = 0; i < NUM_BUTTONS; i++) {
        nrfx_gpiote_input_configure(&gpiote_instance, buttons[i].pin, &input_config);
    }
    // set initial key states
    current_key_states = read_keys();
}

int main(void)
{
    // Initialize pof, gazell, rtc, and gpiote subsystems
    power_failure_init();
    gazell_init();
    rtc_config();
    gpiote_config();

    /*
     * The main loop just waits for events
     */
    while (true) {
        __WFE(); // Wait for event (low-power sleep until next RTC interrupt or GPIO event)
        __SEV(); // Ensure that we wake up on the next event
        __WFE(); // Wait for event (consume the event that woke us up)
    }
}

/* ---- Interrupt Handlers ---- */
void power_warn_event_handler(void)
{
    // We'll leave this on unconditionally.
    // If the battery is replaced, the flag will be false again on power up
    low_power_alert = true;
}

void wake_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *p_context)
{
    // This will be called on any button press due to our GPIOTE configuration
    // We can use this to wake up from sleep immediately without waiting for the next RTC tick
    if (current_power_mode == POWER_MODE_SLEEP) {
        LOG_INF("Woke up from SLEEP due to GPIO event. Re-enabling Gazell in HIGH power mode.");
        nrf_gzll_enable();
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
        current_power_mode = POWER_MODE_HIGH;
        nrfx_rtc_enable(&rtc_debounce);
        nrfx_rtc_enable(&rtc_keepalive);
    }
    if (current_power_mode == POWER_MODE_MEDIUM) {
        LOG_INF("Activity detected from GPIO event. Setting HIGH power mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
        current_power_mode = POWER_MODE_HIGH;
    }
}

/* ---- Tick Handlers ---- */

// Send key states at 8hz keepalive interval even w/o key changes
void keepalive_handler(nrfx_rtc_int_type_t int_type)
{
    k_work_submit(&send_packet_work);
}

// Handle debouncing key presses and sleep logic for inactivity
void debounce_handler(nrfx_rtc_int_type_t int_type)
{
    if (!debouncing && current_key_states != read_keys()) {
        // If we detect a change and we're not already debouncing, start debouncing
        debouncing = true;
        debounce_key_states = read_keys();
        debounce_ticks = 0;
    }

    if (debouncing) {
        if (debounce_key_states != read_keys()) {
            debouncing = false;
        } else {
            debounce_ticks++;
            if (debounce_ticks >= DEBOUNCE_TICKS) {
                debouncing = false;
                current_key_states = debounce_key_states;
                k_work_submit(&send_packet_work);
            }
        }
    }

    check_inactivity_timeout();
}

void check_inactivity_timeout(void)
{
    if (read_keys() != 0) {
        // If any key is pressed, reset the inactivity counter and ensure we're in high power mode
        inactivity_counter_ticks = 0;
        return;
    }

    inactivity_counter_ticks += 1;

    if (inactivity_counter_ticks <= INACTIVITY_TIMEOUT_LIGHT_SLEEP_TICKS) {
        return;
    }

    if (current_power_mode == POWER_MODE_HIGH) {
        // Reduce transmit power to save battery
        LOG_INF("Inactivity detected. Setting MEDIUM power mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_N8_DBM); // Reduce power to -4dBm
        current_power_mode = POWER_MODE_MEDIUM;
    } else if (current_power_mode == POWER_MODE_MEDIUM && inactivity_counter_ticks >= INACTIVITY_TIMEOUT_HEAVY_SLEEP_TICKS) {
        // Reduce power to minimum -- wakes up on GPIO event
        LOG_INF("Extended inactivity detected. Setting SLEEP mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_N8_DBM);
        nrf_gzll_disable();
        current_power_mode = POWER_MODE_SLEEP;
        nrfx_rtc_disable(&rtc_debounce);
        nrfx_rtc_disable(&rtc_keepalive);
    }

}

// Helper function to get the state of a key from the data payload
// Used for logging the state of keys based on the payload we're sending
// inline the function to avoid function call overhead since this will be called for every key in the sent payload
inline bool get_keystate(uint8_t *data_payload, size_t key_index)
{
    size_t byte_index = key_index / 8;
    size_t bit_index = key_index % 8;

    return (data_payload[byte_index] & (1 << bit_index)) != 0;
}


static void send_packet_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    bool result_value;

    // If we're asleep, wake up and set to high power
    if (current_power_mode == POWER_MODE_SLEEP) {
        LOG_INF("Waking from SLEEP. Re-enabling Gazell in HIGH power mode.");
        result_value = nrf_gzll_enable();
        if (!result_value) { LOG_ERR("Failed to re-enable Gazell!"); }
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
        current_power_mode = POWER_MODE_HIGH;
    }
    // If we were in medium power, ramp back up to high power
    else if (current_power_mode == POWER_MODE_MEDIUM) {
        LOG_INF("Activity detected. Setting HIGH power mode.");
        nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM);
        current_power_mode = POWER_MODE_HIGH;
    }

    
    char state_string[NUM_BUTTONS + 1];

    // Update the payload based on the current key states.
    data_payload[0] = (current_key_states & 1<<S00) ? 1 : 0 << 7 |
                       (current_key_states & 1<<S01) ? 1 : 0 << 6 | 
                       (current_key_states & 1<<S02) ? 1 : 0 << 5 | 
                       (current_key_states & 1<<S03) ? 1 : 0 << 4 | 
                       (current_key_states & 1<<S04) ? 1 : 0 << 3 | 
                       (current_key_states & 1<<S05) ? 1 : 0 << 2 | 
                       (current_key_states & 1<<S06) ? 1 : 0 << 1 | 
                       (current_key_states & 1<<S07) ? 1 : 0;
    data_payload[1] = (current_key_states & 1<<S08) ? 1 : 0 << 7 | 
                       (current_key_states & 1<<S09) ? 1 : 0 << 6 | 
                       (current_key_states & 1<<S10) ? 1 : 0 << 5 |
                       (current_key_states & 1<<S11) ? 1 : 0 << 4 |
                       (current_key_states & 1<<S12) ? 1 : 0 << 3 |
                       (current_key_states & 1<<S13) ? 1 : 0 << 2 |
                       (current_key_states & 1<<S14) ? 1 : 0 << 1 |
                       (current_key_states & 1<<S15) ? 1 : 0;
    data_payload[2] = (current_key_states & 1<<S16) ? 1 : 0 << 7 |
                       (current_key_states & 1<<S17) ? 1 : 0 << 6 |
                       (current_key_states & 1<<S18) ? 1 : 0 << 5 |
                       (current_key_states & 1<<S19) ? 1 : 0 << 4 |
                       (current_key_states & 1<<S20) ? 1 : 0 << 3 |
                       (current_key_states & 1<<S21) ? 1 : 0 << 2 |
                       (low_power_alert) ? 1 : 0 << 1 | // Power alert bit
                       0; // Padding bit

    /* Build the log string based on the (correct) packet data */
    for (int i = 0; i < NUM_BUTTONS; i++) {
        state_string[i] = (get_keystate(data_payload, i)) ? '1' : '0';
    }
    state_string[NUM_BUTTONS] = '\0';

    LOG_INF("Sending 17 bytes: [%s] (1=PRESSED)", state_string);

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
    struct gzll_tx_result tx_result;

    tx_result.success = success;
    tx_result.pipe = pipe;
    tx_result.info = *tx_info;
    err = k_msgq_put(&gzll_msgq, &tx_result, K_NO_WAIT);
    if (!err) {
        k_work_submit(&gzll_results_work);
    } else {
        LOG_ERR("Cannot put TX result to message queue");
    }
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

    while (!k_msgq_get(&gzll_msgq, &tx_result, K_NO_WAIT)) {
        gzll_tx_result_handler(&tx_result);
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