#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <nrf_gzll.h>
#include <gzll_glue.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF); // Use INF for RTT logs

/* Pipe 0 is used for the LEFT half. */
#define PIPE_NUMBER 0

/* Polling and debounce delay in milliseconds */
#define POLLING_INTERVAL_MS 50

/* Define the number of buttons we are polling. */
#define NUM_BUTTONS 22

/* The payload length is the full array of buttons. */
#define TX_PAYLOAD_LENGTH (NUM_BUTTONS + 7) / 8 // Number of bytes needed to represent NUM_BUTTONS bits

/* Maximum number of transmission attempts */
#define MAX_TX_ATTEMPTS 100

/* --- NEW: Power Saving --- */
#define INACTIVITY_TIMEOUT_MEDIUM_MS (15 * 1000) // 15 seconds
#define INACTIVITY_TIMEOUT_SLEEP_MS  (30 * 1000) // 30 seconds

/* Power state machine */
enum power_mode {
    POWER_MODE_HIGH,
    POWER_MODE_MEDIUM,
    POWER_MODE_SLEEP
};
static enum power_mode current_power_mode = POWER_MODE_HIGH;
static uint32_t inactivity_counter_ms = 0;
/* --- END NEW --- */


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

/* Create arrays to hold the current and previous state of all buttons. */
static uint8_t current_button_states[TX_PAYLOAD_LENGTH] = {0}; // Buffer to hold current button states (1=pressed, 0=released)
static uint8_t prev_button_states[TX_PAYLOAD_LENGTH] = {0}; // Buffer to hold previous button states

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

/* Function prototypes */
static void gzll_tx_result_handler(struct gzll_tx_result *tx_result);
static void gzll_results_work_handler(struct k_work *work);
static void send_packet_work_handler(struct k_work *work);

void update_keystate(uint8_t *data_payload, size_t key_index, bool state);

int main(void)
{
    int err;
    bool result_value;

    /* --- Work Queue Initialization --- */
    k_work_init(&gzll_results_work, gzll_results_work_handler);
    k_work_init(&send_packet_work, send_packet_work_handler);

    /* --- GPIO Button Configuration (Polling) --- */
    for (int i = 0; i < NUM_BUTTONS; i++) {
        if (!gpio_is_ready_dt(&buttons[i])) {
            LOG_ERR("Error: button device %s is not ready", buttons[i].port->name);
            return 0;
        }
        err = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
        if (err) {
            LOG_ERR("Error %d: failed to configure %s pin %d",
                err, buttons[i].port->name, buttons[i].pin);
            return 0;
        }

        /* Read the initial state of all buttons (0 = released) */
        int state = gpio_pin_get_dt(&buttons[i]);
        update_keystate(prev_button_states, i, state);
        update_keystate(current_button_states, i, state);
    }

    /* --- Gazell Initialization --- */
    result_value = gzll_glue_init();
    if (!result_value) {
        LOG_ERR("Cannot initialize GZLL glue code");
        return 0;
    }

    result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    if (!result_value) {
        LOG_ERR("Cannot initialize GZLL");
        return 0;
    }

    // --- Set Base Addresses (from our previous fix) ---
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);
    // This sender (left) uses Pipe 0, which defaults to base_address_0
    
    nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);

    result_value = nrf_gzll_enable();
    if (!result_value) {
        LOG_ERR("Cannot enable GZLL");
        return 0;
    }

    // --- NEW: Set initial power mode ---
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_0_DBM); // 0dBm is max power
    current_power_mode = POWER_MODE_HIGH;
    LOG_INF("Gzll device started in HIGH POWER mode (Pipe %d).", PIPE_NUMBER);
    
    /*
     * The main loop now polls ALL buttons and checks for ANY state change.
     * It also manages the power-saving timeouts.
     */
    while (true) {
        bool state_changed = false;
        for (int i = 0; i < NUM_BUTTONS; i++) {
            update_keystate(current_button_states, i, gpio_pin_get_dt(&buttons[i]));
        }
        state_changed = memcmp(current_button_states, prev_button_states, TX_PAYLOAD_LENGTH) != 0;

        /* If any button changed state, trigger a send */
        if (state_changed) {
            // --- NEW: Reset inactivity counter on any key press ---
            inactivity_counter_ms = 0;
            
            // Submit the work to wake up (if needed) and send
            k_work_submit(&send_packet_work);
        
        } else {
            // --- NEW: No keys pressed, increment inactivity counter ---
            inactivity_counter_ms += POLLING_INTERVAL_MS;

            // Check for 30-second SLEEP timeout
            if (inactivity_counter_ms >= INACTIVITY_TIMEOUT_SLEEP_MS &&
                current_power_mode != POWER_MODE_SLEEP) 
            {
                LOG_INF("30s inactivity. Entering SLEEP mode (disabling Gazell).");
                nrf_gzll_disable();
                current_power_mode = POWER_MODE_SLEEP;
            }
            // Check for 15-second MEDIUM power timeout
            else if (inactivity_counter_ms >= INACTIVITY_TIMEOUT_MEDIUM_MS &&
                     current_power_mode == POWER_MODE_HIGH)
            {
                LOG_INF("15s inactivity. Entering MEDIUM power mode (-8dBm).");
                // We're still on, but at a lower power
                nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_N8_DBM);
                current_power_mode = POWER_MODE_MEDIUM;
            }
        }

        k_sleep(K_MSEC(POLLING_INTERVAL_MS));
    }
}

void update_keystate(uint8_t *data_payload, size_t key_index, bool state)
{
    size_t byte_index = key_index / 8;
    size_t bit_index = key_index % 8;

    if (state) {
        data_payload[byte_index] |= (1 << bit_index);
    } else {
        data_payload[byte_index] &= ~(1 << bit_index);
    }
}

bool get_keystate(uint8_t *data_payload, size_t key_index)
{
    size_t byte_index = key_index / 8;
    size_t bit_index = key_index % 8;

    return (data_payload[byte_index] & (1 << bit_index)) != 0;
}

static void send_packet_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    bool result_value;

    // --- NEW: Wake-up logic ---
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
    // --- END NEW ---

    
    char state_string[NUM_BUTTONS + 1];

    // Copy the current (logical 1=pressed) state
    memcpy(data_payload, current_button_states, TX_PAYLOAD_LENGTH);

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