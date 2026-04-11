#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <string.h>

#include <nrf_gzll.h>
#include <gzll_glue.h>

#include <zephyr/sys/util.h>

#define KEYS_PER_ROW_HALF 6
#define KEYS_PER_ROW_FULL (KEYS_PER_ROW_HALF * 2)
#define FULL_ROWS 3
#define KEYS_FINAL_ROW_HALF 4
#define KEYS_PER_HALF (KEYS_PER_ROW_HALF * FULL_ROWS + KEYS_FINAL_ROW_HALF)
#define TOTAL_KEYS (KEYS_PER_HALF * 2)

// Gazell config
#define PIPE_NUMBER_LEFT 0
#define PIPE_NUMBER_RIGHT 1
#define HALF_PAYLOAD_LENGTH 3 // 3 byte payload length for each keyboard half
#define DEVICE_PAYLOAD_LENGTH (HALF_PAYLOAD_LENGTH * 2) // payload length for full keyboard
#define TX_PAYLOAD_LENGTH 1 // 1 byte payload length for ACKs
static uint8_t ack_payload[TX_PAYLOAD_LENGTH]; // Payload to attach to ACK sent to device.

// New data flags for each pipe
static volatile bool packet_received_left = false;
static volatile bool packet_received_right = false;

// UART config
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define BITS_PER_ROW_HALF KEYS_PER_ROW_FULL // Number of bits needed to represent the state of each half row of keys
// A bit pattern that will never appear in the actual key state data, used to indicate end of frame to host
#define END_OF_FRAME_BYTE (0xFF>>BITS_PER_ROW_HALF) << BITS_PER_ROW_HALF // 0xC0 for 6 columns per half, 0xE0 for 5 columns per half.
#define TOTAL_DATA_BYTES ((FULL_ROWS + 1) * 2) // 2 bytes per row for full keyboard
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const uint8_t STOP_BYTE = END_OF_FRAME_BYTE; // Arbitrary stop byte to indicate end of frame

// key state buffers
static volatile uint8_t keystate[TOTAL_DATA_BYTES] = {0}; // Buffer to hold key states received from left half of keyboard

static bool initialize_uart(void);
static bool initialize_gazell(void);
static void fetch_gazell_packet(uint32_t pipe);
void update_keystate(uint32_t pipe, uint8_t *data_payload, size_t row, size_t column);

int main(void)
{
    if (!initialize_uart())
    {
        return 0;
    }

    if (!initialize_gazell())
    {
        return 0;
    }

    while (true)
    {
        // Check for new packets from left and right halves of keyboard
        if (packet_received_left)
        {
            packet_received_left = false;
            fetch_gazell_packet(PIPE_NUMBER_LEFT);
        }
        if (packet_received_right)
        {
            packet_received_right = false;
            fetch_gazell_packet(PIPE_NUMBER_RIGHT);
        }

        // Check for poll request from host
        uint8_t rx_byte;
        if (uart_poll_in(uart_dev, &rx_byte) == 0 && rx_byte == 's')
        {
            // Send keystates to host in a single frame
            uint8_t tx_frame[TOTAL_DATA_BYTES + 1]; // +1 for stop byte
            memcpy(tx_frame, keystate, DEVICE_PAYLOAD_LENGTH);
            tx_frame[TOTAL_DATA_BYTES] = STOP_BYTE; // Append stop byte to indicate end of frame
            for (size_t i = 0; i < sizeof(tx_frame); i++)
            {
                uart_poll_out(uart_dev, tx_frame[i]);
            }
        }

        k_sleep(K_USEC(10)); // Allow UART buffers to clear
    }
}


static bool initialize_uart(void)
{
    if (!device_is_ready(uart_dev))
    {
        return false;
    }

    return true;
}

static bool initialize_gazell(void)
{
    bool success;

    // Initialize Gazell in host mode
    success = gzll_glue_init();
    if (!success) {
        return false;
    }
    success = nrf_gzll_init(NRF_GZLL_MODE_HOST);
    if (!success) {
        return false;
    }

    // Set base addresses for Gazell pipes
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);

    // Load ACK payload into TX queue for both pipes
    ack_payload[0] = 0xFF; // Arbitrary ACK payload
    // ack left pipe
    success = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER_LEFT, ack_payload, TX_PAYLOAD_LENGTH);
    if (!success) {
        return false;
    }
    // ack right pipe
    success = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER_RIGHT, ack_payload, TX_PAYLOAD_LENGTH);
    if (!success) {
        return false;
    }

    // Enable Gazell to start sending over the air
    success = nrf_gzll_enable();
    if (!success) {
        return false;
    }

    return true;
}

void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
    if (pipe == PIPE_NUMBER_LEFT) {
        packet_received_left = true;
    } else if (pipe == PIPE_NUMBER_RIGHT) {
        packet_received_right = true;
    }
}

static void fetch_gazell_packet(uint32_t pipe)
{
    uint8_t data_payload[HALF_PAYLOAD_LENGTH];
    uint32_t payload_length = HALF_PAYLOAD_LENGTH;

    // Fetch packet from Gazell RX FIFO
    bool success = nrf_gzll_fetch_packet_from_rx_fifo(pipe, data_payload, &payload_length);
    if (!success) {
        return;
    }

    // Data is received per side of the keyboard, with each key represented by a bit. Each side is in row major order.
    // The keystate buffer is represented in a combined row major order for the whole keyboard.
    for (size_t i = 0; i < FULL_ROWS; i++)
    {
        for (size_t j = 0; j < KEYS_PER_ROW_HALF; j++)
        {
            update_keystate(pipe, data_payload, i, j);
        }
    }
    for (size_t j = 0; j < KEYS_FINAL_ROW_HALF; j++)
    {
        update_keystate(pipe, data_payload, FULL_ROWS, j);
    }

    // Load ACK payload into TX queue for the pipe
    ack_payload[0] = 0xFF; // Arbitrary ACK payload
    nrf_gzll_add_packet_to_tx_fifo(pipe, ack_payload, TX_PAYLOAD_LENGTH);
}

void update_keystate(uint32_t pipe, uint8_t *data_payload, size_t row, size_t column)
{
    size_t key_index = row * KEYS_PER_ROW_HALF + column;
    size_t byte_index = key_index / 8;
    size_t bit_index = key_index % 8;

    // Extract bit value for the key from the received data payload
    bool bit_value = (data_payload[byte_index] >> bit_index) & 1;

    // Each row is a byte, and columns are represented by bits within that byte packed to the lower bits of the byte.
    const size_t max_column = (row == FULL_ROWS) ? KEYS_FINAL_ROW_HALF-1 : KEYS_PER_ROW_HALF-1;
    size_t column_index = (max_column*2) - (column + max_column);
    // Each row takes 2 bytes in the keystate buffer, left half is in even bytes and right half is in odd bytes
    size_t keystate_index = row * 2 + (pipe == PIPE_NUMBER_LEFT ? 0 : 1); 
    keystate[keystate_index] &= ~(1 << column_index); // Clear bit in keystate buffer
    keystate[keystate_index] |= (bit_value << column_index); // Set bit in keystate buffer based on received data
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}
