#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <string.h>

#include <nrf_gzll.h>
#include <gzll_glue.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG); // Use DBG for RTT logs

#define KEYS_PER_ROW_HALF 6
#define KEYS_PER_ROW_FULL (KEYS_PER_ROW_HALF * 2)
#define FULL_ROWS 3
#define KEYS_FINAL_ROW_HALF 4 + 1 // 4 keys in the final row (thumb cluster) plus 1 power alert bit
#define KEYS_PER_HALF (KEYS_PER_ROW_HALF * FULL_ROWS + KEYS_FINAL_ROW_HALF)
#define TOTAL_KEYS (KEYS_PER_HALF * 2)

// Gazell config
#define PIPE_NUMBER_LEFT 0
#define PIPE_NUMBER_RIGHT 1
#define HALF_NUM_BITS KEYS_PER_HALF // Total bits needed (22 buttons + 1 power alert bit)
#define HALF_PAYLOAD_LENGTH (HALF_NUM_BITS + 7) / 8 // Number of bytes needed to represent NUM_BUTTONS bits
#define TX_PAYLOAD_LENGTH 1 // 1 byte payload length for ACKs
static uint8_t ack_payload[TX_PAYLOAD_LENGTH]; // Payload to attach to ACK sent to device.

// New data flags for each pipe
static volatile bool packet_received_left = false;
static volatile bool packet_received_right = false;

// UART config
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define BITS_PER_ROW_HALF KEYS_PER_ROW_HALF // Number of bits needed to represent the state of each half row of keys
// A bit pattern that will never appear in the actual key state data, used to indicate end of frame to host
#define END_OF_FRAME_BYTE (0xFF>>BITS_PER_ROW_HALF) << BITS_PER_ROW_HALF // 0xC0 for 6 columns per half, 0xE0 for 5 columns per half.
#define TOTAL_DATA_BYTES ((FULL_ROWS + 1) * 2) // 2 bytes per row for full keyboard
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const uint8_t STOP_BYTE = END_OF_FRAME_BYTE; // Arbitrary stop byte to indicate end of frame

struct gzll_rx_result {
    uint32_t pipe;
    nrf_gzll_host_rx_info_t info;
};

// Structure to hold packet data for keystate update thread
struct keystate_packet {
    uint32_t pipe;
    uint8_t data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
    uint32_t payload_length;
};

// key state buffer
static struct k_spinlock key_state_lock;
static uint8_t keystate[TOTAL_DATA_BYTES] = {0}; // Buffer to hold key states received from left half of keyboard

// Message queue for keystate updates
K_MSGQ_DEFINE(keystate_msgq,
              sizeof(struct keystate_packet),
              8,
              4);

K_MSGQ_DEFINE(gzll_msgq,
	      sizeof(struct gzll_rx_result),
	      1,
	      sizeof(uint32_t));

static K_SEM_DEFINE(main_sem, 0, 1);
static struct k_work gzll_work;

// Keystate processing thread stack and thread
#define KEYSTATE_THREAD_STACK_SIZE 512
K_THREAD_STACK_DEFINE(keystate_thread_stack, KEYSTATE_THREAD_STACK_SIZE);
#define UART_THREAD_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
static struct k_thread keystate_thread;
static struct k_thread uart_thread;

static void keystate_thread_handler(void *p1, void *p2, void *p3);
static void uart_thread_handler(void *p1, void *p2, void *p3);
static void process_keystate_packet(struct keystate_packet *packet);

static bool initialize_uart(void);
static bool initialize_gazell(void);
static void fetch_gazell_packet(struct gzll_rx_result *rx_result);
void update_keystate(uint32_t pipe, uint8_t *data_payload, size_t row, size_t column);

static void gzll_work_handler(struct k_work *work);

int main(void)
{
    uint8_t rx_byte;
    k_work_init(&gzll_work, gzll_work_handler);

    // Start keystate processing thread
    k_thread_create(&keystate_thread, keystate_thread_stack,
                    KEYSTATE_THREAD_STACK_SIZE,
                    keystate_thread_handler,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

    // Start UART processing thread
    k_thread_create(&uart_thread, uart_thread_stack,
                    UART_THREAD_STACK_SIZE,
                    uart_thread_handler,
                    NULL, NULL, NULL,
                    K_PRIO_PREEMPT(0), 0, K_NO_WAIT);

    if (!initialize_gazell())
    {
        return 0;
    }
    if (!initialize_uart())
    {
        return 0;
    }

    while (true)
    {
        if (k_sem_take(&main_sem, K_FOREVER)) {
            continue;
        }
        k_sleep(K_USEC(10));
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
    // ack right pipe
    success = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER_RIGHT, ack_payload, TX_PAYLOAD_LENGTH);
    if (!success) {
        return false;
    }
    // ack left pipe
    success = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER_LEFT, ack_payload, TX_PAYLOAD_LENGTH);
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
    int err;
    struct gzll_rx_result rx_result;

    rx_result.pipe = pipe;
    rx_result.info = rx_info;
    err = k_msgq_put(&gzll_msgq, &rx_result, K_NO_WAIT);
    if (err == 0) {
        // Handle error
        k_work_submit(&gzll_work);
    }
}

static void fetch_gazell_packet(struct gzll_rx_result *rx_result)
{
    struct keystate_packet pkt;
    uint32_t payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

    // Fetch packet from Gazell RX FIFO
    bool success = nrf_gzll_fetch_packet_from_rx_fifo(rx_result->pipe, pkt.data_payload, &payload_length);
    if (!success) {
        LOG_INF("RX fifo error");
    } else if (payload_length > 0) {
        LOG_INF("Received data on pipe %u: 0x%02x 0x%02x 0x%02x", rx_result->pipe, pkt.data_payload[0], pkt.data_payload[1], pkt.data_payload[2]);
    }

    // Queue the packet for keystate processing in the dedicated thread
    pkt.pipe = rx_result->pipe;
    pkt.payload_length = payload_length;
    k_msgq_put(&keystate_msgq, &pkt, K_NO_WAIT);

    // Load ACK payload into TX queue for the pipe immediately to keep Gazell responsive
    ack_payload[0] = 0xFF; // Arbitrary ACK payload
    nrf_gzll_add_packet_to_tx_fifo(rx_result->pipe, ack_payload, TX_PAYLOAD_LENGTH);
}

/*
Update keystate buffer based on received data payload for a given key position
pipe indicates which half of the keyboard the data is from, row and column indicate the key position within that half
inline the function to avoid function call overhead since this will be called for every key in the received payload
*/
inline void update_keystate(uint32_t pipe, uint8_t *data_payload, size_t row, size_t column)
{
    // in the source data, the 0th column is the most significant bit
    size_t key_offset = row * KEYS_PER_ROW_HALF + column;
    size_t byte_index = key_offset / 8;
    size_t bit_index = 7 - (key_offset % 8); // MSB
    // size_t column_num = (KEYS_PER_ROW_HALF - 1) - column;
    // size_t key_index = row * KEYS_PER_ROW_HALF + column_num;
    // size_t byte_index = key_index / 8;
    // size_t bit_index = 7 - (key_index % 8);

    // Extract bit value for the key from the received data payload
    bool bit_value = (data_payload[byte_index] >> bit_index) & 1;

    /*
    Each row half is a byte, and columns are bits within that byte packed to the lower bits of the byte.
    Column 0 is the least significant bit, column 5 is the most significant bit for a half row of 6 keys.
    This leaves some unused bits in each byte which we use to design a termination bit pattern that
    will never appear in actual key data to indicate end of frame to the host.
    */
    // Each row takes 2 bytes in the keystate buffer, left half is in even bytes and right half is in odd bytes
    // Find the byte for the key based on row, and set/clear the bit (column) based on the received data
    size_t keystate_index = row * 2 + (pipe == PIPE_NUMBER_LEFT ? 0 : 1); 
    keystate[keystate_index] &= ~(1 << column); // Clear bit in keystate buffer
    keystate[keystate_index] |= (bit_value << column); // Set bit in keystate buffer based on received data
}

static void gzll_work_handler(struct k_work *work)
{
    struct gzll_rx_result rx_result;

    while(!k_msgq_get(&gzll_msgq, &rx_result, K_NO_WAIT)) {
        fetch_gazell_packet(&rx_result);
    }
    k_sem_give(&main_sem);
}

/*
Dedicated thread for processing keystate updates from received packets.
This runs in its own thread to prevent blocking Gazell packet processing.
*/
static void keystate_thread_handler(void *p1, void *p2, void *p3)
{
    struct keystate_packet packet;

    while (true) {
        // Wait for a keystate packet to process
        if (k_msgq_get(&keystate_msgq, &packet, K_FOREVER) == 0) {
            process_keystate_packet(&packet);
        }
    }
}

static void uart_thread_handler(void *p1, void *p2, void *p3)
{
    uint8_t rx_byte;
    bool uart_initialized = initialize_uart();
    uint8_t tx_frame[TOTAL_DATA_BYTES + 1] = {0}; // +1 for stop byte

    if (!uart_initialized)
    {
        LOG_ERR("Failed to initialize UART");
        return;
    }
    while(true)
    {
        if (uart_poll_in(uart_dev, &rx_byte) == 0)
        {
            if (rx_byte == 's')
            {
                // Send keystates to host in a single frame
                k_spinlock_key_t key = k_spin_lock(&key_state_lock);
                memcpy(tx_frame, keystate, TOTAL_DATA_BYTES);
                k_spin_unlock(&key_state_lock, key);
                tx_frame[TOTAL_DATA_BYTES] = STOP_BYTE; // Append stop byte to indicate end of frame
                for (size_t i = 0; i < sizeof(tx_frame); i++)
                {
                    uart_poll_out(uart_dev, tx_frame[i]);
                }
            }
        }
        k_sleep(K_USEC(10));
    }
}

/*
Process a single keystate packet by updating the keystate buffer.
Data is received per side of the keyboard, with each key represented by a bit. 
Each side is in row major order. The keystate buffer is represented in a combined 
row major order for the whole keyboard.
*/
static void process_keystate_packet(struct keystate_packet *packet)
{
    // LOG_INF("Processing keystate packet");
    k_spinlock_key_t key = k_spin_lock(&key_state_lock);
    for (size_t i = 0; i < FULL_ROWS; i++)
    {
        for (size_t j = 0; j < KEYS_PER_ROW_HALF; j++)
        {
            update_keystate(packet->pipe, packet->data_payload, i, j);
        }
    }
    for (size_t j = 0; j < KEYS_FINAL_ROW_HALF; j++)
    {
        update_keystate(packet->pipe, packet->data_payload, FULL_ROWS, j);
    }
    k_spin_unlock(&key_state_lock, key);
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}
