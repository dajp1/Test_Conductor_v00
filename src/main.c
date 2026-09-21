#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_timer.h"

static const char *TAG = "TEST_MUSICIAN";

#define LED_STRIP_GPIO_PIN 38  // Your working GPIO pin mapping
#define LED_STRIP_NUM_LEDS 1
#define LED_FLASH_TIME_MS 50
#define UART_PORT_NUM UART_NUM_1
// Original version: #define UART_TX_PIN 43
// Original version: #define UART_RX_PIN 44
// Non-working version: #define UART_TX_PIN 20
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD_RATE 115200
#define UART_RX_BUFFER_SIZE 256
#define MUSIC_QUEUE_SIZE 256

// Global handle for the LED strip
static led_strip_handle_t led_strip = NULL;

// There needs to be buffers for the uart rx data:
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint16_t uart_rx_bytes_received = 0;

// There's a large circular buffer than holds all of the information about
// bars, beats, time signatures, keys, chord, and root notes.
//
// The data structure is designed so that a bar can only have one tempo and
// time signature, but chords and root notes can change on each beat.  The 
// size of the buffer is 256 bars (for ease of calculation).  This should be 
// more that enough to keep going until it receives a new message from the 
// conductor.

typedef struct {
    uint8_t beat;
    uint8_t chord_root;  // 0 - 16 only
    uint8_t chord_type;
    uint8_t bass_note;   // 0 - 16 only
    uint8_t legato;      // 0 - 16 only
    uint8_t dynamic;     // 0 - 16 only
} beat_state_t;

typedef struct {
    uint16_t bar;
    uint8_t tempo;
    uint8_t time_sig_numerator;
    uint8_t time_sig_denominator;
    uint8_t key;
    uint8_t key_type;
    beat_state_t beats[16];  // Maximum of 16 beats per bar, can be adjusted as needed
} bar_state_t;

bar_state_t music_queue[MUSIC_QUEUE_SIZE];  // Circular buffer for music state
uint16_t currentBar = 0;  // Track the current bar number for processing
uint8_t currentBeat = 0;  // Track the current beat number for processing
uint8_t currentTempo = 120;  // Default tempo in BPM, can be updated based on incoming messages
uint8_t currentTimeSigNumerator = 4;  // Default time signature numerator
uint8_t currentTimeSigDenominator = 4;  // Default time signature denominator
uint8_t currentKey = 0;  // Default key, can be updated based on incoming messages
uint8_t currentKeyType = 0;  // Default key, can be updated based on incoming messages
uint8_t currentChordRoot = 0;  // Default chord, can be updated based on incoming messages
uint8_t currentChordType = 0;  // Default chord, can be updated based on incoming messages
uint8_t currentBassNote = 0;  // Default root note, can be updated based on incoming messages
uint8_t currentLegato = 0;  // Default legato, can be updated based on incoming messages
uint8_t currentDynamic = 0;  // Default dynamic, can be updated based on incoming messages
uint8_t currentMusicQueueIndex = 0;  // Index for the circular buffer
uint64_t currentBarStartTime = 0;  // Timestamp for the start of the current bar
uint16_t lastSyncBar = 0;  // Last bar number that was synchronized
uint64_t lastSyncTime = 0;  // Timestamp for the last synchronization event

// There's an array of LED colours:
typedef struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} colour_t;
colour_t LED_colours[8];

// Data structures to hold the conductor messages.  Given how small
// the overhead is in sending another few bytes, I'll make the messages
// a constant size for simplicity, holding four things of data each.
// Type can be 'S', 'K', 'T', 'C', 'D' or 'G'.  Anything else is ignored.
typedef struct __attribute__((packed)) {
    uint8_t beat;
    uint8_t type;
    uint8_t dataA[2];
} baton_data_t;

typedef struct __attribute__((packed)) {
    uint16_t address;
    uint16_t bar;
    baton_data_t baton_data[4];
} baton_msg_t;

// Data structure.  MUST perfectly match the transmitter's byte layout:
typedef struct __attribute__((packed)) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    char status_msg[16]; 
} colour_msg_t;

// Some function prototypes for the functions that will be defined later in the code:
static void init_music_queue(void);
static void add_default_tune_to_music_queue(void);
static void update_music_queue(const baton_msg_t *msg);
static void on_data_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
static void uart_send_bytes(const uint8_t *data, size_t len);
static void on_uart_data_recv(const uint8_t *data, size_t len);
static void uart_rx_task(void *arg);
static void init_uart(void);
static void init_esp_now(void);
static void init_led_colours(void);

static void init_led_colours(void) {
    LED_colours[0].r = 32; LED_colours[0].g = 0; LED_colours[0].b = 0;
    LED_colours[1].r = 0; LED_colours[1].g = 8; LED_colours[1].b = 0;
    LED_colours[2].r = 0; LED_colours[2].g = 0; LED_colours[2].b = 8;
    LED_colours[3].r = 4; LED_colours[3].g = 4; LED_colours[3].b = 0;
    LED_colours[4].r = 4; LED_colours[4].g = 0; LED_colours[4].b = 4;
    LED_colours[5].r = 0; LED_colours[5].g = 4; LED_colours[5].b = 4;
    LED_colours[6].r = 4; LED_colours[6].g = 4; LED_colours[6].b = 4;
    LED_colours[7].r = 0; LED_colours[7].g = 0; LED_colours[7].b = 0;
}

// Some functions to manage the music queue and update the current state based on incoming messages
static void init_music_queue(void) {
    memset(music_queue, 0, sizeof(music_queue));

    for (int i = 0; i < MUSIC_QUEUE_SIZE; i++) {
        music_queue[i].bar = i;  // Initialize bar numbers sequentially
        music_queue[i].tempo = 120;  // Default tempo
        music_queue[i].time_sig_numerator = 6;  // Default time signature numerator
        music_queue[i].time_sig_denominator = 8;  // Default time signature denominator
        for (int j = 0; j < 16; j++) {
            music_queue[i].beats[j].beat = j + 1;  // Beat numbers start from 1
            music_queue[i].beats[j].chord_root = 0;  // Default chord (C)
            music_queue[i].beats[j].chord_type = 0;  // Default chord type (major)
            music_queue[i].beats[j].bass_note = 0;  // Default bass note (C)
        }
    }

    add_default_tune_to_music_queue();

    currentBar = music_queue[0].bar;
    currentBeat = 0;
    currentTempo = music_queue[0].tempo;
    currentTimeSigNumerator = music_queue[0].time_sig_numerator;
    currentTimeSigDenominator = music_queue[0].time_sig_denominator;
    currentKey = music_queue[0].key;
    currentKeyType = music_queue[0].key_type;
    currentChordRoot = music_queue[0].beats[0].chord_root;
    currentChordType = music_queue[0].beats[0].chord_type;
    currentBassNote = music_queue[0].beats[0].bass_note;
    currentLegato = music_queue[0].beats[0].legato;
    currentDynamic = music_queue[0].beats[0].dynamic;
    currentMusicQueueIndex = 0;
}

static void add_default_tune_to_music_queue(void) {

    // For now, just try this one.  First, defaults for 6/8 in G major.
    for (uint32_t u = 0; u < MUSIC_QUEUE_SIZE; u++) {
        music_queue[u].bar = u;
        music_queue[u].tempo = 60;  // Default tempo
        music_queue[u].time_sig_numerator = 6;  // Default time signature numerator
        music_queue[u].time_sig_denominator = 8;  // Default time signature denominator
        music_queue[u].key = 7;  // Default key (G)
        music_queue[u].key_type = 0;  // Default key type (major)

        for (uint32_t v = 0; v < 16; v++) {
            music_queue[u].beats[v].beat = v + 1;  // Beat numbers start from 1
            music_queue[u].beats[v].chord_root = 0;  // Default chord no change
            music_queue[u].beats[v].chord_type = 0;  // Default chord type change
            music_queue[u].beats[v].bass_note = 0;   // Default bass no change
            music_queue[u].beats[v].legato = 0;  // Default legato no change
            music_queue[u].beats[v].dynamic = 0;  // Default dynamic no change
        }
    }

    uint8_t chord_roots[32] = {
        8, 5, 8, 5, 1, 3, 8, 3, 
        8, 1, 5, 1, 3, 12, 5, 5, 
        1, 1, 5, 5, 1, 1, 8, 3, 
        8, 5, 8, 3, 8, 5, 1, 3
    };
    uint8_t chord_types[32] = {
        1, 2, 1, 2, 1, 1, 1, 7,
        1, 1, 2, 1, 1, 7, 2, 2,
        1, 1, 2, 2, 1, 1, 1, 7,
        1, 2, 1, 1, 1, 2, 1, 7
    };
    uint8_t bass_notes[32] = {
        8, 8, 8, 8, 1, 3, 8, 8,
        8, 1, 5, 8, 10, 12, 8, 5,
        1, 1, 5, 5, 1, 1, 8, 3,
        8, 8, 8, 12, 8, 5, 1, 3
    };

    // Then fill up with the chord structure:
    for (uint32_t w = 0; w < 256; w++) {
        music_queue[w].beats[0].chord_root = chord_roots[w % 32];
        music_queue[w].beats[0].chord_type = chord_types[w % 32];
        music_queue[w].beats[0].bass_note = bass_notes[w % 32];
        music_queue[w].beats[0].legato = 10;     // Legato = 10
        music_queue[w].beats[0].dynamic = 8;     // Dynamic = 8

        // One bar has a change of chord in the middle:
        if (w % 32 == 9) {
            music_queue[w].beats[3].chord_root = 3;
            music_queue[w].beats[3].chord_type = 1;
            music_queue[w].beats[3].bass_note = 3;
        }
    }
}

static void update_music_queue(const baton_msg_t *msg) {
    // Update the music queue based on the incoming baton message.
    // First check that the bar number is within the valid range of the circular buffer,
    // and if so, find the index in the music_queue array corresponding to that bar number.
    int queue_index = msg->bar % MUSIC_QUEUE_SIZE;  // Simple modulo-based indexing for circular buffer
    if (music_queue[queue_index].bar != msg->bar) {
        // If the bar number doesn't match, we can either log a warning or handle it as needed.
        ESP_LOGW(TAG, "Bar number mismatch: %d not in range from %d to %d", msg->bar, music_queue[0].bar, music_queue[MUSIC_QUEUE_SIZE - 1].bar);
        return;
    }

    // Now update the tempo, time signature, and beat information based on the baton data.
    for (int i = 0; i < 4; i++) {
        if (msg->baton_data[i].beat == 0) {
            // If the beat number is zero, it applies to the entire bar.  The tempo and 
            // time signature can only be updated here.
            if (msg->baton_data[i].type == 'T') {
                music_queue[queue_index].tempo = msg->baton_data[i].dataA[0];  // Update tempo
            } else if (msg->baton_data[i].type == 'S') {
                // This is a synchronisation message, the current bar starts now.
                currentBar = msg->bar;
                currentBeat = 0;
                currentTempo = music_queue[currentBar % MUSIC_QUEUE_SIZE].tempo;
                currentTimeSigNumerator = music_queue[currentBar % MUSIC_QUEUE_SIZE].time_sig_numerator;
                currentTimeSigDenominator = music_queue[currentBar % MUSIC_QUEUE_SIZE].time_sig_denominator;
                currentBarStartTime = esp_timer_get_time();  // Record the start time of the current bar
                // Send a synchronisation message to the UART:
                // XXX edit this to be identical to the one sent by the main while(1) loop:
                uint8_t sync_msg[6];
                sync_msg[0] = 250;  // Sync message type
                sync_msg[1] = currentBar >> 8;  // High byte of bar number
                sync_msg[2] = currentBar & 0xFF;  // Low byte of bar number
                sync_msg[3] = currentTempo;
                sync_msg[4] = currentTimeSigNumerator;
                sync_msg[5] = currentTimeSigDenominator;   
                // XXX Testing XXX uart_send_bytes(sync_msg, sizeof(sync_msg));                
            } else if (msg->baton_data[i].type == 'G') {
                music_queue[queue_index].time_sig_numerator = msg->baton_data[i].dataA[0];  // Update time signature numerator
                music_queue[queue_index].time_sig_denominator = msg->baton_data[i].dataA[1];  // Update time signature denominator
            } else if (msg->baton_data[i].type == 'K') {
                music_queue[queue_index].key = msg->baton_data[i].dataA[0];  // Update key for the first beat
                music_queue[queue_index].key_type = msg->baton_data[i].dataA[1];  // Update key type for the first beat
            } else if (msg->baton_data[i].type == 'C') {
                for (int j = 0; j < 16; j++) {
                    music_queue[queue_index].beats[j].chord_root = msg->baton_data[i].dataA[0] >> 4;
                    music_queue[queue_index].beats[j].bass_note = msg->baton_data[i].dataA[0] & 0x0f;
                    music_queue[queue_index].beats[j].chord_type = msg->baton_data[i].dataA[1];
                }
            }

            music_queue[queue_index].tempo = msg->baton_data[i].dataA[0];
            music_queue[queue_index].time_sig_numerator = msg->baton_data[i].dataA[1];
            music_queue[queue_index].time_sig_denominator = msg->baton_data[i].dataA[2];
        }
        else if (msg->baton_data[i].beat <= 16) {
            // Update the chord and root note information only for the specified beat number:
            music_queue[queue_index].beats[msg->baton_data[i].beat - 1].chord_root = msg->baton_data[i].dataA[0] >> 4;
            music_queue[queue_index].beats[msg->baton_data[i].beat - 1].bass_note = msg->baton_data[i].dataA[0] & 0x0f;
            music_queue[queue_index].beats[msg->baton_data[i].beat - 1].chord_type = msg->baton_data[i].dataA[1];
        }
    }
}

// ESP-NOW Receive Callback for modern ESP-IDF (v5.5+)
static void on_data_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    // Validate that incoming data matches our expected struct size
    if (len == sizeof(baton_msg_t)) {
        // Cast the raw byte data into our structured layout
        baton_msg_t *incoming_msg = (baton_msg_t *)data;

        ESP_LOGI(TAG, "Packet from node: %d (MAC: " MACSTR ") | Bar: %d", 
             incoming_msg->address, MAC2STR(recv_info->src_addr), incoming_msg->bar);

        for (int i = 0; i < 4; i++) {
            ESP_LOGI(TAG, "Baton %d - Type: %c | DataA: %02X%02X", 
                 i, incoming_msg->baton_data[i].type, 
                 incoming_msg->baton_data[i].dataA[0], 
                 incoming_msg->baton_data[i].dataA[1]);
        }

        // Now there's some stuff to do to get this new data into the music queue.
        // Chances it's not too far in the future, so I'll start looking from the
        // current queue head and move forward.
        update_music_queue(incoming_msg);
    }
    else if (len == sizeof(colour_msg_t)) {
        // Cast the raw byte data into our structured layout
        colour_msg_t *incoming_msg = (colour_msg_t *)data;

        ESP_LOGI(TAG, "Packet from " MACSTR " - Msg: %s | R:%d G:%d B:%d", 
             MAC2STR(recv_info->src_addr), incoming_msg->status_msg, 
             incoming_msg->r, incoming_msg->g, incoming_msg->b);

        // Update the local RGB LED based on incoming instruction
        if (led_strip != NULL) {
            led_strip_set_pixel(led_strip, 0, incoming_msg->r, incoming_msg->g, incoming_msg->b);
            led_strip_refresh(led_strip);
        }
    }
    else {
        ESP_LOGW(TAG, "Received packet size mismatch! Expected %d, got %d", sizeof(baton_msg_t), len);
    }
    return;
}

static void uart_send_bytes(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
        return;
    }

    size_t written = 0;
    while (written < len) {
        int bytes_written = uart_write_bytes(UART_PORT_NUM, (const char *)(data + written), len - written);
        if (bytes_written <= 0) {
            ESP_LOGE(TAG, "UART write failed");
            return;
        }
        written += (size_t)bytes_written;
    }
}

static void on_uart_data_recv(const uint8_t *data, size_t len) {
    if (data == NULL || len == 0) {
        return;
    }

    ESP_LOGI(TAG, "on_uart_data_recv called, %d bytes received:", len);
    for (size_t i = 0; i < len; i++) {
        // ESP_LOGI(TAG, "[%zu] = 0x%02X", i, data[i]);
    }

    // All serial messages start with 0xFX where X is the length of the message.
    // (Numbers this large should not occur in the data stream).  What can happen
    // is that this function is called before an entire message is received, so 
    // we need to buffer the data until we have a complete message.
    if (uart_rx_bytes_received == 0) {
        // There is nothing currently in the buffer.  Ignore anything in the 
        // incoming data up to the start of a message, then copy the rest into 
        // the buffer.
        size_t start_index = 0;
        while (start_index < len && data[start_index] <= 240) {
            start_index++;
        }
        for (size_t i = start_index; i < len && uart_rx_bytes_received < UART_RX_BUFFER_SIZE; i++) {
            uart_rx_buffer[uart_rx_bytes_received++] = data[i];
        }
    }
    else {
        // There is already data in the buffer.  Copy the incoming data into the 
        // buffer until it is full or we have copied all of the incoming data.
        for (size_t i = 0; i < len && uart_rx_bytes_received < UART_RX_BUFFER_SIZE; i++) {
            uart_rx_buffer[uart_rx_bytes_received++] = data[i];
        }

        // The buffer should never fill up.  If it does, something has gone wrong,
        // so clear the buffer and start again.
        if (uart_rx_bytes_received == UART_RX_BUFFER_SIZE) {
            ESP_LOGW(TAG, "on_uart_data_recv buffer full!");
            uart_rx_bytes_received = 0;
        }
    }

    // Now check if we have a complete message in the buffer.  The first byte
    // should be 24x where x is the length of the message.  If we have enough 
    // bytes in the buffer, process the message and reset the buffer index.
    if (uart_rx_bytes_received > 0 && uart_rx_buffer[0] >= 240) {
        size_t message_length = uart_rx_buffer[0] - 240;
        if (uart_rx_bytes_received >= message_length) {
            // A complete message has arrived, and can be processed.
            ESP_LOGI(TAG, "Complete UART message received (%zu bytes):", message_length);
            for (size_t i = 0; i < message_length; i++) {
                ESP_LOGI(TAG, "Rx msg byte [%zu] = 0x%02X", i, uart_rx_buffer[i]);
            }

            // Currently, only three messages are accepted: one specifying a bar number, 
            // one specifying a beat number, and one specifying the colour of the LED.
            // The first byte of the message is the message type, and the rest of the bytes 
            // are the data.  Formats for each type of message are as follows:
            //   0x01: Bar number (2 bytes data, big-endian) 
            //       Five-byte reply: tempo, key, key type, time signature numerator, time signature denominator
            //   0x02: Bar number (2 bytes bar, big-endian, then beat number (1 byte)
            //       Three-byte reply: chord root, chord type, bass note
            //   0x03: LED colour (3 bytes, r, g & b)
            //       There is no reply to this message, but the LED colour is updated immediately.

            if (uart_rx_buffer[1] == 0x01 && message_length == 4) {
                // Bar number message
                uint16_t bar_number = (uart_rx_buffer[2] << 8) | uart_rx_buffer[3];
                ESP_LOGI(TAG, "Received Bar Number: %d", bar_number);

                // Send the reply message with tempo, key, key type, time signature numerator, and time signature denominator
                uint8_t reply[11];
                int queue_index = bar_number % MUSIC_QUEUE_SIZE;

                reply[0] = 240 + 11;  // Starting delimiter and length;
                reply[1] = 0x01; // Sync message type: regular bar sync
                reply[2] = 128 + (bar_number >> 8);  // High byte of bar number
                reply[3] = bar_number & 0xFF;  // Low byte of bar number
                reply[4] = music_queue[queue_index].tempo;
                reply[5] = music_queue[queue_index].time_sig_numerator;
                reply[6] = music_queue[queue_index].time_sig_denominator;
                reply[7] = music_queue[queue_index].key << 4
                    | music_queue[queue_index].key_type;

                // Then make a compressed version of information about
                // the first beat in the bar, and send that too:
                reply[8] = music_queue[queue_index].beats[0].chord_root << 4 
                    | music_queue[queue_index].beats[0].bass_note;
                reply[9] = music_queue[queue_index].beats[0].chord_type;
                reply[10] = music_queue[queue_index].beats[0].dynamic << 4
                    | music_queue[queue_index].beats[0].legato;

                uart_send_bytes(reply, sizeof(reply));
            }
            else if (uart_rx_buffer[1] == 0x03 && message_length == 5) {
                // Beat info request message
                uint16_t bar_number = (uart_rx_buffer[2] << 8) | uart_rx_buffer[3];
                uint8_t beat_number = uart_rx_buffer[4];
                ESP_LOGI(TAG, "Received Bar Number: %d, Beat Number: %d", bar_number, beat_number);

                // Send the reply message with chord root, chord type, bass note,
                // dynamic and legato information:
                uint8_t reply[6];
                int queue_index = bar_number % MUSIC_QUEUE_SIZE;
                if (beat_number > 0 && beat_number <= 16) {
                    uint8_t chord_root = music_queue[queue_index].beats[beat_number - 1].chord_root;
                    uint8_t bass_note = music_queue[queue_index].beats[beat_number - 1].bass_note;
                    uint8_t chord_type = music_queue[queue_index].beats[beat_number - 1].chord_type;
                    uint8_t legato = music_queue[queue_index].beats[beat_number - 1].legato;
                    uint8_t dynamic = music_queue[queue_index].beats[beat_number - 1].dynamic;
                    reply[0] = 245;
                    reply[1] = 0x03;
                    reply[2] = beat_number;
                    reply[3] = ((chord_root & 0x0f) << 4) | bass_note;
                    reply[4] = chord_type;
                    reply[5] = ((dynamic & 0x0f) << 4) | legato;
                    uart_send_bytes(reply, sizeof(reply));
                } else {
                    ESP_LOGW(TAG, "Invalid beat number: %d", beat_number);
                }
            }
            else if (uart_rx_buffer[1] == 0x03 && message_length == 5) {
                // LED colour message
                uint8_t r = uart_rx_buffer[2];
                uint8_t g = uart_rx_buffer[3];
                uint8_t b = uart_rx_buffer[4];
                ESP_LOGI(TAG, "Received LED Colour: R=%d, G=%d, B=%d", r, g, b);

                // Update the local RGB LED based on incoming instruction
                if (led_strip != NULL) {
                    led_strip_set_pixel(led_strip, 0, r, g, b);
                    led_strip_refresh(led_strip);
                }
            }
            else {
                ESP_LOGW(TAG, "Unknown UART message type or invalid length");
                uint8_t reply[5] = {'E', 'r', 'r', 'o', 'r'};
                uart_send_bytes(reply, sizeof(reply));
            }

            // Reset the buffer index for the next message.  Note that this could 
            // in theory go wrong if the next message arrives before the current 
            // message is fully processed, but that should be very unlikely given 
            // the message processing time and the expected message frequency.
            uart_rx_bytes_received = 0;
        }
    }
}

static void uart_rx_task(void *arg) {
    uint8_t buffer[UART_RX_BUFFER_SIZE];

    while (1) {
        // Note: this might impose a short delay, as it can wait for pdMS_TO_TICKS(10) before
        // returning to see if any more data arrives.  Setting this to pdMS_TO_TICKS(5) crashes
        // the program with a watchdog timeout error for this process.  There must be a better
        // way to do this, but for now I can live with a 10 ms delay; there are longer delays
        // elsewhere which need addressing first.
        int len = uart_read_bytes(UART_PORT_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
        if (len > 0) {
            on_uart_data_recv(buffer, (size_t)len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void init_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG, "Initialising UART%d at %d baud, tx pin %d", UART_PORT_NUM, UART_BAUD_RATE, UART_TX_PIN);
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // ESP_LOGI(TAG, "Does this line print one?");
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_LOGI(TAG, "Does this line print two?");
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUFFER_SIZE * 2, UART_RX_BUFFER_SIZE * 2, 20, NULL, 0));
    // ESP_LOGI(TAG, "Does this line print three?");
 
    // There's an issue with creating the receive task.  It crashes UART0 and the ESP_LOGx functions.
    // I'm not sure why, since they are different UARTs.
    //
    // Perhaps try a flush of the buffers first, in case they have filled up with zeros during the
    // reset phase?
    ESP_ERROR_CHECK(uart_flush(UART_PORT_NUM));
    
    // Set timeout in symbol periods (e.g., 10 symbols after the last byte), otherwise it waits
    // for the buffer to fill up before calling the handler to do any processing: 
    // uart_set_rx_timeout(UART_PORT_NUM, 10);
    
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART%d initialized at %d baud", UART_PORT_NUM, UART_BAUD_RATE);
}

// System initialization for Wi-Fi and ESP-NOW
static void init_esp_now(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started in Station mode successfully.");

    // Initialize ESP-NOW protocol layer
    ESP_ERROR_CHECK(esp_now_init());
    
    // Register our data processing function
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv_cb));
    ESP_LOGI(TAG, "ESP-NOW Receiver initialized successfully.");
}

void app_main(void) {
    // Pause for monitor to connect:
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Starting Receiver Application");
    ESP_LOGI(TAG, "About to initialise UART");
    init_uart();
    ESP_LOGI(TAG, "UART initialised");

    init_led_colours();

    // Configure the onboard LED device
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_NUM_LEDS,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, 
        .led_model = LED_MODEL_WS2812,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 0, // Auto-resolve hardware clocks safely
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "LED strip initialized");

    // Initialise the music_queue array and current bar/beat/tempo tracking variables
    init_music_queue();

    // Start network loops
    init_esp_now();

    uint8_t hello[] = {'H', 'i', '\r', '\n'};
    // uart_send_bytes(hello, sizeof(hello));

    // The main execution task flashes the LEDs around a sequence of colours,
    // in time with the beats in each bar.  It also keeps track of the current
    // bar and beat, based on the times since the last sync message was received.
    uint64_t lastSyncBarStartTime = 0;
    uint64_t nextBeatTime = 0;
    uint64_t nextLEDOffTime = 0;
    bool LEDsNowOn = false;

    while (1) {
        uint64_t currentTime = esp_timer_get_time();

        // 10 ms resolution should be enough to look synchronised.
        while (currentTime < nextBeatTime) {
            vTaskDelay(pdMS_TO_TICKS(20));
            currentTime = esp_timer_get_time();

            // If the LED flash time is over, turn off the LEDs
            if (LEDsNowOn && currentTime > nextLEDOffTime){
                led_strip_set_pixel(led_strip, 0, 0, 0, 0);
                led_strip_refresh(led_strip);
                LEDsNowOn = false;            
            }
        }

        // The current beat has just been passed, so I need to work out the next beat
        // time.  For this, I need to work out the current bar and beat based on the 
        // current time, the last sync message, and the tempo/time signature in all 
        // bars since the last sync message.  This is a bit tricky, but I should be 
        // able to do it by keeping track of the current bar and beat, and the time 
        // of the last sync message.  This should be quick if the last sync message
        // was recent, but could be slow if it was a long time ago.
        uint16_t working_bar = lastSyncBar;
        uint64_t beat_time = lastSyncBarStartTime;
        uint16_t working_beat = 0;

        bool found_next_beat = false;
        while (!found_next_beat) {
            uint16_t workingBarTempo = music_queue[working_bar % MUSIC_QUEUE_SIZE].tempo;
            uint16_t workingBarTimeSigNumerator = music_queue[working_bar % MUSIC_QUEUE_SIZE].time_sig_numerator;
            uint16_t workingBarTimeSigDenominator = music_queue[working_bar % MUSIC_QUEUE_SIZE].time_sig_denominator;
            uint16_t beatsInWorkingBar = workingBarTimeSigNumerator;
            uint32_t beat_duration_us = (uint32_t)((60.0e6 / workingBarTempo) * (4.0f / workingBarTimeSigDenominator));
            uint64_t end_of_working_bar_time = beat_time + (uint64_t)(beatsInWorkingBar * beat_duration_us);

            if (currentTime < end_of_working_bar_time) {
                // The current time is within the working bar, so I can calculate the next beat time.
                uint32_t beats_since_bar_start = (uint32_t)((currentTime - beat_time) / beat_duration_us + 0.5);
                nextBeatTime = beat_time + ((beats_since_bar_start + 1) * beat_duration_us);
                working_beat = beats_since_bar_start;
                found_next_beat = true;
            } else {
                // The current time is beyond the working bar, so I need to move to the next bar.
                working_bar++;
                beat_time = end_of_working_bar_time;
            }
        }

        // Then it's just a matter of updating the LEDs with the colour corresponding to the
        // current beat:
        uint16_t led_index = working_beat % 8;
        if (led_strip != NULL) {
            led_strip_set_pixel(led_strip, 0, LED_colours[led_index].g, LED_colours[led_index].r, LED_colours[led_index].b);
            led_strip_refresh(led_strip);
            nextLEDOffTime = currentTime + LED_FLASH_TIME_MS * 1000;
            LEDsNowOn = true;
        }

        // currentChordRoot, currentChordType, currentBassNote, currentLegato, and currentDynamic need to be updated 
        // from the music queue on every beat (or left as they are if set to zero):
        uint32_t changes = 0;
        uint8_t newChordRoot = music_queue[working_bar % MUSIC_QUEUE_SIZE].beats[working_beat].chord_root;
        uint8_t newChordType = music_queue[working_bar % MUSIC_QUEUE_SIZE].beats[working_beat].chord_type;
        uint8_t newBassNote = music_queue[working_bar % MUSIC_QUEUE_SIZE].beats[working_beat].bass_note;
        uint8_t newLegato = music_queue[working_bar % MUSIC_QUEUE_SIZE].beats[working_beat].legato;
        uint8_t newDynamic = music_queue[working_bar % MUSIC_QUEUE_SIZE].beats[working_beat].dynamic;

        if (newChordRoot != currentChordRoot && newChordRoot != 0) {
            changes |= 0x01;
            currentChordRoot = newChordRoot;
        }
        if (newChordType != currentChordType && newChordType != 0) {
            changes |= 0x02;
            currentChordType = newChordType;
        }
        if (newBassNote != currentBassNote && newBassNote != 0) {
            changes |= 0x04;
            currentBassNote = newBassNote;
        }
        if (newLegato != currentLegato && newLegato != 0) {
            changes |= 0x08;
            currentLegato = newLegato;
        }
        if (newDynamic != currentDynamic && newDynamic != 0) {
            changes |= 0x10;
            currentDynamic = newDynamic;
        }

        // ESP_LOGI(TAG, "Bar %d, beat %d had changes: 0x%02X", working_bar, working_beat, (unsigned int)changes);

        // If this isn't the first beat in the bar but something has changed, then a beat update
        // message needs to be generated and sent:
        if (working_beat != 0 && changes != 0) {
            ESP_LOGI(TAG, "Sending beat data for bar %d, beat %d, with changes: 0x%02X", working_bar, working_beat, (unsigned int)changes);
            uint8_t beat_msg[6];
            beat_msg[0] = 240 + 6;  // Starting delimiter and length;
            beat_msg[1] = 0x02; // Beat message type: regular beat update
            beat_msg[2] = 128 + (working_bar >> 8);  // High byte of bar number
            beat_msg[3] = working_bar & 0xFF;  // Low byte of bar number
            beat_msg[4] = currentChordRoot << 4 | currentBassNote;
            beat_msg[5] = currentChordType;

            uart_send_bytes(beat_msg, sizeof(beat_msg));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Otherwise, on the first beat in every bar, send the data to the UART:
        // if (working_beat == 0) {
        if (working_beat == 0) {
            ESP_LOGI(TAG, "Sending bar data for bar %d, beat %d", working_bar, working_beat);
            uint8_t sync_msg[11];
            sync_msg[0] = 240 + 11;  // Starting delimiter and length;
            sync_msg[1] = 0x00; // Sync message type: regular bar sync
            sync_msg[2] = 128 + (working_bar >> 8);  // High byte of bar number
            sync_msg[3] = working_bar & 0xFF;  // Low byte of bar number
            sync_msg[4] = music_queue[working_bar % MUSIC_QUEUE_SIZE].tempo;
            sync_msg[5] = music_queue[working_bar % MUSIC_QUEUE_SIZE].time_sig_numerator;
            sync_msg[6] = music_queue[working_bar % MUSIC_QUEUE_SIZE].time_sig_denominator;
            sync_msg[7] = music_queue[working_bar % MUSIC_QUEUE_SIZE].key << 4
                | music_queue[working_bar % MUSIC_QUEUE_SIZE].key_type;

            // Then make a compressed version of information about
            // the first beat in the bar, and send that too.
            sync_msg[8] = currentChordRoot << 4 | currentBassNote;
            sync_msg[9] = currentChordType;
            sync_msg[10] = currentDynamic << 4 | currentLegato;

            uart_send_bytes(sync_msg, sizeof(sync_msg));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
