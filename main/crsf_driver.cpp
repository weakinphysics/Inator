#include "crsf_driver.h"


#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

static const char *TAG = "CRSF";

// -------- UART config --------
// Change these to suit your board.
#define CRSF_UART_NUM      UART_NUM_1
#define CRSF_UART_TX_PIN   17
#define CRSF_UART_RX_PIN   16
#define CRSF_BAUD          420000

// CRSF frame:
// [device_addr][length][type][payload...][crc]
// length = bytes from type through crc inclusive

typedef struct {
    uint8_t buf[CRSF_MAX_FRAME_SIZE];
    int idx;
    int expected_total;
} crsf_rx_state_t;

static crsf_rx_state_t s_rx;

// CRC8 polynomial 0xD5, init 0
static uint8_t crsf_crc8_byte(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
        else            crc <<= 1;
    }
    return crc;
}

static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc = crsf_crc8_byte(crc, *ptr++);
    }
    return crc;
}

// 0x16 frame payload = 22 bytes = 16 channels * 11 bits
static void crsf_decode_channels_0x16(const uint8_t payload[22], uint16_t ch_out[16])
{
    uint32_t bitbuf = 0;
    int bits = 0;
    int byte_i = 0;

    for (int ch = 0; ch < 16; ch++) {
        while (bits < 11) {
            bitbuf |= ((uint32_t)payload[byte_i++]) << bits;
            bits += 8;
        }

        uint16_t raw = bitbuf & 0x7FF; // 11 bits
        bitbuf >>= 11;
        bits -= 11;

        // Official mapping example:
        // us = ((raw - 992) * 5 / 8) + 1500
        // Gives ~172..1811 for full raw range, but around stick range this maps nicely.
        int32_t us = ((int32_t)raw - 992) * 5 / 8 + 1500;

        // Optional clamp
        if (us < 800)  us = 800;
        if (us > 2200) us = 2200;

        ch_out[ch] = (uint16_t)us;
    }
}

void crsf_init(void)
{
	uart_config_t uart_config{};
	uart_config.baud_rate = CRSF_BAUD;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.rx_flow_ctrl_thresh = 0;
	uart_config.source_clk = UART_SCLK_DEFAULT;
	uart_config.flags.backup_before_sleep = 0;
	uart_config.flags.allow_pd = 0;

    ESP_ERROR_CHECK(uart_driver_install(CRSF_UART_NUM, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CRSF_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CRSF_UART_NUM,
                                 CRSF_UART_TX_PIN,
                                 CRSF_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    memset(&s_rx, 0, sizeof(s_rx));
    ESP_LOGI(TAG, "CRSF UART initialized at %d baud", CRSF_BAUD);
}

static bool crsf_try_parse_frame(const uint8_t *frame, int total_len, crsf_channels_t *out)
{
    if (total_len < 5) {
        return false;
    }

    uint8_t device_addr = frame[0];
    uint8_t length      = frame[1];

    // total_len should be addr + len + length bytes
    if (total_len != (2 + length)) {
        return false;
    }

    uint8_t type = frame[2];
    const uint8_t *payload = &frame[3];
    uint8_t crc_rx = frame[total_len - 1];

    // CRC covers [type + payload], not addr/length
    uint8_t crc_calc = crsf_crc8(&frame[2], (uint8_t)(length - 1));
    if (crc_calc != crc_rx) {
        return false;
    }

    // Usually you'll see frames addressed to FC (0xC8), but don't be too strict.
    (void)device_addr;

    if (type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        // 0x16 payload must be 22 bytes
        if ((length - 2) != 22) {
            return false;
        }

        crsf_decode_channels_0x16(payload, out->ch);
        out->failsafe = false;
        out->last_frame_us = (uint32_t)esp_timer_get_time();
        return true;
    }

    return false;
}

bool crsf_poll(crsf_channels_t *out)
{
    uint8_t tmp[128];
    int n = uart_read_bytes(CRSF_UART_NUM, tmp, sizeof(tmp), 0);
    if (n <= 0) {
        return false;
    }

    bool got_channels = false;

    for (int i = 0; i < n; i++) {
        uint8_t b = tmp[i];

        if (s_rx.idx == 0) {
            // CRSF sync/device byte. For FC-facing stream this is typically 0xC8.
            // Some implementations just treat first byte as address and continue.
            s_rx.buf[s_rx.idx++] = b;
            s_rx.expected_total = 0;
            continue;
        }

        if (s_rx.idx == 1) {
            s_rx.buf[s_rx.idx++] = b;

            // total bytes = 2 header bytes + length
            s_rx.expected_total = 2 + b;

            if (s_rx.expected_total < 5 || s_rx.expected_total > CRSF_MAX_FRAME_SIZE) {
                s_rx.idx = 0;
                s_rx.expected_total = 0;
            }
            continue;
        }

        s_rx.buf[s_rx.idx++] = b;

        if (s_rx.expected_total > 0 && s_rx.idx == s_rx.expected_total) {
            crsf_channels_t parsed{};
            if (crsf_try_parse_frame(s_rx.buf, s_rx.expected_total, &parsed)) {
                *out = parsed;
                got_channels = true;
            }

            s_rx.idx = 0;
            s_rx.expected_total = 0;
        } else if (s_rx.idx >= CRSF_MAX_FRAME_SIZE) {
            s_rx.idx = 0;
            s_rx.expected_total = 0;
        }
    }

    return got_channels;
}