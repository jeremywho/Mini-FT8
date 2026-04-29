#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define TRUSDX_CAT_BAUD 115200
#define TRUSDX_RX_SAMPLE_RATE 7825
#define TRUSDX_TX_SAMPLE_RATE 11520

#ifdef __cplusplus
extern "C" {
#endif

bool trusdx_serial_start(void);
void trusdx_serial_stop(void);
bool trusdx_serial_is_streaming(void);
bool trusdx_serial_is_ready(void);
bool trusdx_serial_streaming_mode_active(void);

const char* trusdx_serial_get_status_string(void);
const char* trusdx_serial_get_debug_line1(void);
const char* trusdx_serial_get_debug_line2(void);

esp_err_t trusdx_serial_send_cat(const char* cmd, uint32_t timeout_ms);
esp_err_t trusdx_serial_set_tune(bool enable);

esp_err_t trusdx_serial_begin_ft8_tx(const uint8_t tones[79],
                                     int base_hz,
                                     int64_t slot_start_ms,
                                     int64_t now_ms);
void trusdx_serial_cancel_ft8_tx(void);
bool trusdx_serial_ft8_tx_active(void);
bool trusdx_serial_ft8_tx_done(void);
bool trusdx_serial_ft8_tx_failed(void);
void trusdx_serial_clear_ft8_tx_result(void);

#ifdef __cplusplus
}
#endif
