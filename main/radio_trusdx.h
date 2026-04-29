#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t radio_trusdx_begin_ft8_tx(const uint8_t tones[79],
                                    int base_hz,
                                    int64_t slot_start_ms,
                                    int64_t now_ms);
void radio_trusdx_cancel_ft8_tx(void);
bool radio_trusdx_ft8_tx_active(void);
bool radio_trusdx_ft8_tx_done(void);
bool radio_trusdx_ft8_tx_failed(void);
void radio_trusdx_clear_ft8_tx_result(void);

#ifdef __cplusplus
}
#endif
