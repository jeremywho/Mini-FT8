#include "radio_trusdx.h"

#include <cstdio>
#include <cstring>

#include "audio_trusdx_serial.h"
#include "esp_log.h"
#include "radio_control_backend.h"

static const char* TAG = "RADIO_TRUSDX";

static esp_err_t trusdx_send_cmd(const char* cmd, uint32_t timeout_ms)
{
    return trusdx_serial_send_cat(cmd, timeout_ms);
}

static bool trusdx_ready(void)
{
    return trusdx_serial_is_ready();
}

static esp_err_t trusdx_on_audio_start(void)
{
    if (!trusdx_ready()) return ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "truSDX CAT/audio backend initialized (startup sequence: ID/PS/FA/IF/MD/RX/UA2)");
    return ESP_OK;
}

static esp_err_t trusdx_sync_frequency_mode(int freq_hz)
{
    // RX; suppresses the UA1 audio stream on R2.00x firmware. While the streaming
    // backend is active (post-connect band/status sync), skip RX; so we do not
    // knock the stream offline; only send it when not streaming (legacy/idle path).
    if (!trusdx_serial_streaming_mode_active()) {
        esp_err_t rx_err = trusdx_send_cmd("RX;", 500);
        if (rx_err != ESP_OK) return rx_err;
    }

    esp_err_t err = trusdx_send_cmd("MD2;", 500);
    if (err != ESP_OK) return err;

    char fa[32];
    std::snprintf(fa, sizeof(fa), "FA%011d;", freq_hz);
    err = trusdx_send_cmd(fa, 500);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "truSDX sync ok rx->md2->fa freq=%d", freq_hz);
    }
    return err;
}

static esp_err_t trusdx_begin_tx(int freq_hz, int tx_base_hz)
{
    (void)freq_hz;
    (void)tx_base_hz;
    if (trusdx_serial_streaming_mode_active()) {
        ESP_LOGI(TAG, "streaming backend owns TX begin; FT8 task will send direct TX0;");
        return ESP_OK;
    }
    // On truSDX/uSDX, TX0; is a TX-on command. RX; is the TX-off command.
    return trusdx_send_cmd("TX0;", 500);
}

static esp_err_t trusdx_set_tone_hz(float tone_hz)
{
    (void)tone_hz;
    return ESP_OK;
}

static esp_err_t trusdx_end_tx(void)
{
    if (trusdx_serial_ft8_tx_active()) {
        trusdx_serial_cancel_ft8_tx();
        return ESP_OK;
    }
    if (trusdx_serial_streaming_mode_active()) {
        ESP_LOGI(TAG, "streaming backend owns TX end; skip direct RX;");
        return ESP_OK;
    }
    return trusdx_send_cmd("RX;", 500);
}

static esp_err_t trusdx_set_tune(bool enable, int freq_hz, int tone_hz)
{
    (void)freq_hz;
    (void)tone_hz;
    return trusdx_serial_set_tune(enable);
}

static const radio_control_ops_t k_ops = {
    .name = "trusdx_cat",
    .ready = trusdx_ready,
    .on_audio_start = trusdx_on_audio_start,
    .sync_frequency_mode = trusdx_sync_frequency_mode,
    .begin_tx = trusdx_begin_tx,
    .set_tone_hz = trusdx_set_tone_hz,
    .end_tx = trusdx_end_tx,
    .set_tune = trusdx_set_tune,
    .set_time = nullptr,
};

const radio_control_ops_t* radio_control_trusdx_get_ops(void)
{
    return &k_ops;
}

esp_err_t radio_trusdx_begin_ft8_tx(const uint8_t tones[79],
                                    int base_hz,
                                    int64_t slot_start_ms,
                                    int64_t now_ms)
{
    return trusdx_serial_begin_ft8_tx(tones, base_hz, slot_start_ms, now_ms);
}

void radio_trusdx_cancel_ft8_tx(void)
{
    trusdx_serial_cancel_ft8_tx();
}

bool radio_trusdx_ft8_tx_active(void)
{
    return trusdx_serial_ft8_tx_active();
}

bool radio_trusdx_ft8_tx_done(void)
{
    return trusdx_serial_ft8_tx_done();
}

bool radio_trusdx_ft8_tx_failed(void)
{
    return trusdx_serial_ft8_tx_failed();
}

void radio_trusdx_clear_ft8_tx_result(void)
{
    trusdx_serial_clear_ft8_tx_result();
}
