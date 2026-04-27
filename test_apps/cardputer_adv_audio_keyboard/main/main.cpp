#include <string.h>

#include "M5Cardputer.h"
#include "board_audio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ADV_AUDIO_TEST";

static constexpr int SAMPLE_RATE = 48000;
static constexpr int CHANNELS = 1;
static constexpr int BITS_PER_SAMPLE = 16;
static constexpr int FRAME_SAMPLES = 480;
static constexpr int RECORD_SECONDS = 3;
static constexpr int TOTAL_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;

static int16_t* s_audio = nullptr;

static void draw_status(const char* line1, const char* line2 = "")
{
    auto& d = M5Cardputer.Display;
    d.fillScreen(TFT_BLACK);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.setTextSize(1);
    d.setCursor(4, 6);
    d.print(line1 ? line1 : "");
    d.setCursor(4, 22);
    d.print(line2 ? line2 : "");
}

static bool record_audio(void)
{
    draw_status("Recording 3 sec...", "Speak near mic");
    size_t offset = 0;
    while (offset < TOTAL_SAMPLES) {
        size_t n = TOTAL_SAMPLES - offset;
        if (n > FRAME_SAMPLES) n = FRAME_SAMPLES;

        size_t bytes_read = 0;
        esp_err_t err = board_audio_read(&s_audio[offset], n, &bytes_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "board_audio_read failed: %s", esp_err_to_name(err));
            draw_status("READ FAILED", esp_err_to_name(err));
            return false;
        }
        offset += bytes_read / sizeof(int16_t);
    }
    return true;
}

static bool playback_audio(void)
{
    draw_status("Playing back...", "");
    size_t offset = 0;
    while (offset < TOTAL_SAMPLES) {
        size_t n = TOTAL_SAMPLES - offset;
        if (n > FRAME_SAMPLES) n = FRAME_SAMPLES;

        size_t bytes_written = 0;
        esp_err_t err = board_audio_write(&s_audio[offset], n, &bytes_written);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "board_audio_write failed: %s", esp_err_to_name(err));
            draw_status("WRITE FAILED", esp_err_to_name(err));
            return false;
        }
        offset += bytes_written / sizeof(int16_t);
    }
    return true;
}

extern "C" void app_main(void)
{
    M5Cardputer.beginDisplayOnly(true);
    draw_status("Press r/R to test", "record -> delay -> play");

    s_audio = (int16_t*)heap_caps_malloc(TOTAL_SAMPLES * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!s_audio) {
        ESP_LOGE(TAG, "audio buffer allocation failed");
        draw_status("ALLOC FAILED", "");
        return;
    }

    board_audio_config_t cfg = {};
    cfg.sample_rate = SAMPLE_RATE;
    cfg.channels = CHANNELS;
    cfg.bits_per_sample = BITS_PER_SAMPLE;
    cfg.mic_gain_db = 30.0f;
    cfg.speaker_volume = 80;

    esp_err_t err = board_audio_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "board_audio_init failed: %s", esp_err_to_name(err));
        draw_status("AUDIO INIT FAILED", esp_err_to_name(err));
        return;
    }

    while (true) {
        M5Cardputer.update();
        const auto& keys = M5Cardputer.Keyboard.keysState();
        bool run = M5Cardputer.Keyboard.isKeyPressed('r') ||
                   M5Cardputer.Keyboard.isKeyPressed('R');
        for (char c : keys.word) {
            if (c == 'r' || c == 'R') {
                run = true;
                break;
            }
        }

        if (run) {
            memset(s_audio, 0, TOTAL_SAMPLES * sizeof(int16_t));
            if (record_audio()) {
                draw_status("Recorded", "playback in 1 sec");
                vTaskDelay(pdMS_TO_TICKS(1000));
                (void)playback_audio();
                draw_status("Done", "Press r/R to repeat");
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
