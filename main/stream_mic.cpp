#include "stream_mic.h"

#include <cmath>
#include <cstring>

#include "board_audio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "ft8_audio_pipeline.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "resample.h"

static const char* TAG = "KH1_MIC";

static constexpr int MIC_SAMPLE_RATE = 48000;
static constexpr int MIC_BITS_PER_SAMPLE = 16;
static constexpr int MIC_CHANNELS = 1;
static constexpr float MIC_GAIN_DB = 30.0f;
static constexpr int MIC_FRAME_SAMPLES = 480;
static constexpr int MIC_DECIM_FACTOR = 8;

static TaskHandle_t s_task_handle = nullptr;
static volatile bool s_stop_requested = false;
static bool s_started = false;
static bool s_streaming = false;
static char s_status_string[64] = "Idle";
static char s_debug_line1[64] = "";
static char s_debug_line2[64] = "";

extern bool g_streaming;

typedef struct {
    int16_t* pcm;
    resample_state_t resample;
    uint32_t frame;
} mic_reader_ctx_t;

static bool mic_should_stop(void* ctx)
{
    (void)ctx;
    return s_stop_requested;
}

static int mic_read_ft8_samples(void* ctx, float* out, int max_samples)
{
    mic_reader_ctx_t* mic = (mic_reader_ctx_t*)ctx;
    if (!mic || !mic->pcm || !out || max_samples <= 0) return 0;

    const int input_samples = MIC_FRAME_SAMPLES;
    const int output_samples = input_samples / MIC_DECIM_FACTOR;
    if (max_samples < output_samples) return 0;

    size_t bytes_read = 0;
    esp_err_t err = board_audio_read(mic->pcm, input_samples, &bytes_read);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "board_audio_read failed: %s", esp_err_to_name(err));
        return 0;
    }

    int frames = (int)(bytes_read / sizeof(int16_t));
    if (frames <= 0) return 0;
    int out_count = frames / MIC_DECIM_FACTOR;
    if (out_count > max_samples) out_count = max_samples;

    int16_t minv = mic->pcm[0];
    int16_t maxv = mic->pcm[0];
    int64_t sum_abs = 0;
    for (int i = 0; i < frames; ++i) {
        int16_t v = mic->pcm[i];
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
        sum_abs += std::abs((int)v);
    }

    float mono[MIC_FRAME_SAMPLES];
    for (int i = 0; i < frames; ++i) {
        mono[i] = (float)mic->pcm[i] / 32768.0f;
    }
    out_count = resample_48k_to_6k(&mic->resample, mono, out, frames);
    if (out_count > max_samples) out_count = max_samples;

    if ((mic->frame % 25) == 0) {
        int32_t avg_abs = frames > 0 ? (int32_t)(sum_abs / frames) : 0;
        ESP_LOGI(TAG, "frame=%lu bytes=%u min=%d max=%d avg_abs=%ld",
                 (unsigned long)mic->frame, (unsigned)bytes_read,
                 (int)minv, (int)maxv, (long)avg_abs);
    }

    snprintf(s_debug_line1, sizeof(s_debug_line1),
             "KH1-MIC %d/%d/%d", MIC_SAMPLE_RATE, MIC_BITS_PER_SAMPLE, MIC_CHANNELS);
    snprintf(s_debug_line2, sizeof(s_debug_line2),
             "rd=%u min=%d max=%d", (unsigned)bytes_read, (int)minv, (int)maxv);

    mic->frame++;
    return out_count;
}

static void mic_stream_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG, "KH1-MIC audio streaming task started");

    board_audio_config_t cfg = {};
    cfg.sample_rate = MIC_SAMPLE_RATE;
    cfg.channels = MIC_CHANNELS;
    cfg.bits_per_sample = MIC_BITS_PER_SAMPLE;
    cfg.mic_gain_db = MIC_GAIN_DB;
    cfg.speaker_volume = 80;

    esp_err_t err = board_audio_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "board_audio_init failed: %s", esp_err_to_name(err));
        snprintf(s_status_string, sizeof(s_status_string), "Mic init failed");
        s_streaming = false;
        s_started = false;
        g_streaming = false;
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    int16_t* pcm = (int16_t*)heap_caps_malloc(sizeof(int16_t) * MIC_FRAME_SAMPLES,
                                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!pcm) {
        ESP_LOGE(TAG, "mic PCM buffer allocation failed");
        board_audio_deinit();
        snprintf(s_status_string, sizeof(s_status_string), "Mic alloc failed");
        s_streaming = false;
        s_started = false;
        g_streaming = false;
        s_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    s_streaming = true;
    g_streaming = true;
    snprintf(s_status_string, sizeof(s_status_string), "Streaming KH1-MIC 48k/16/1");

    mic_reader_ctx_t reader = {
        .pcm = pcm,
        .resample = {},
        .frame = 0,
    };
    resample_init(&reader.resample);
    ft8_audio_pipeline_config_t pipe_cfg = {
        .tag = TAG,
        .ctx = &reader,
        .read = mic_read_ft8_samples,
        .should_stop = mic_should_stop,
        .on_block_processed = nullptr,
    };
    ft8_audio_pipeline_run(&pipe_cfg);

    free(pcm);
    board_audio_deinit();
    s_streaming = false;
    s_started = false;
    g_streaming = false;
    s_task_handle = nullptr;
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "KH1-MIC audio streaming task stopped");
    vTaskDelete(nullptr);
}

bool mic_stream_start(void)
{
    if (s_started || s_task_handle) {
        ESP_LOGW(TAG, "KH1-MIC stream already started");
        return true;
    }

    s_stop_requested = false;
    ft8_audio_pipeline_clear_latest_waterfall_row();
    snprintf(s_status_string, sizeof(s_status_string), "Starting KH1-MIC");
    BaseType_t ret = xTaskCreatePinnedToCore(mic_stream_task, "stream_mic",
                                             8192, nullptr, 4, &s_task_handle, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create KH1-MIC stream task");
        s_task_handle = nullptr;
        snprintf(s_status_string, sizeof(s_status_string), "Mic task failed");
        return false;
    }

    s_started = true;
    return true;
}

void mic_stream_stop(void)
{
    if (!s_started && !s_task_handle) return;
    ESP_LOGI(TAG, "Stopping KH1-MIC audio");
    s_stop_requested = true;
    g_streaming = false;

    int timeout = 50;
    while (s_task_handle && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }

    if (s_task_handle) {
        ESP_LOGW(TAG, "KH1-MIC stream task did not stop before timeout");
        snprintf(s_status_string, sizeof(s_status_string), "Mic stopping");
    } else {
        s_started = false;
        s_streaming = false;
        snprintf(s_status_string, sizeof(s_status_string), "Idle");
    }
}

bool mic_stream_is_streaming(void)
{
    return s_streaming;
}

const char* mic_stream_get_status_string(void)
{
    return s_status_string;
}

const char* mic_stream_get_debug_line1(void)
{
    return s_debug_line1;
}

const char* mic_stream_get_debug_line2(void)
{
    return s_debug_line2;
}
