#include "audio_trusdx_serial.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "ch340_usb_serial.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ft8_audio_pipeline.h"

extern bool g_streaming;
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

static const char* TAG = "TRUSDX";
static const char* TAG_RX = "TRUSDX_RX";
static const char* TAG_TX = "TRUSDX_TX";
static const char* TAG_TUNE = "TRUSDX_TUNE";

static constexpr int TRUSDX_RX_OUTPUT_RATE = FT8_SAMPLE_RATE;
static constexpr int TRUSDX_CAT_TIMEOUT_MS = 1000;
// The CH340 init sequence ends by asserting DTR, which is wired to the truSDX
// ATmega RESET — so enumerating the link REBOOTS the rig. wait_until_ready()
// only confirms USB enumeration, not that the rig has finished booting. Without
// a settle delay the connect races ahead and sends CAT / UA1; into a rig that is
// still rebooting, which is the "first connect dead (~6 B/s), works on retry" bug.
// The PC reference script waits 4 s after open for exactly this reason.
static constexpr int TRUSDX_RIG_REBOOT_MS = 4000;
// First CAT query (ID;) can still land before the rig's parser is alive; retry a
// few times so a slightly-slow boot self-heals instead of needing a manual reconnect.
static constexpr int TRUSDX_ID_RETRIES = 5;
// ESP-IDF's USB host does not reliably enumerate a device that is already attached
// when the host is installed (it expects hot-plug AFTER install; see esp-idf issues
// #12412 / #17918 — the timing Kconfig knobs do NOT fix it). Espressif's recommended
// remedy is app-level retry: fully uninstall + reinstall the host to re-run
// enumeration. So if the link doesn't come ready, tear the host all the way down
// and bring it back up, a few times, before giving up. This makes boot-with-cable
// recover by itself instead of needing a Cardputer power-cycle.
static constexpr int TRUSDX_ENUM_ATTEMPTS = 4;
static constexpr int TRUSDX_ENUM_ATTEMPT_MS = 2500;  // per-attempt ready wait
// The truSDX sometimes accepts UA1; (every CAT query OK) but never starts the RX audio
// stream: totalRxBytes stays at the CAT byte count, so the audio-fed waterfall is blank
// while the UI reads "Sync to truSDX" (a dead pipe). Detect that (bytes still low after
// the reader task starts) and re-issue MD2;/UA1; until audio actually flows.
static constexpr uint32_t TRUSDX_FLOWING_MIN = 200;       // > this many bytes => RX audio flowing
static constexpr int      TRUSDX_UA1_RETRIES = 3;         // re-send MD2;/UA1; up to N times
static constexpr int      TRUSDX_UA1_RETRY_WAIT_MS = 900; // settle window after each re-send
static constexpr int TRUSDX_TX_CHUNK_BYTES = 64;
static constexpr int TRUSDX_TX_LEAD_SAMPLES = 128;
static constexpr int TRUSDX_TX_MARKER_TIMEOUT_MS = 5000;
static constexpr float TRUSDX_TX_AMPLITUDE = 88.0f;
static constexpr double TRUSDX_TWO_PI = 6.28318530717958647692;
static constexpr double TRUSDX_FT8_TONE_SECONDS = 0.160;
static constexpr int TRUSDX_FT8_TONE_COUNT = 79;

enum class TrusdxStreamState {
    Audio,
    CatToken,
};

struct TrusdxParser {
    TrusdxStreamState state = TrusdxStreamState::Audio;
    // UA1 streams raw 8-bit audio with no ;CAT; framing, so the ';' state machine
    // must be bypassed. Lives in the parser (not a global) so it is reset by
    // reset_parser() on every connect and is written before the stream task that
    // reads it is created (task creation acts as the cross-core memory barrier).
    bool pure_audio = false;
    char token[48] = {};
    int token_len = 0;
    bool have_sample = false;
    float prev_sample = 0.0f;
    float curr_sample = 0.0f;
    double input_index = 0.0;
    double next_output_pos = 0.0;
    uint32_t frame = 0;
    uint32_t raw_since_log = 0;
    uint32_t audio_since_log = 0;
    uint32_t out_since_log = 0;
    int64_t last_log_ms = 0;
};

static Ch340UsbSerial s_ch340;
static SemaphoreHandle_t s_transport_mutex = nullptr;
static SemaphoreHandle_t s_write_mutex = nullptr;
static TaskHandle_t s_stream_task_handle = nullptr;
static TaskHandle_t s_tx_task_handle = nullptr;

static volatile bool s_started = false;
static volatile bool s_streaming = false;
static volatile bool s_streaming_cat_mode = false;
static volatile bool s_stop_requested = false;
static volatile bool s_connected = false;
static volatile bool s_tune_on = false;
static volatile bool s_tune_waiting_for_marker = false;
static volatile bool s_tx_active = false;
static volatile bool s_tx_waiting_for_marker = false;
static volatile bool s_tx_audio_streaming = false;
static volatile bool s_tx_marker_seen = false;
static volatile bool s_tx_done = false;
static volatile bool s_tx_failed = false;
static volatile bool s_tx_cancel_requested = false;

static char s_status_string[64] = "Idle";
static char s_debug_line1[64] = "";
static char s_debug_line2[64] = "";

static TrusdxParser s_parser;

static uint8_t s_tx_tones[TRUSDX_FT8_TONE_COUNT] = {};
static int s_tx_base_hz = 1500;
static int64_t s_tx_slot_start_ms = 0;
static int64_t s_tx_start_ms = 0;

static bool ensure_mutex(void)
{
    if (!s_transport_mutex) {
        s_transport_mutex = xSemaphoreCreateMutex();
    }
    if (!s_write_mutex) {
        s_write_mutex = xSemaphoreCreateMutex();
    }
    return s_transport_mutex != nullptr && s_write_mutex != nullptr;
}

static bool lock_transport(uint32_t timeout_ms)
{
    if (!ensure_mutex()) return false;
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_transport_mutex, ticks) == pdTRUE;
}

static void unlock_transport(void)
{
    if (s_transport_mutex) {
        xSemaphoreGive(s_transport_mutex);
    }
}

static bool lock_write(uint32_t timeout_ms)
{
    if (!ensure_mutex()) return false;
    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(s_write_mutex, ticks) == pdTRUE;
}

static void unlock_write(void)
{
    if (s_write_mutex) {
        xSemaphoreGive(s_write_mutex);
    }
}

static bool timeout_expired(TickType_t start, uint32_t timeout_ms)
{
    return (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms);
}

static void reset_parser(void)
{
    std::memset(&s_parser, 0, sizeof(s_parser));
    s_parser.state = TrusdxStreamState::Audio;
    s_parser.last_log_ms = rtc_now_ms();
}

static void set_status(const char* text)
{
    std::snprintf(s_status_string, sizeof(s_status_string), "%s", text ? text : "");
}

static bool transport_is_ready_locked(void)
{
    s_ch340.poll();
    return s_ch340.isReady();
}

static esp_err_t transport_write_blocking(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!data && len > 0) return ESP_ERR_INVALID_ARG;
    size_t sent = 0;
    TickType_t start = xTaskGetTickCount();

    while (sent < len && !timeout_expired(start, timeout_ms)) {
        if (!lock_transport(100)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        s_ch340.poll();
        bool ready = s_ch340.isReady();
        int rc = ready ? s_ch340.writeBytes(data + sent, len - sent) : -1;
        unlock_transport();

        if (rc > 0) {
            sent += (size_t)rc;
            continue;
        }
        if (rc == -2) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        return ready ? ESP_FAIL : ESP_ERR_INVALID_STATE;
    }

    return (sent == len) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static size_t format_cat_for_current_mode(const char* cmd, char* out, size_t out_sz)
{
    if (!cmd || !out || out_sz == 0) return 0;

    const size_t in_len = std::strlen(cmd);
    if (!s_streaming_cat_mode || cmd[0] == ';') {
        size_t n = (in_len < out_sz - 1) ? in_len : out_sz - 1;
        std::memcpy(out, cmd, n);
        out[n] = '\0';
        return n;
    }

    size_t pos = 0;
    out[pos++] = ';';
    for (size_t i = 0; i < in_len && pos + 1 < out_sz; ++i) {
        out[pos++] = cmd[i];
    }
    if ((pos == 0 || out[pos - 1] != ';') && pos + 1 < out_sz) {
        out[pos++] = ';';
    }
    out[pos] = '\0';
    return pos;
}

static esp_err_t send_cat_locked(const char* cmd, uint32_t timeout_ms)
{
    char framed[96];
    size_t len = format_cat_for_current_mode(cmd, framed, sizeof(framed));
    if (len == 0) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "send %s", framed);
    return transport_write_blocking(reinterpret_cast<const uint8_t*>(framed), len, timeout_ms);
}

static bool wait_for_tx_marker(const char* tag, volatile bool* waiting_flag, volatile bool* cancel_flag)
{
    // Legacy helper kept for diagnostics. Mini-FT8 now drives truSDX directly:
    // setup/sync via ID/PS/FA/IF/MD/RX/UA2, tune via TX2;/RX;, and TX via TX0;/RX;.
    if (waiting_flag) {
        *waiting_flag = true;
    }

    TickType_t start = xTaskGetTickCount();
    while (!timeout_expired(start, TRUSDX_TX_MARKER_TIMEOUT_MS)) {
        if (s_tx_marker_seen) {
            s_tx_marker_seen = false;
            if (waiting_flag) {
                *waiting_flag = false;
            }
            return true;
        }
        if (s_stop_requested || !s_connected || (cancel_flag && *cancel_flag)) {
            if (waiting_flag) {
                *waiting_flag = false;
            }
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (waiting_flag) {
        *waiting_flag = false;
    }
    ESP_LOGW(tag ? tag : TAG, "TX; marker timeout");
    return false;
}

esp_err_t trusdx_serial_send_cat(const char* cmd, uint32_t timeout_ms)
{
    if (!cmd) return ESP_ERR_INVALID_ARG;
    if (!trusdx_serial_is_ready()) return ESP_ERR_INVALID_STATE;

    if (s_tx_active && xTaskGetCurrentTaskHandle() != s_tx_task_handle) {
        ESP_LOGI(TAG, "skip background CAT during TX stream");
        return ESP_ERR_INVALID_STATE;
    }

    if (!lock_write(timeout_ms)) return ESP_ERR_TIMEOUT;
    esp_err_t err = send_cat_locked(cmd, timeout_ms);
    unlock_write();
    return err;
}

static bool transport_read_one(uint8_t* out)
{
    if (!out) return false;
    if (!lock_transport(20)) return false;

    s_ch340.poll();
    if (!s_ch340.isConnected()) {
        s_connected = false;
        s_streaming_cat_mode = false;
        s_stop_requested = true;
        unlock_transport();
        ESP_LOGW(TAG, "CH340 disconnected");
        return false;
    }

    int n = s_ch340.readBytes(out, 1);
    unlock_transport();
    return n == 1;
}

static esp_err_t send_cat_wait_response(const char* cmd,
                                        char* out,
                                        size_t out_sz,
                                        uint32_t timeout_ms)
{
    if (!out || out_sz == 0) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';

    esp_err_t err = trusdx_serial_send_cat(cmd, timeout_ms);
    if (err != ESP_OK) return err;

    size_t len = 0;
    TickType_t start = xTaskGetTickCount();
    while (!timeout_expired(start, timeout_ms)) {
        uint8_t b = 0;
        if (!transport_read_one(&b)) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        if (len + 1 < out_sz) {
            out[len++] = (char)b;
            out[len] = '\0';
        }
        if (b == ';') {
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

static float u8_to_float(uint8_t raw)
{
    return ((float)((int)raw - 128)) / 128.0f;
}

static bool resampler_push(TrusdxParser* p, float sample, float* out, int* out_count, int max_samples)
{
    if (!p || !out || !out_count || *out_count >= max_samples) return false;

    const double current_index = p->input_index;
    bool produced = false;

    if (!p->have_sample) {
        p->prev_sample = sample;
        p->curr_sample = sample;
        p->have_sample = true;
    } else {
        p->prev_sample = p->curr_sample;
        p->curr_sample = sample;
    }

    const double prev_index = p->have_sample && current_index >= 1.0 ? current_index - 1.0 : current_index;
    const double step = (double)TRUSDX_RX_SAMPLE_RATE / (double)TRUSDX_RX_OUTPUT_RATE;

    while (p->next_output_pos <= current_index && *out_count < max_samples) {
        double frac = current_index > prev_index ? (p->next_output_pos - prev_index) : 0.0;
        if (frac < 0.0) frac = 0.0;
        if (frac > 1.0) frac = 1.0;

        float y = p->prev_sample + (p->curr_sample - p->prev_sample) * (float)frac;
        out[(*out_count)++] = y;
        p->next_output_pos += step;
        produced = true;
    }

    p->input_index = current_index + 1.0;
    return produced;
}

static void process_token(const char* token)
{
    if (!token || token[0] == '\0') return;

    if (std::strncmp(token, "US", 2) == 0) {
        ESP_LOGI(TAG_RX, "token US");
        return;
    }
    if (std::strcmp(token, "TX") == 0) {
        ESP_LOGI(TAG_RX, "token TX;");
        s_tx_marker_seen = true;
        if (s_tx_waiting_for_marker || s_tune_waiting_for_marker) {
            ESP_LOGI(TAG_RX, "TX; marker seen");
        }
        return;
    }
    if (std::strncmp(token, "FA", 2) == 0 ||
        std::strncmp(token, "IF", 2) == 0 ||
        std::strncmp(token, "ID", 2) == 0 ||
        std::strncmp(token, "MD", 2) == 0) {
        ESP_LOGI(TAG_RX, "token %.32s;", token);
        return;
    }

    ESP_LOGD(TAG_RX, "token %.32s;", token);
}

static bool parser_feed_byte(TrusdxParser* p, uint8_t b, float* out, int* out_count, int max_samples)
{
    if (!p) return false;

    if (p->pure_audio) {
        // UA1 streams raw 8-bit audio with no ;CAT; framing, so a byte equal to
        // 59 (';') is just an audio sample, not a delimiter. Treat all bytes as audio.
        p->audio_since_log++;
        return resampler_push(p, u8_to_float(b), out, out_count, max_samples);
    }

    if (p->state == TrusdxStreamState::Audio) {
        if (b == ';') {
            p->state = TrusdxStreamState::CatToken;
            p->token_len = 0;
            p->token[0] = '\0';
            return false;
        }

        p->audio_since_log++;
        return resampler_push(p, u8_to_float(b), out, out_count, max_samples);
    }

    if (b == ';') {
        p->token[p->token_len] = '\0';
        process_token(p->token);
        p->token_len = 0;
        p->state = TrusdxStreamState::Audio;
        return false;
    }

    if (p->token_len + 1 < (int)sizeof(p->token)) {
        p->token[p->token_len++] = (char)b;
        p->token[p->token_len] = '\0';
    } else {
        p->token[p->token_len] = '\0';
        ESP_LOGW(TAG_RX, "token overflow %.32s", p->token);
        p->token_len = 0;
        p->state = TrusdxStreamState::Audio;
        return false;
    }

    if (p->token_len == 2 && p->token[0] == 'U' && p->token[1] == 'S') {
        process_token("US");
        p->token_len = 0;
        p->state = TrusdxStreamState::Audio;
    }

    return false;
}

static bool trusdx_should_stop(void* ctx)
{
    (void)ctx;
    return s_stop_requested || !s_connected;
}

static int trusdx_read_ft8_samples(void* ctx, float* out, int max_samples)
{
    TrusdxParser* parser = (TrusdxParser*)ctx;
    if (!parser || !out || max_samples <= 0) return 0;
    if (s_tx_audio_streaming || s_tune_on) {
        vTaskDelay(pdMS_TO_TICKS(5));
        return 0;
    }

    int out_count = 0;
    const int64_t start_ms = rtc_now_ms();

    while (out_count < max_samples && !s_stop_requested && s_connected && !s_tx_audio_streaming && !s_tune_on) {
        uint8_t b = 0;
        if (!transport_read_one(&b)) {
            if (out_count > 0 || (rtc_now_ms() - start_ms) > 200) break;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        parser->raw_since_log++;
        int before = out_count;
        parser_feed_byte(parser, b, out, &out_count, max_samples);
        if (out_count > before) {
            parser->out_since_log += (uint32_t)(out_count - before);
        }
    }

    const int64_t now_ms = rtc_now_ms();
    if (now_ms - parser->last_log_ms >= 1000) {
        ESP_LOGI(TAG_RX, "raw bytes received=%lu audio samples=%lu resampled frames=%lu",
                 (unsigned long)parser->raw_since_log,
                 (unsigned long)parser->audio_since_log,
                 (unsigned long)parser->out_since_log);
        // Line 1: CH340 driver diagnostics so a stuck connect is visible on-screen.
        // st=DriverState (5=Ready), bi=last bulk-IN status (0=COMPLETED),
        // tot=cumulative bytes the USB driver has received since begin().
        std::snprintf(s_debug_line1, sizeof(s_debug_line1),
                      "st%d bi%d tot%lu",
                      s_ch340.driverState(),
                      s_ch340.bulkInStatus(),
                      (unsigned long)s_ch340.totalRxBytes());
        std::snprintf(s_debug_line2, sizeof(s_debug_line2),
                      "r%lu a%lu o%lu",
                      (unsigned long)parser->raw_since_log,
                      (unsigned long)parser->audio_since_log,
                      (unsigned long)parser->out_since_log);
        parser->raw_since_log = 0;
        parser->audio_since_log = 0;
        parser->out_since_log = 0;
        parser->last_log_ms = now_ms;
    }

    return out_count;
}

static void stream_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG_RX, "stream parser start");
    s_streaming = true;
    g_streaming = true;
    set_status("Streaming truSDX");

    ft8_audio_pipeline_config_t cfg = {
        .tag = TAG_RX,
        .ctx = &s_parser,
        .read = trusdx_read_ft8_samples,
        .should_stop = trusdx_should_stop,
        .on_block_processed = nullptr,
    };

    ESP_LOGI(TAG_RX, "audio pipeline start ok");
    ft8_audio_pipeline_run(&cfg);

    s_streaming = false;
    g_streaming = false;
    s_stream_task_handle = nullptr;
    if (!s_stop_requested) {
        s_connected = false;
        s_started = false;
        s_streaming_cat_mode = false;
        set_status("truSDX disconnected");
        ESP_LOGW(TAG, "CH340 stream stopped");
        if (lock_transport(1000)) {
            s_ch340.end();
            unlock_transport();
        }
    }
    ESP_LOGI(TAG_RX, "stream parser stopped");
    vTaskDelete(nullptr);
}

static esp_err_t wait_until_ready(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while (!timeout_expired(start, timeout_ms)) {
        if (!lock_transport(100)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        bool ready = transport_is_ready_locked();
        unlock_transport();
        if (ready) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

bool trusdx_serial_start(void)
{
    if (s_started || s_stream_task_handle) {
        ESP_LOGW(TAG, "truSDX serial already started");
        return true;
    }
    if (!ensure_mutex()) {
        set_status("truSDX mutex failed");
        return false;
    }

    s_stop_requested = false;
    s_connected = false;
    s_streaming_cat_mode = false;
    s_tune_on = false;
    s_tune_waiting_for_marker = false;
    s_tx_active = false;
    s_tx_waiting_for_marker = false;
    s_tx_audio_streaming = false;
    s_tx_marker_seen = false;
    s_tx_done = false;
    s_tx_failed = false;
    s_tx_cancel_requested = false;
    reset_parser();  // clears s_parser.pure_audio
    ft8_audio_pipeline_clear_latest_waterfall_row();

    ESP_LOGI(TAG, "connect start");
    set_status("Connecting truSDX");

    // Enumeration with retry. A device already attached when the host installs may
    // not enumerate on the first try (ESP-IDF limitation). Each attempt fully
    // reinstalls the host (begin) and waits a short window for the link to come
    // ready; on timeout we tear the host all the way down (end) and try again,
    // which re-runs enumeration from scratch. This recovers boot-with-cable.
    esp_err_t err = ESP_ERR_TIMEOUT;
    bool ready = false;
    for (int attempt = 0; attempt < TRUSDX_ENUM_ATTEMPTS && !s_stop_requested; ++attempt) {
        ESP_LOGI(TAG, "ch340 begin (enum attempt %d/%d)", attempt + 1, TRUSDX_ENUM_ATTEMPTS);
        if (attempt > 0) set_status("Retrying truSDX");
        err = s_ch340.begin(TRUSDX_CAT_BAUD);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ch340 begin err %s", esp_err_to_name(err));
            s_ch340.end();
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }
        // KEY MEASUREMENT: poll the stack a moment, then ask what devices it knows
        // about (independent of NEW_DEV) and whether NEW_DEV actually fired.
        vTaskDelay(pdMS_TO_TICKS(500));
        if (wait_until_ready(TRUSDX_ENUM_ATTEMPT_MS) == ESP_OK) {
            ready = true;
            break;
        }
        ESP_LOGW(TAG, "enum attempt %d timed out; reinstalling host", attempt + 1);
        s_ch340.end();           // full uninstall -> forces fresh enumeration next begin()
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (!ready) {
        ESP_LOGE(TAG, "connect failed: ch340 not ready after %d attempts", TRUSDX_ENUM_ATTEMPTS);
        set_status("truSDX not found");
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }

    s_connected = true;
    ESP_LOGI(TAG, "ch340 ready");

    // The CH340 DTR assertion above just reset the rig's ATmega. Give it time to
    // reboot before any CAT, then send a lone ';' to resync the rig's CAT parser
    // and drop the boot-time bytes. Mirrors the PC capture script (4 s + ';').
    ESP_LOGI(TAG, "rig reboot settle %d ms", TRUSDX_RIG_REBOOT_MS);
    set_status("Waking truSDX");
    vTaskDelay(pdMS_TO_TICKS(TRUSDX_RIG_REBOOT_MS));
    if (s_stop_requested) { s_connected = false; s_ch340.end(); return false; }
    trusdx_serial_send_cat(";", TRUSDX_CAT_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(250));
    { uint8_t drain; while (transport_read_one(&drain)) { /* flush boot/resync bytes */ } }
    set_status("Connecting truSDX");

    char rsp[96] = {};
    // ID; is the first real query — retry it so a slightly-slow boot self-heals
    // instead of dropping the whole connect (the old behavior needed a manual reconnect).
    err = ESP_ERR_TIMEOUT;
    for (int attempt = 0; attempt < TRUSDX_ID_RETRIES && !s_stop_requested; ++attempt) {
        ESP_LOGI(TAG, "send ID; (attempt %d/%d)", attempt + 1, TRUSDX_ID_RETRIES);
        err = send_cat_wait_response("ID;", rsp, sizeof(rsp), TRUSDX_CAT_TIMEOUT_MS);
        if (err == ESP_OK && rsp[0] != '\0') break;
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: ID %s", esp_err_to_name(err));
        set_status("truSDX ID failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }
    ESP_LOGI(TAG, "rx ID %s", rsp);

    ESP_LOGI(TAG, "send PS;");
    err = send_cat_wait_response("PS;", rsp, sizeof(rsp), TRUSDX_CAT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: PS %s", esp_err_to_name(err));
        set_status("truSDX PS failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }
    ESP_LOGI(TAG, "rx PS %s", rsp);

    ESP_LOGI(TAG, "send FA;");
    err = send_cat_wait_response("FA;", rsp, sizeof(rsp), TRUSDX_CAT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: FA %s", esp_err_to_name(err));
        set_status("truSDX FA failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }
    ESP_LOGI(TAG, "rx FA %s", rsp);

    ESP_LOGI(TAG, "send IF;");
    err = send_cat_wait_response("IF;", rsp, sizeof(rsp), TRUSDX_CAT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: IF %s", esp_err_to_name(err));
        set_status("truSDX IF failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }
    ESP_LOGI(TAG, "rx IF %s", rsp);

    ESP_LOGI(TAG, "send MD2;");
    err = trusdx_serial_send_cat("MD2;", TRUSDX_CAT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: MD2 %s", esp_err_to_name(err));
        set_status("truSDX MD2 failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }

    // NOTE: the PC capture script (which streams reliably) sends MD2; then UA1;
    // and never sends RX;. Sending RX; here appeared to leave the rig in a state
    // where UA1; returns a reply but does not start the audio stream. Omit it.
    ESP_LOGI(TAG, "send UA1;");
    err = trusdx_serial_send_cat("UA1;", TRUSDX_CAT_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "connect failed: UA1 %s", esp_err_to_name(err));
        set_status("truSDX UA1 failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }

    ESP_LOGI(TAG, "streaming mode enabled");
    s_streaming_cat_mode = true;
    reset_parser();
    s_parser.pure_audio = true;  // UA1 stream is raw audio; set AFTER reset, BEFORE the
                                 // stream task is created (task create = memory barrier)

    BaseType_t ret = xTaskCreatePinnedToCore(stream_task, "trusdx_rx",
                                             8192, nullptr, 4,
                                             &s_stream_task_handle, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "connect failed: stream task create");
        set_status("truSDX task failed");
        s_connected = false;
        s_streaming_cat_mode = false;
        s_ch340.end();
        return false;
    }

    s_started = true;
    // Give the stream task a moment to pull bytes, then snapshot whether audio is
    // actually flowing — this distinguishes "connected + streaming" from
    // "connected but dead pipe" (the st5 tot-frozen case) in the persisted log.
    vTaskDelay(pdMS_TO_TICKS(800));

    // Dead-pipe recovery: the rig opened and answered all CAT, but UA1; sometimes
    // doesn't start audio (tot frozen near the CAT byte count -> blank waterfall). The
    // stream reader task is already running, so re-issuing MD2;/UA1; makes totalRxBytes
    // climb the moment the rig starts streaming. Stop as soon as audio flows; if it
    // never does we still return connected (caller/UI unchanged) so the user can retry.
    for (int r = 0; r < TRUSDX_UA1_RETRIES
                    && s_ch340.totalRxBytes() <= TRUSDX_FLOWING_MIN
                    && !s_stop_requested; ++r) {
        ESP_LOGW(TAG, "no RX audio (tot=%lu); re-sending MD2;/UA1; (retry %d/%d)",
                 (unsigned long)s_ch340.totalRxBytes(), r + 1, TRUSDX_UA1_RETRIES);
        set_status("Starting audio");
        trusdx_serial_send_cat("MD2;", TRUSDX_CAT_TIMEOUT_MS);
        trusdx_serial_send_cat("UA1;", TRUSDX_CAT_TIMEOUT_MS);
        vTaskDelay(pdMS_TO_TICKS(TRUSDX_UA1_RETRY_WAIT_MS));
    }
    return true;
}

void trusdx_serial_stop(void)
{
    if (!s_started && !s_stream_task_handle && !s_connected) return;

    ESP_LOGI(TAG, "stop");
    s_stop_requested = true;
    trusdx_serial_cancel_ft8_tx();

    for (int i = 0; i < 50 && (s_stream_task_handle || s_tx_task_handle); ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (lock_transport(1000)) {
        s_ch340.end();
        unlock_transport();
    }

    s_started = false;
    s_streaming = false;
    s_connected = false;
    s_streaming_cat_mode = false;
    s_tune_on = false;
    s_tune_waiting_for_marker = false;
    s_tx_active = false;
    s_tx_waiting_for_marker = false;
    s_tx_audio_streaming = false;
    s_tx_marker_seen = false;
    g_streaming = false;
    set_status("Idle");
}

bool trusdx_serial_is_streaming(void)
{
    return s_streaming;
}

uint32_t trusdx_serial_total_rx_bytes(void)
{
    return s_ch340.totalRxBytes();
}

uint32_t trusdx_serial_dropped_rx_bytes(void)
{
    return s_ch340.droppedRxBytes();
}

bool trusdx_serial_is_ready(void)
{
    if (!s_connected) return false;
    if (!lock_transport(20)) return s_connected;
    bool ready = transport_is_ready_locked();
    unlock_transport();
    return ready;
}

bool trusdx_serial_streaming_mode_active(void)
{
    return s_connected && s_streaming_cat_mode;
}

const char* trusdx_serial_get_status_string(void)
{
    return s_status_string;
}

const char* trusdx_serial_get_debug_line1(void)
{
    return s_debug_line1;
}

const char* trusdx_serial_get_debug_line2(void)
{
    return s_debug_line2;
}

static esp_err_t send_safe_rx_usb_locked(void)
{
    esp_err_t err1 = send_cat_locked("RX;", 300);
    esp_err_t err2 = send_cat_locked("MD2;", 300);
    return (err1 == ESP_OK && err2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t trusdx_serial_set_tune(bool enable)
{
    if (!trusdx_serial_is_ready()) return ESP_ERR_INVALID_STATE;

    if (enable) {
        if (s_tx_active) {
            ESP_LOGW(TAG_TUNE, "reject tune ON while TX active");
            return ESP_ERR_INVALID_STATE;
        }

        ESP_LOGI(TAG_TUNE, "tune ON sequence: TX2;");
        if (!lock_write(1000)) return ESP_ERR_TIMEOUT;

        s_tune_on = true;
        ESP_LOGI(TAG_TUNE, "send ;TX2;");
        esp_err_t err = send_cat_locked("TX2;", 500);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_TUNE, "TX2 failed: %s", esp_err_to_name(err));
            send_safe_rx_usb_locked();
            unlock_write();
            s_tune_on = false;
            return err;
        }

        unlock_write();
        ESP_LOGI(TAG_TUNE, "tune ON (TX2;)");
        set_status("truSDX Tune ON");
        return ESP_OK;
    }

    if (!lock_write(1000)) return ESP_ERR_TIMEOUT;
    ESP_LOGI(TAG_TUNE, "send ;RX;");
    esp_err_t err = send_cat_locked("RX;", 500);
    ESP_LOGI(TAG_TUNE, "send ;MD2;");
    esp_err_t err2 = send_cat_locked("MD2;", 500);
    unlock_write();
    s_tune_on = false;
    set_status("Streaming truSDX");
    ESP_LOGI(TAG_TUNE, "tune OFF");
    ESP_LOGI(TAG_TUNE, "resume RX stream");
    return (err == ESP_OK && err2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static uint8_t clamp_u8(int v)
{
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static inline uint8_t sanitize_trusdx_tx_byte(uint8_t b)
{
    // truSDX shared CAT/audio stream uses ';' as a delimiter, so raw TX audio
    // must not contain 0x3B.
    return (b == 0x3B) ? 0x3A : b;
}

static void tx_task(void* arg)
{
    (void)arg;
    ESP_LOGI(TAG_TX, "TX sequence: TX0; -> ;US -> audio -> RX;");
    if (!lock_write(1000)) {
        ESP_LOGW(TAG_TX, "write lock timeout");
        s_tx_failed = true;
        s_tx_active = false;
        s_tx_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    // truSDX/uSDX uses TX0; as TX-on. RX; is the TX-off/return-to-RX command.
    ESP_LOGI(TAG_TX, "send ;TX0;");
    esp_err_t err = send_cat_locked("TX0;", 500);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_TX, "TX0 failed: %s", esp_err_to_name(err));
        unlock_write();
        s_tx_failed = true;
        s_tx_active = false;
        s_tx_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG_TX, "send ;US");
    err = send_cat_locked(";US", 500);
    if (err != ESP_OK) {
        ESP_LOGW(TAG_TX, ";US failed: %s", esp_err_to_name(err));
        esp_err_t rx_err = send_cat_locked("RX;", 300);
        if (rx_err != ESP_OK) {
            ESP_LOGW(TAG_TX, "recovery ;RX; failed: %s", esp_err_to_name(rx_err));
        }
        unlock_write();
        s_tx_failed = true;
        s_tx_active = false;
        s_tx_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG_TX, "stream start rate=%d", TRUSDX_TX_SAMPLE_RATE);
    set_status("truSDX TX");
    s_tx_audio_streaming = true;

    const double samples_per_tone = TRUSDX_FT8_TONE_SECONDS * (double)TRUSDX_TX_SAMPLE_RATE;
    const int64_t total_samples = (int64_t)std::ceil(samples_per_tone * TRUSDX_FT8_TONE_COUNT);
    int64_t sample_pos = ((s_tx_start_ms - s_tx_slot_start_ms) * TRUSDX_TX_SAMPLE_RATE) / 1000;
    if (sample_pos < 0) sample_pos = 0;
    if (sample_pos > total_samples) sample_pos = total_samples;

    uint8_t chunk[TRUSDX_TX_CHUNK_BYTES];
    double phase = 0.0;
    int64_t sent_samples = 0;
    int64_t logged_bytes = 0;
    int semicolon_count = 0;
    int64_t stream_start_us = esp_timer_get_time();

    while (sample_pos < total_samples && !s_tx_cancel_requested && !s_stop_requested) {
        int n = (int)(total_samples - sample_pos);
        if (n > TRUSDX_TX_CHUNK_BYTES) n = TRUSDX_TX_CHUNK_BYTES;

        for (int i = 0; i < n; ++i) {
            int tone_idx = (int)std::floor((double)sample_pos / samples_per_tone);
            if (tone_idx < 0) tone_idx = 0;
            if (tone_idx >= TRUSDX_FT8_TONE_COUNT) tone_idx = TRUSDX_FT8_TONE_COUNT - 1;
            float tone_hz = (float)s_tx_base_hz + 6.25f * (float)s_tx_tones[tone_idx];
            phase += TRUSDX_TWO_PI * (double)tone_hz / (double)TRUSDX_TX_SAMPLE_RATE;
            if (phase > TRUSDX_TWO_PI) {
                phase = std::fmod(phase, TRUSDX_TWO_PI);
            }
            int v = 128 + (int)std::lrintf(TRUSDX_TX_AMPLITUDE * std::sinf((float)phase));
            uint8_t b = clamp_u8(v);
            if (b == 0x3B) {
                semicolon_count++;
            }
            chunk[i] = sanitize_trusdx_tx_byte(b);
            sample_pos++;
        }

        const int64_t elapsed_samples =
            ((esp_timer_get_time() - stream_start_us) * TRUSDX_TX_SAMPLE_RATE) / 1000000;
        if (sent_samples > elapsed_samples + TRUSDX_TX_LEAD_SAMPLES) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        err = transport_write_blocking(chunk, (size_t)n, 200);
        if (err != ESP_OK) {
            ESP_LOGW(TAG_TX, "stream write failed: %s", esp_err_to_name(err));
            s_tx_failed = true;
            break;
        }

        sent_samples += n;
        if (sent_samples - logged_bytes >= 2048) {
            ESP_LOGI(TAG_TX, "stream bytes=%lld semicolons_sanitized=%d",
                     (long long)sent_samples, semicolon_count);
            logged_bytes = sent_samples;
        }
    }

    if (s_tx_cancel_requested) {
        ESP_LOGI(TAG_TX, "stream cancelled");
        s_tx_failed = true;
    } else if (!s_tx_failed) {
        ESP_LOGI(TAG_TX, "stream complete");
    }

    ESP_LOGI(TAG_TX, "send ;RX;");
    esp_err_t rx_err = send_cat_locked("RX;", 500);
    if (rx_err != ESP_OK) {
        ESP_LOGW(TAG_TX, ";RX; failed: %s", esp_err_to_name(rx_err));
        ESP_LOGW(TAG_TX, "stuck TX recovery needed");
        s_tx_failed = true;
        s_tx_audio_streaming = false;
        unlock_write();
        set_status("truSDX TX recovery needed");
        s_tx_active = false;
        s_tx_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG_TX, ";RX; ok");
    s_tx_audio_streaming = false;
    if (!s_tx_failed) {
        s_tx_done = true;
        ESP_LOGI(TAG_TX, "TX done");
    }
    ESP_LOGI(TAG_TX, "resume RX stream");
    unlock_write();
    set_status("Streaming truSDX");
    s_tx_active = false;
    s_tx_task_handle = nullptr;
    vTaskDelete(nullptr);
}

esp_err_t trusdx_serial_begin_ft8_tx(const uint8_t tones[79],
                                     int base_hz,
                                     int64_t slot_start_ms,
                                     int64_t now_ms)
{
    if (!tones) return ESP_ERR_INVALID_ARG;
    if (!trusdx_serial_is_ready()) return ESP_ERR_INVALID_STATE;
    if (s_tune_on || s_tx_active || s_tx_task_handle) return ESP_ERR_INVALID_STATE;

    std::memcpy(s_tx_tones, tones, TRUSDX_FT8_TONE_COUNT);
    s_tx_base_hz = base_hz;
    s_tx_slot_start_ms = slot_start_ms;
    s_tx_start_ms = now_ms;
    s_tx_done = false;
    s_tx_failed = false;
    s_tx_cancel_requested = false;
    s_tx_waiting_for_marker = false;
    s_tx_audio_streaming = false;
    s_tx_marker_seen = false;
    s_tx_active = true;

    ESP_LOGI(TAG_TX, "tx pending");
    BaseType_t ret = xTaskCreatePinnedToCore(tx_task, "trusdx_tx",
                                             4096, nullptr, 5,
                                             &s_tx_task_handle, 1);
    if (ret != pdPASS) {
        s_tx_active = false;
        s_tx_failed = true;
        s_tx_task_handle = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void trusdx_serial_cancel_ft8_tx(void)
{
    if (s_tx_active || s_tx_task_handle) {
        s_tx_cancel_requested = true;
    }
}

bool trusdx_serial_ft8_tx_active(void)
{
    return s_tx_active || s_tx_task_handle != nullptr;
}

bool trusdx_serial_ft8_tx_done(void)
{
    return s_tx_done;
}

bool trusdx_serial_ft8_tx_failed(void)
{
    return s_tx_failed;
}

void trusdx_serial_clear_ft8_tx_result(void)
{
    s_tx_done = false;
    s_tx_failed = false;
    s_tx_cancel_requested = false;
}
