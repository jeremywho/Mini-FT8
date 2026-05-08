#include "stream_uac.h"
#include "resample.h"
#include "dds_q15.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "usb/usb_host.h"
#include "usb/uac_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/usb_types_ch9.h"

extern "C" {
#include "ft8/decode.h"
#include "ft8/constants.h"
#include "common/monitor.h"
}

#include "ui.h"
#include "core_api_internal.h"
#include <cstring>
#include <cmath>
#include <inttypes.h>

static const char* TAG = "UAC_STREAM";
extern void log_heap(const char* tag);

// External references from main.cpp
extern bool g_streaming;
extern bool g_decode_enabled;
extern int g_time_osr;
extern int g_freq_osr;
extern int64_t g_decode_slot_idx;
extern volatile bool g_decode_in_progress;
extern volatile int64_t g_decode_applied_slot_idx;
extern volatile bool g_cdc_initial_sync_pending;
void decode_monitor_results(monitor_t* mon, const monitor_config_t* cfg, bool update_ui);
int64_t rtc_now_ms();

#ifndef FT8_SAMPLE_RATE
#define FT8_SAMPLE_RATE 6000
#endif

// Task priorities and stack sizes
#define USB_HOST_TASK_PRIORITY  5
#define UAC_TASK_PRIORITY       5
#define UAC_STREAM_TASK_PRIORITY 4
#define TASK_STACK_SIZE         4096
#define STREAM_TASK_STACK_SIZE  8192

// UAC read buffer size (bytes) - must be multiple of 288 (USB transfer size at 48kHz/24bit/stereo)
// 288 bytes = 48 stereo samples per 1ms USB transfer.
// 2304 = 288 * 8 — halved from the original 4608 to free 2304 B of
// DMA-capable BSS for the heap pool. The decoder is offline (FFT runs
// for ~13 s with the sample pump effectively paused), so a smaller
// in-flight USB EP IN buffer only matters during live collection — and
// 8 ms of headroom is still well above what the host scheduler needs.
// Regression canary: waterfall artifacts from the UAC signal generator.
#define UAC_READ_BUFFER_SIZE    2304

// USB host DMAs into this buffer. Placed in DMA-capable BSS via DMA_ATTR so
// task start-up doesn't depend on finding a large DMA-capable heap block —
// previously heap_caps_malloc(MALLOC_CAP_DMA) failed under fragmentation
// even when raw free bytes looked sufficient (alignment eats into runs).
static DMA_ATTR uint8_t s_usb_buffer[UAC_READ_BUFFER_SIZE];

typedef struct {
    uint32_t sample_freq;
    uint8_t bit_resolution;
    uint8_t channels;
} uac_active_format_t;

// Event types for internal queue
typedef enum {
    UAC_EVT_DRIVER,
    UAC_EVT_DEVICE,
    UAC_EVT_STOP,
} uac_event_type_t;

typedef struct {
    uac_event_type_t type;
    union {
        struct {
            uint8_t addr;
            uint8_t iface_num;
            uac_host_driver_event_t event;
        } driver;
        struct {
            uac_host_device_handle_t handle;
            uac_host_device_event_t event;
        } device;
    };
} uac_event_t;

// Global state
static uac_stream_state_t s_state = UAC_STATE_IDLE;
static QueueHandle_t s_event_queue = NULL;
static uac_host_device_handle_t s_mic_handle = NULL;
static cdc_acm_dev_hdl_t s_cdc_handle = NULL;
static TaskHandle_t s_usb_task_handle = NULL;
static TaskHandle_t s_uac_task_handle = NULL;

// Speaker (UAC OUT) — captured when UAC_HOST_DRIVER_EVENT_TX_CONNECTED
// fires during enumeration. Used by uac_tx_test_start() to validate the
// PTX FIFO under the experimental 364/364 split: open the speaker
// endpoint, push a fixed 1.5 kHz tone for the TX duration, close. This
// replaces the QMX TA-based tone path during the test bench.
static uint8_t s_spk_addr = 0;
static uint8_t s_spk_iface = 0;
static bool s_spk_known = false;
static uac_host_device_handle_t s_spk_handle = NULL;
static TaskHandle_t s_spk_writer_task = NULL;
static volatile bool s_spk_writer_stop = false;  // permanent shutdown
static volatile bool s_spk_writer_run = false;   // per-TX gate
static uint8_t* s_spk_pump_buf = nullptr;
static uint32_t s_spk_packets_sent = 0;
static uint32_t s_spk_write_errors = 0;

// Speaker pump health counters. The verifier was bit-exact (591k frames,
// 0 errors) but visible waterfall artifacts on both the impersonator and
// real QMX point at PTX FIFO underruns — brief gaps when the driver
// ringbuffer empties between SOFs, hardware emits an empty/short ISO
// packet, audio gets a 1 ms blip, receiver shows wideband scatter.
//
//   tx_done_count:   driver ringbuffer fell below threshold (1 KB).
//                    Approaching-underrun event, fires often even on a
//                    healthy stream — useful as a rate signal.
//   write_ok_count:  uac_host_device_write returned ESP_OK promptly
//                    (ringbuffer had room — writer ahead of consumer).
//   write_to_count:  uac_host_device_write hit ESP_ERR_TIMEOUT (ring-
//                    buffer was full — backpressure, healthy state).
//
// A healthy run should look mostly like write_to_count >> write_ok_count
// (writer pinned by ringbuffer-full backpressure, refilling at consumer
// pace). If write_ok_count dominates, the ringbuffer is starving and
// underruns are likely.
static uint32_t s_spk_tx_done_count = 0;
static uint32_t s_spk_write_ok_count = 0;
static uint32_t s_spk_write_to_count = 0;
// Write-elapsed-time histogram. The 14 KB pump write blocks waiting for
// ringbuffer drain at the consumer's pace. Steady state should be ~30-50 ms
// per write. Spikes >55 ms = scheduling stall long enough to drain the
// 16 KB ringbuffer = PTX FIFO underrun = audible glitch.
static uint32_t s_spk_write_max_ms = 0;
static uint32_t s_spk_write_lt_30ms = 0;   // fast — buffer was empty (underrun risk)
static uint32_t s_spk_write_30_55ms = 0;   // healthy steady state
static uint32_t s_spk_write_55_100ms = 0;  // stalled — possible underrun already
static uint32_t s_spk_write_gt_100ms = 0;  // long stall — definite underrun
static TaskHandle_t s_stream_task_handle = NULL;
static volatile bool s_stop_requested = false;
static char s_status_string[64] = "Idle";
static uac_stream_profile_t s_profile = UAC_PROFILE_QMX;
static uac_active_format_t s_format = {
    .sample_freq = UAC_SAMPLE_RATE,
    .bit_resolution = UAC_BIT_RESOLUTION,
    .channels = UAC_CHANNELS,
};
static bool s_cdc_installed = false;
static int s_cdc_iface = -1;
static int s_cdc_iface_hint = -1;
static int64_t s_cdc_last_attempt_ms = 0;
static constexpr uint16_t k_qmx_vid = 0x0483;
static constexpr uint16_t k_qmx_pid = 0xA34C;

// Debug display buffers
static char s_debug_line1[64] = "";
static char s_debug_line2[64] = "";

// Resampler state
static resample_state_t s_resample_state;
static uint8_t s_latest_waterfall_row[UAC_WATERFALL_ROW_WIDTH] = {0};
static bool s_latest_waterfall_row_valid = false;
static portMUX_TYPE s_latest_waterfall_row_lock = portMUX_INITIALIZER_UNLOCKED;

// Forward declarations
static void usb_lib_task(void* arg);
static void uac_lib_task(void* arg);
static void stream_uac_task(void* arg);
static void cdc_close(void);
static void cdc_try_open(void);
static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx);

// Speaker constants and event callback forward declaration. The
// constants are referenced from uac_lib_task's TX_CONNECTED handler
// (which now pre-opens the speaker on enum) as well as from the
// pump task itself further down in the file.
// Speaker ringbuffer must hold one full pump write atomically (push
// is all-or-none in xRingbuffer). All speaker resources are pre-
// allocated at TX_CONNECTED enum time (handle, ringbuffer, xfer URBs,
// pump buffer, writer task) so per-TX is alloc-free. Sizes are tight
// because we share heap with NimBLE (~64 KB), USB host (~40 KB),
// FFT statics, waterfall, and the mic 4 KB ringbuffer.
//   ringbuffer: 4 KB  (~14 ms of audio at 288 B/ms)
//   pump:       1.7 KB (6 packets × 288 B) — must fit in ringbuffer
//   stack:      3 KB  for the writer task
static const uint32_t SPK_BUFFER_SIZE      = 4000;      // driver ringbuffer
static const uint32_t SPK_BUFFER_THRESHOLD = 600;       // ~2 ms at 48k/24/stereo
static const uint32_t SPK_WRITE_TIMEOUT_MS = 200;
static const uint32_t SPK_PACKET_BYTES     = 288;       // 48 stereo frames × 6 B
static const uint32_t SPK_PUMP_PACKETS     = 6;         // 6 packets per write = 1728 B
static const uint32_t SPK_PUMP_BYTES       = SPK_PACKET_BYTES * SPK_PUMP_PACKETS;  // 1728
static const uint32_t SPK_FRAME_BYTES      = 6;         // L+R, 24-bit each
static void spk_event_cb(uac_host_device_handle_t dev,
                         const uac_host_device_event_t event,
                         void* arg);
static void spk_writer_task(void* arg);
static void cdc_new_dev_cb(usb_device_handle_t usb_dev);

static const char* profile_name(uac_stream_profile_t profile) {
    switch (profile) {
    case UAC_PROFILE_QMX:
        return "qmx_uac";
    case UAC_PROFILE_GENERIC_USB:
        return "usb_uac_generic";
    default:
        return "unknown";
    }
}

// Push waterfall row — zero-copy at freq_osr=1 (our current config).
// `base` points into mon.wf.mag for the most recently written block's
// data. That memory is stable until the next slot boundary (other blocks
// write to different offsets), so downstream consumers (UI scaling here,
// ble_native elsewhere) can read from it safely without us copying.
//
// At freq_osr>1 we still need a scratch buffer to merge the multiple
// frequency-sub bands into one row — but since we're at freq_osr=1, no
// static or stack buffer is allocated here in practice.
static void push_waterfall_latest(const monitor_t& mon) {
    if (mon.wf.num_blocks <= 0 || mon.wf.mag == nullptr) return;
    const int block = mon.wf.num_blocks - 1;
    const int num_bins = mon.wf.num_bins;
    const int freq_osr = mon.wf.freq_osr;
    const uint8_t* base = mon.wf.mag + block * mon.wf.block_stride;

    const uint8_t* row_bins;     // pointer to `num_bins` bytes, one per FFT bin
    if (freq_osr <= 1) {
        row_bins = base;          // already single-band, zero-copy
    } else {
        // Rare path (not used in current 6 kHz config). Keep the merge
        // buffer function-local so freq_osr=1 doesn't pay for BSS.
        static uint8_t collapsed[480];
        for (int b = 0; b < num_bins; ++b) {
            uint8_t v = 0;
            for (int fs = 0; fs < freq_osr; ++fs) {
                uint8_t val = base[fs * num_bins + b];
                if (val > v) v = val;
            }
            collapsed[b] = v;
        }
        row_bins = collapsed;
    }

    constexpr int width = UAC_WATERFALL_ROW_WIDTH;
    static uint8_t scaled[width];
    for (int x = 0; x < width; ++x) {
        int start = (int)((int64_t)x * num_bins / width);
        int end = (int)((int64_t)(x + 1) * num_bins / width);
        if (end <= start) end = start + 1;
        uint8_t maxv = 0;
        for (int s = start; s < end && s < num_bins; ++s) {
            if (row_bins[s] > maxv) maxv = row_bins[s];
        }
        scaled[x] = maxv;
    }

    ui_push_waterfall_row(scaled, width);
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memcpy(s_latest_waterfall_row, scaled, width);
    s_latest_waterfall_row_valid = true;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);

    // Stubbed swr/pwr/ptt until real polling lands (see
    // NATIVE_CLIENT_ARCHITECTURE.md). row_bins points into mon.wf.mag
    // (zero-copy) so the callback receives a pointer into authoritative
    // waterfall storage — valid until the next slot boundary.
    static int wf_log_counter = 0;
    if ((wf_log_counter++ % 60) == 0) {
        ESP_LOGI(TAG, "push_waterfall_latest: block=%d num_bins=%d", block, num_bins);
    }
    core_fire_waterfall_row(block, row_bins, num_bins,
                            /*swr=*/1.5f, /*pwr=*/2.0f, /*ptt=*/false);
}

// CDC-ACM helpers (CAT TX only)
static void cdc_close(void) {
    if (s_cdc_handle) {
        cdc_acm_host_close(s_cdc_handle);
        s_cdc_handle = NULL;
    }
    s_cdc_iface = -1;
    s_cdc_last_attempt_ms = 0;
    s_cdc_iface_hint = -1;
}

static void cdc_event_cb(const cdc_acm_host_dev_event_data_t* event, void* user_ctx) {
    (void)user_ctx;
    if (!event) return;
    switch (event->type) {
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGI(TAG, "CDC device disconnected");
        cdc_close();
        break;
    case CDC_ACM_HOST_ERROR:
        ESP_LOGW(TAG, "CDC device error: %d", event->data.error);
        break;
    default:
        break;
    }
}

static void cdc_try_open(void) {
    if (!s_cdc_installed) return;
    if (s_cdc_handle) return;

    // Throttle attempts
    int64_t now_ms = rtc_now_ms();
    if (s_cdc_last_attempt_ms != 0 && (now_ms - s_cdc_last_attempt_ms) < 1000) return;
    s_cdc_last_attempt_ms = now_ms;

    cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 64,   // small TX buffer; RX disabled
        .in_buffer_size = 0,
        .event_cb = cdc_event_cb,
        .data_cb = NULL,
        .user_arg = NULL,
    };

    const int max_iface_scan = 12;

    // QMX profile: try known QMX CAT (VID/PID, iface 0) first.
    if (s_profile == UAC_PROFILE_QMX) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(k_qmx_vid, k_qmx_pid, 0, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = 0;
            ESP_LOGI(TAG, "CDC-ACM opened (QMX iface 0, VID 0x%04x PID 0x%04x)", k_qmx_vid, k_qmx_pid);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            return;
        } else if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "CDC open QMX iface 0 failed: %s", esp_err_to_name(err));
        }
    }

    // Try hinted CDC interface first (if we saw class 0x02)
    if (s_cdc_iface_hint >= 0) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID,
                                          (uint8_t)s_cdc_iface_hint, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = s_cdc_iface_hint;
            ESP_LOGI(TAG, "CDC-ACM opened (hint iface %d)", s_cdc_iface_hint);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            return;
        } else {
            ESP_LOGW(TAG, "CDC open hint iface %d failed: %s",
                     s_cdc_iface_hint, esp_err_to_name(err));
        }
    }

    for (int iface = 0; iface < max_iface_scan; ++iface) {
        cdc_acm_dev_hdl_t handle = NULL;
        esp_err_t err = cdc_acm_host_open(CDC_HOST_ANY_VID, CDC_HOST_ANY_PID,
                                          (uint8_t)iface, &dev_cfg, &handle);
        if (err == ESP_OK) {
            s_cdc_handle = handle;
            s_cdc_iface = iface;
            ESP_LOGI(TAG, "CDC-ACM opened (iface %d)", iface);
            cdc_acm_host_desc_print(handle);
            g_cdc_initial_sync_pending = true;  // main loop will auto-sync VFO + RX mode
            break;
        } else if (err != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "CDC open iface %d failed: %s", iface, esp_err_to_name(err));
        }
    }

    if (!s_cdc_handle) {
        ESP_LOGD(TAG, "CDC-ACM not found yet (profile=%s attempt at %" PRId64 " ms)",
                 profile_name(s_profile), now_ms);
    }
}

static void cdc_new_dev_cb(usb_device_handle_t usb_dev) {
    usb_device_info_t info = {};
    if (usb_host_device_info(usb_dev, &info) == ESP_OK) {
        ESP_LOGI(TAG, "USB dev addr:%u speed:%d", info.dev_addr, info.speed);
    }

    const usb_device_desc_t* dev_desc = nullptr;
    if (usb_host_get_device_descriptor(usb_dev, &dev_desc) == ESP_OK && dev_desc) {
        ESP_LOGI(TAG, "USB dev attached: VID:0x%04x PID:0x%04x cfgs:%u",
                 dev_desc->idVendor, dev_desc->idProduct, dev_desc->bNumConfigurations);
    }

    const usb_config_desc_t* cfg = nullptr;
    if (usb_host_get_active_config_descriptor(usb_dev, &cfg) == ESP_OK && cfg) {
        const uint8_t* p = (const uint8_t*)cfg;
        int offset = 0;
        while (offset + 2 <= cfg->wTotalLength) {
            uint8_t len = p[offset];
            uint8_t dtype = p[offset + 1];
            if (len == 0) break;
            if (dtype == USB_B_DESCRIPTOR_TYPE_INTERFACE && len >= sizeof(usb_intf_desc_t)) {
                const usb_intf_desc_t* intf = (const usb_intf_desc_t*)(p + offset);
                ESP_LOGI(TAG, "  IF num=%u alt=%u eps=%u class=0x%02x subclass=0x%02x proto=0x%02x",
                         intf->bInterfaceNumber, intf->bAlternateSetting,
                         intf->bNumEndpoints, intf->bInterfaceClass,
                         intf->bInterfaceSubClass, intf->bInterfaceProtocol);
                if (intf->bInterfaceClass == USB_CLASS_COMM && s_cdc_iface_hint < 0) {
                    s_cdc_iface_hint = intf->bInterfaceNumber;
                    ESP_LOGI(TAG, "  -> CDC candidate iface %d", s_cdc_iface_hint);
                }
            }
            offset += len;
        }
    }
}

// UAC device callback
static void uac_device_callback(uac_host_device_handle_t handle,
                                 const uac_host_device_event_t event,
                                 void* arg) {
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "UAC device disconnected");
        cdc_close();
        if (handle == s_mic_handle) {
            s_mic_handle = NULL;
            s_state = UAC_STATE_WAITING;
            snprintf(s_status_string, sizeof(s_status_string), "Disconnected");
        }
        uac_host_device_close(handle);
        return;
    }

    uac_event_t evt = {};
    evt.type = UAC_EVT_DEVICE;
    evt.device.handle = handle;
    evt.device.event = event;
    xQueueSend(s_event_queue, &evt, 0);
}

// UAC driver callback
static void uac_driver_callback(uint8_t addr, uint8_t iface_num,
                                 const uac_host_driver_event_t event,
                                 void* arg) {
    ESP_LOGI(TAG, "UAC driver callback - addr:%d, iface:%d, event:%d", addr, iface_num, event);

    uac_event_t evt = {};
    evt.type = UAC_EVT_DRIVER;
    evt.driver.addr = addr;
    evt.driver.iface_num = iface_num;
    evt.driver.event = event;
    xQueueSend(s_event_queue, &evt, 0);
}

// USB host library task
static void usb_lib_task(void* arg) {
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

    // Custom FIFO partitioning to enable simultaneous bidirectional ISO
    // streaming with QMX. ESP32-S3's FS host has a 200-line (800-byte)
    // FIFO total. Built-in Kconfig biases give either RX 608 / PTX 128
    // (BIAS_IN, our previous setting) or RX 136 / PTX 600 — neither
    // covers QMX's 24-bit/48k/stereo MPS=300 in both directions at once.
    //
    // This split reserves 364 B for RX and 364 B for PTX (91 lines each)
    // plus 72 B for non-periodic OUT (CDC CAT bulk-OUT). 364 B is enough
    // for one 300-byte ISO packet plus 64-byte status overhead — i.e.
    // 1.21x MPS, vs the 2x MPS the USB-OTG programming guide recommends
    // for back-to-back ISO IN reception. Should be fine as long as the
    // USB host task drains each packet within the 1 ms inter-frame
    // window (typical drain takes <100 us); under heavy core 0 load
    // (decode + BLE notification storm) we may see occasional glitches.
    //
    // Test-bench purpose: validate whether this margin is acceptable for
    // a future QMX console design that needs full bidirectional UAC.
    // Empirical canary: waterfall artifacts on the UAC signal generator,
    // and runs of "Decoded 0 unique messages" that don't track band
    // conditions. If those appear, fall back to BIAS_IN or implement a
    // runtime FIFO swap around RX/TX boundaries.
    host_config.fifo_settings_custom.rx_fifo_lines   = 91;   // 364 B
    host_config.fifo_settings_custom.nptx_fifo_lines = 18;   // 72 B
    host_config.fifo_settings_custom.ptx_fifo_lines  = 91;   // 364 B

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install USB host: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "USB init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "USB Host installed");

    // Install CDC-ACM driver (for CAT control) before starting class drivers
    const cdc_acm_host_driver_config_t cdc_cfg = {
        .driver_task_stack_size = 3072,
        .driver_task_priority = 4,
        .xCoreID = 0,
        .new_dev_cb = cdc_new_dev_cb,
    };
    err = cdc_acm_host_install(&cdc_cfg);
    if (err == ESP_OK) {
        s_cdc_installed = true;
        ESP_LOGI(TAG, "CDC-ACM driver installed");
    } else {
        ESP_LOGW(TAG, "CDC-ACM driver install failed: %s", esp_err_to_name(err));
    }

    xTaskNotifyGive((TaskHandle_t)arg);

    while (!s_stop_requested) {
        uint32_t event_flags;
        err = usb_host_lib_handle_events(pdMS_TO_TICKS(100), &event_flags);
        if (err == ESP_OK) {
            if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
                ESP_LOGI(TAG, "No USB clients");
                usb_host_device_free_all();
            }
        }
    }

    if (s_cdc_installed) {
        cdc_close();
        cdc_acm_host_uninstall();
        s_cdc_installed = false;
        ESP_LOGI(TAG, "CDC-ACM driver uninstalled");
    }

    // Drain pending USB host events. usb_host_uninstall() refuses to
    // run (returns ESP_ERR_INVALID_STATE without releasing the USB
    // PHY) if process_pending_flags / lib_event_flags / flags.val is
    // non-zero. cdc_acm_host_uninstall posts a client-detach event
    // that must be pumped through, otherwise the PHY stays claimed
    // and the subsequent TinyUSB device-mode init fails with
    // "selected PHY is in use" — which is exactly what we hit on the
    // no-QMX-after-10s path where no device-disconnect ever
    // pre-drained the queue. Pump until ALL_FREE fires or we time
    // out (defence against a stuck event we can't service anyway).
    for (int i = 0; i < 50; ++i) {
        uint32_t event_flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(20), &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) break;
    }

    ESP_LOGI(TAG, "USB Host uninstalling");
    esp_err_t uerr = usb_host_uninstall();
    if (uerr != ESP_OK) {
        ESP_LOGW(TAG, "usb_host_uninstall: %s", esp_err_to_name(uerr));
    }
    s_usb_task_handle = NULL;
    vTaskDelete(NULL);
}

// UAC class driver task
static void uac_lib_task(void* arg) {
    // Wait for USB host to be ready
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uac_host_driver_config_t uac_config = {
        .create_background_task = true,
        .task_priority = UAC_TASK_PRIORITY,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_driver_callback,
        .callback_arg = NULL
    };

    esp_err_t err = uac_host_install(&uac_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UAC driver: %s", esp_err_to_name(err));
        s_state = UAC_STATE_ERROR;
        snprintf(s_status_string, sizeof(s_status_string), "UAC init failed");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UAC driver installed");
    s_state = UAC_STATE_WAITING;
    snprintf(s_status_string, sizeof(s_status_string), "Waiting for device");

    uac_event_t evt;
    while (!s_stop_requested) {
        if (xQueueReceive(s_event_queue, &evt, pdMS_TO_TICKS(100))) {
            if (evt.type == UAC_EVT_STOP) {
                break;
            } else if (evt.type == UAC_EVT_DRIVER) {
                if (evt.driver.event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
                    ESP_LOGI(TAG, "Microphone connected - addr:%d, iface:%d",
                             evt.driver.addr, evt.driver.iface_num);

                    if (s_mic_handle != NULL) {
                        ESP_LOGW(TAG, "Already have a mic device, ignoring");
                        continue;
                    }

                    uac_host_device_config_t dev_config = {
                        .addr = evt.driver.addr,
                        .iface_num = evt.driver.iface_num,
                        .buffer_size = UAC_BUFFER_SIZE,
                        .buffer_threshold = UAC_BUFFER_THRESHOLD,
                        .callback = uac_device_callback,
                        .callback_arg = NULL
                    };

                    uac_host_device_handle_t handle = NULL;
                    err = uac_host_device_open(&dev_config, &handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
                        snprintf(s_status_string, sizeof(s_status_string), "Open failed");
                        continue;
                    }

                    // Print device info
                    uac_host_printf_device_param(handle);

                    // Start stream format negotiation.
                    // QMX profile keeps the existing strict format.
                    // Generic profile tries mono and 16-bit fallbacks.
                    uac_host_stream_config_t candidates[4];
                    int candidate_count = 0;
                    auto add_candidate = [&](uint8_t ch, uint8_t bits) {
                        candidates[candidate_count].channels = ch;
                        candidates[candidate_count].bit_resolution = bits;
                        candidates[candidate_count].sample_freq = UAC_SAMPLE_RATE;
                        candidates[candidate_count].flags = 0;
                        candidate_count++;
                    };
                    if (s_profile == UAC_PROFILE_GENERIC_USB) {
                        add_candidate(1, 24);
                        add_candidate(1, 16);
                        add_candidate(2, 24);
                        add_candidate(2, 16);
                    } else {
                        add_candidate(UAC_CHANNELS, UAC_BIT_RESOLUTION);
                    }

                    bool started = false;
                    for (int i = 0; i < candidate_count; ++i) {
                        const uac_host_stream_config_t* cfg = &candidates[i];
                        ESP_LOGI(TAG, "Starting stream (profile=%s) candidate %d/%d: %dHz, %d-bit, %dch",
                                 profile_name(s_profile), i + 1, candidate_count,
                                 cfg->sample_freq, cfg->bit_resolution, cfg->channels);
                        err = uac_host_device_start(handle, cfg);
                        if (err == ESP_OK) {
                            s_format.sample_freq = cfg->sample_freq;
                            s_format.bit_resolution = cfg->bit_resolution;
                            s_format.channels = cfg->channels;
                            started = true;
                            ESP_LOGI(TAG, "Selected stream format: %dHz, %d-bit, %dch",
                                     s_format.sample_freq, s_format.bit_resolution, s_format.channels);
                            break;
                        }
                        ESP_LOGW(TAG, "Stream candidate failed: %s", esp_err_to_name(err));
                    }

                    if (!started) {
                        ESP_LOGE(TAG, "Failed to start stream for profile=%s", profile_name(s_profile));
                        snprintf(s_status_string, sizeof(s_status_string), "Format not supported");
                        uac_host_device_close(handle);
                        continue;
                    }

                    s_mic_handle = handle;
                    s_state = UAC_STATE_STREAMING;
                    g_streaming = true;
                    snprintf(s_status_string, sizeof(s_status_string),
                             "Streaming %s %luk/%u/%u",
                             profile_name(s_profile),
                             (unsigned long)(s_format.sample_freq / 1000),
                             s_format.bit_resolution,
                             s_format.channels);

                    // Try to open companion CDC-ACM interface (CAT)
                    cdc_try_open();

                    // Start the audio processing task using a STATIC stack
                    // (BSS) so task creation doesn't depend on finding a
                    // contiguous 8 KB block in a fragmented heap.
                    if (s_stream_task_handle == NULL) {
                        static StackType_t  s_stream_task_stack[STREAM_TASK_STACK_SIZE / sizeof(StackType_t)];
                        static StaticTask_t s_stream_task_tcb;
                        size_t free_before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
                        size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
                        ESP_LOGI(TAG, "Pre-task-create heap: free=%u largest=%u",
                                 (unsigned)free_before, (unsigned)largest);
                        s_stream_task_handle = xTaskCreateStaticPinnedToCore(
                            stream_uac_task, "stream_uac",
                            STREAM_TASK_STACK_SIZE / sizeof(StackType_t), NULL,
                            UAC_STREAM_TASK_PRIORITY,
                            s_stream_task_stack, &s_stream_task_tcb, 1);
                        if (!s_stream_task_handle) {
                            ESP_LOGE(TAG, "stream_uac_task create FAILED "
                                     "(static) free=%u largest=%u",
                                     (unsigned)free_before, (unsigned)largest);
                        }
                    }

                } else if (evt.driver.event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
                    s_spk_addr  = evt.driver.addr;
                    s_spk_iface = evt.driver.iface_num;
                    s_spk_known = true;
                    ESP_LOGI(TAG, "Speaker captured: addr=%u iface=%u",
                             (unsigned)s_spk_addr, (unsigned)s_spk_iface);
                    // Open the speaker NOW (16 KB ringbuffer alloc, DMA-
                    // capable). Heap is healthy at enum time (~31 KB
                    // largest contiguous block); after the FFT static
                    // alloc + BLE notification fragmentation, the largest
                    // contiguous drops to ~1.6 KB and 16 KB allocations
                    // intermittently fail. Hold the handle across all
                    // TX cycles — uac_tx_test_start/stop only flip stream
                    // state (alt-iface set/unset), no realloc per TX.
                    if (!s_spk_handle) {
                        uac_host_device_config_t cfg = {};
                        cfg.addr = s_spk_addr;
                        cfg.iface_num = s_spk_iface;
                        cfg.buffer_size = SPK_BUFFER_SIZE;
                        cfg.buffer_threshold = SPK_BUFFER_THRESHOLD;
                        cfg.callback = spk_event_cb;
                        cfg.callback_arg = nullptr;
                        esp_err_t oerr = uac_host_device_open(&cfg, &s_spk_handle);
                        if (oerr != ESP_OK) {
                            ESP_LOGE(TAG, "spk pre-open failed at enum: %s",
                                     esp_err_to_name(oerr));
                            s_spk_handle = NULL;
                        } else {
                            // Claim iface + alloc xfer URBs + EP slot now,
                            // but pass FLAG_STREAM_SUSPEND_AFTER_START so
                            // no URBs are queued on the bus yet. The
                            // 512-byte aligned xfer_desc_list (2x 512B
                            // for ISOC double-buffering) requires fresh
                            // heap to satisfy alignment — at TX time the
                            // largest contiguous block is <2 KB and
                            // aligned alloc fails. Resume happens at TX
                            // via uac_host_device_resume (cheap, no
                            // realloc). Suspend at TX-end stops URB flow
                            // but keeps everything allocated.
                            uac_host_stream_config_t scfg = {};
                            scfg.channels = 2;
                            scfg.bit_resolution = 24;
                            scfg.sample_freq = 48000;
                            scfg.flags = FLAG_STREAM_SUSPEND_AFTER_START;
                            esp_err_t serr = uac_host_device_start(s_spk_handle, &scfg);
                            if (serr != ESP_OK) {
                                ESP_LOGE(TAG, "spk pre-start failed at enum: %s",
                                         esp_err_to_name(serr));
                                uac_host_device_close(s_spk_handle);
                                s_spk_handle = NULL;
                            } else {
                                ESP_LOGI(TAG, "Speaker pre-opened+claimed (handle=%p, ringbuf=%u B, suspended)",
                                         s_spk_handle, (unsigned)SPK_BUFFER_SIZE);
                                // Allocate the pump buffer + permanent
                                // writer task NOW too, while heap is
                                // healthy. Per-TX cost is just flipping
                                // s_spk_writer_run — no allocs.
                                if (!s_spk_pump_buf) {
                                    s_spk_pump_buf = (uint8_t*)heap_caps_malloc(
                                        SPK_PUMP_BYTES, MALLOC_CAP_DEFAULT);
                                    if (!s_spk_pump_buf) {
                                        ESP_LOGE(TAG, "spk pump buf alloc failed at enum");
                                    }
                                }
                                if (s_spk_pump_buf && !s_spk_writer_task) {
                                    s_spk_writer_stop = false;
                                    s_spk_writer_run = false;
                                    BaseType_t ok = xTaskCreatePinnedToCore(
                                        spk_writer_task, "uac_tx_test",
                                        3072, NULL, 5, &s_spk_writer_task, 1);
                                    if (ok != pdPASS) {
                                        ESP_LOGE(TAG, "spk writer task create failed at enum");
                                        s_spk_writer_task = NULL;
                                    } else {
                                        ESP_LOGI(TAG, "Speaker writer task ready (idle)");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Cleanup
    if (s_mic_handle) {
        uac_host_device_stop(s_mic_handle);
        uac_host_device_close(s_mic_handle);
        s_mic_handle = NULL;
    }
    // Stop the permanent writer task so it exits its loop and frees
    // its TCB before we tear down the stream / pump buffer.
    if (s_spk_writer_task) {
        s_spk_writer_run = false;
        s_spk_writer_stop = true;
        for (int i = 0; i < 50 && s_spk_writer_task; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    if (s_spk_pump_buf) {
        free(s_spk_pump_buf);
        s_spk_pump_buf = nullptr;
    }
    if (s_spk_handle) {
        // Speaker was opened + claimed (suspended) at TX_CONNECTED
        // enum and held for the lifetime of the host driver. Tear it
        // down here before uac_host_uninstall() to release xfer URBs
        // and ringbuffer cleanly.
        uac_host_device_stop(s_spk_handle);
        uac_host_device_close(s_spk_handle);
        s_spk_handle = NULL;
        s_spk_known = false;
    }

    ESP_LOGI(TAG, "UAC driver uninstalling");
    uac_host_uninstall();
    s_uac_task_handle = NULL;
    vTaskDelete(NULL);
}

// Audio streaming and processing task
static void stream_uac_task(void* arg) {
    ESP_LOGI(TAG, "Audio streaming task started");

    // Initialize resampler
    resample_init(&s_resample_state);

    // Initialize FT8 monitor and grab heap working buffers FIRST, before
    // the slot-boundary wait below. NimBLE init runs deferred in the main
    // loop and races against this task creation; if we malloc'd after the
    // wait, NimBLE's controller pool would already have drained the heap
    // and ft8_buffer would come back NULL.
    monitor_config_t mon_cfg = {
        .f_min = 200.0f,
        .f_max = 2900.0f,
        .sample_rate = FT8_SAMPLE_RATE,
        .time_osr = g_time_osr,
        .freq_osr = g_freq_osr,
        .protocol = FTX_PROTOCOL_FT8
    };

    monitor_t mon;
    monitor_init(&mon, &mon_cfg);
    monitor_reset(&mon);

    // USB buffer lives in DMA-capable BSS (s_usb_buffer above). Only the
    // float working buffers are heap-allocated; they don't need DMA capability.
    uint8_t* usb_buffer = s_usb_buffer;
    float*   ft8_buffer = (float*)heap_caps_malloc(sizeof(float) * mon.block_size,
                                                   MALLOC_CAP_DEFAULT);
    float*   temp_dec   = (float*)heap_caps_malloc(sizeof(float) * 512,
                                                   MALLOC_CAP_DEFAULT);
    log_heap("UAC_AFTER_FFT_ALLOC");
    if (!ft8_buffer || !temp_dec) {
        ESP_LOGE(TAG, "Buffer allocation failed: ft8=%p temp=%p",
                 ft8_buffer, temp_dec);
        if (ft8_buffer) free(ft8_buffer);
        if (temp_dec) free(temp_dec);
        monitor_free(&mon);
        s_stream_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Wait until the next 15s boundary so the sample-pump loop below aligns
    // to FT8 slot edges. Buffers were grabbed above so heap state during
    // this sleep doesn't matter.
    {
        int64_t now_ms = rtc_now_ms();
        int64_t rem = now_ms % 15000;
        int64_t wait_ms = (rem < 100) ? 0 : (15000 - rem);
        if (wait_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
        }
    }

    const int target_blocks = 80;
    int ft8_buffer_idx = 0;  // Current position in ft8_buffer
    TickType_t next_wake = xTaskGetTickCount();
    int slot_blocks = 0;
    int64_t slot_idx = rtc_now_ms() / 15000;
    int64_t slot_start_ms = slot_idx * 15000;
    (void)slot_start_ms; // silence unused warning

    while (!s_stop_requested && s_mic_handle != NULL) {
        // Read USB audio data
        uint32_t bytes_read = 0;
        esp_err_t ret = uac_host_device_read(s_mic_handle, usb_buffer,
                                              UAC_READ_BUFFER_SIZE,
                                              &bytes_read,
                                              pdMS_TO_TICKS(200));

        if (ret != ESP_OK || bytes_read == 0) {
            if (ret != ESP_ERR_TIMEOUT && ret != ESP_FAIL) {
                ESP_LOGW(TAG, "USB read error: %s", esp_err_to_name(ret));
            }
            continue;
        }

        int frame_bytes = (s_format.bit_resolution / 8) * s_format.channels;
        if (frame_bytes <= 0) {
            ESP_LOGW(TAG, "Invalid stream format bytes/frame=%d", frame_bytes);
            continue;
        }
        int num_frames = bytes_read / frame_bytes;
        int remainder = bytes_read % frame_bytes;

        // Debug display
        if (num_frames > 0) {
            int32_t val = 0;
            if (s_format.bit_resolution == 24) {
                val = usb_buffer[0] | (usb_buffer[1] << 8) | (usb_buffer[2] << 16);
                if (val & 0x800000) val |= 0xFF000000;
            } else { // 16-bit
                int16_t v16 = (int16_t)(usb_buffer[0] | (usb_buffer[1] << 8));
                val = v16;
            }
            snprintf(s_debug_line1, sizeof(s_debug_line1),
                     "fmt=%lu/%u/%u v=%ld",
                     (unsigned long)s_format.sample_freq,
                     s_format.bit_resolution,
                     s_format.channels,
                     (long)val);
            snprintf(s_debug_line2, sizeof(s_debug_line2),
                     "rd=%lu fb=%d rem=%d", (unsigned long)bytes_read, frame_bytes, remainder);
        }

        if (num_frames == 0) continue;

        // Convert and resample selected USB PCM format -> 6kHz mono float.
        int samples_dec = uac_pcm_to_ft8_samples(&s_resample_state, usb_buffer,
                                                 (int)bytes_read, temp_dec,
                                                 s_format.bit_resolution,
                                                 s_format.channels);

        // Accumulate into ft8_buffer
        for (int i = 0; i < samples_dec && !s_stop_requested; i++) {
            ft8_buffer[ft8_buffer_idx++] = temp_dec[i];

            // When we have a full block (960 samples = 160ms at 6kHz)
            if (ft8_buffer_idx >= mon.block_size) {
                // Apply gain normalization (same as stream_wav.cpp)
                double acc = 0.0;
                for (int j = 0; j < mon.block_size; ++j) {
                    acc += fabsf(ft8_buffer[j]);
                }
                float level = (float)(acc / mon.block_size);
                float gain = (level > 1e-6f) ? 0.1f / level : 1.0f;
                if (gain < 0.1f) gain = 0.1f;
                if (gain > 10.0f) gain = 10.0f;
                for (int j = 0; j < mon.block_size; ++j) {
                    ft8_buffer[j] *= gain;
                }

                // Process through monitor
                if (mon.wf.num_blocks < target_blocks) {
                    monitor_process(&mon, ft8_buffer);
                    push_waterfall_latest(mon);
                }

                // Retry CDC open periodically until success
                if (!s_cdc_handle) {
                    cdc_try_open();
                }

                ft8_buffer_idx = 0;

                // Maintain 160ms timing
                vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(160));

                // Align decode to 15s boundaries based on RTC
                slot_blocks++;
                int64_t now_idx = rtc_now_ms() / 15000;
                if (now_idx != slot_idx) {
                    ESP_LOGI(TAG, "Slot boundary %lld->%lld blocks=%d wf=%d",
                             (long long)slot_idx, (long long)now_idx,
                             slot_blocks, mon.wf.num_blocks);
                    // The slot that just ended (slot_idx) is considered "applied"
                    // whether we decoded it or not — we're moving past it either
                    // way, so subsequent TX slots shouldn't block waiting for it.
                    if (slot_idx > g_decode_applied_slot_idx) {
                        g_decode_applied_slot_idx = slot_idx;
                    }
                    // Reset counters at the boundary
                    slot_idx = now_idx;
                    slot_start_ms = slot_idx * 15000;
                    slot_blocks = 0;
                    mon.wf.num_blocks = 0;
                    monitor_reset(&mon);
                    next_wake = xTaskGetTickCount();
                } else if (slot_blocks >= 79 && mon.wf.num_blocks >= 79) {
                    ESP_LOGI(TAG, "Triggering decode at slot %lld blocks=%d wf=%d",
                             (long long)slot_idx, slot_blocks, mon.wf.num_blocks);
                    if (g_decode_enabled) {
                        g_decode_slot_idx = slot_idx;
                        g_decode_in_progress = true;  // Block TX trigger until decode finishes
                        decode_monitor_results(&mon, &mon_cfg, false);
                        // g_decode_in_progress and g_decode_applied_slot_idx are
                        // both updated at the end of decode_monitor_results.
                    } else {
                        ESP_LOGI(TAG, "Decode paused; skipping");
                        // Decode disabled — still mark as applied so TX isn't
                        // blocked waiting for a decode that won't happen.
                        if (slot_idx > g_decode_applied_slot_idx) {
                            g_decode_applied_slot_idx = slot_idx;
                        }
                    }
                    monitor_reset(&mon);
                    mon.wf.num_blocks = 0;
                    slot_blocks = 0;
                    next_wake = xTaskGetTickCount();
                }
            }
        }
    }

    // Cleanup — usb_buffer is static BSS, no free.
    free(ft8_buffer);
    free(temp_dec);
    monitor_free(&mon);

    g_streaming = false;
    s_stream_task_handle = NULL;
    ESP_LOGI(TAG, "Audio streaming task stopped");
    vTaskDelete(NULL);
}

// Public API implementation
uac_stream_state_t uac_get_state(void) {
    return s_state;
}

bool uac_is_streaming(void) {
    return s_state == UAC_STATE_STREAMING && s_mic_handle != NULL;
}

bool uac_get_latest_waterfall_row(uint8_t* out_row, int out_len) {
    if (!out_row || out_len < UAC_WATERFALL_ROW_WIDTH) return false;
    bool valid = false;
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    valid = s_latest_waterfall_row_valid;
    if (valid) {
        memcpy(out_row, s_latest_waterfall_row, UAC_WATERFALL_ROW_WIDTH);
    }
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);
    return valid;
}

bool uac_start_with_profile(uac_stream_profile_t profile) {
    if (s_state != UAC_STATE_IDLE) {
        ESP_LOGW(TAG, "UAC already started");
        return false;
    }

    s_profile = profile;
    s_cdc_last_attempt_ms = 0;
    s_cdc_iface = -1;
    s_cdc_iface_hint = -1;
    s_format.sample_freq = UAC_SAMPLE_RATE;
    s_format.bit_resolution = UAC_BIT_RESOLUTION;
    s_format.channels = UAC_CHANNELS;
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memset(s_latest_waterfall_row, 0, sizeof(s_latest_waterfall_row));
    s_latest_waterfall_row_valid = false;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);

    ESP_LOGI(TAG, "Starting UAC host profile=%s", profile_name(s_profile));
    s_stop_requested = false;
    resample_init(&s_resample_state);

    // Create event queue
    s_event_queue = xQueueCreate(10, sizeof(uac_event_t));
    if (!s_event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return false;
    }

    // Create UAC task first (it will wait for USB task notification)
    BaseType_t ret = xTaskCreatePinnedToCore(uac_lib_task, "uac_lib",
                                              TASK_STACK_SIZE, NULL,
                                              UAC_TASK_PRIORITY,
                                              &s_uac_task_handle, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UAC task");
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return false;
    }

    // Create USB host task (will notify UAC task when ready)
    ret = xTaskCreatePinnedToCore(usb_lib_task, "usb_lib",
                                   TASK_STACK_SIZE, (void*)s_uac_task_handle,
                                   USB_HOST_TASK_PRIORITY,
                                   &s_usb_task_handle, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create USB task");
        s_stop_requested = true;
        vTaskDelete(s_uac_task_handle);
        s_uac_task_handle = NULL;
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return false;
    }

    s_state = UAC_STATE_WAITING;
    snprintf(s_status_string, sizeof(s_status_string), "Waiting for %s", profile_name(s_profile));
    return true;
}

bool uac_start(void) {
    return uac_start_with_profile(UAC_PROFILE_QMX);
}

bool uac_qmx_detected(void) {
  return s_mic_handle != NULL || s_cdc_handle != NULL;
}

void uac_stop(void) {
    if (s_state == UAC_STATE_IDLE) {
        return;
    }

    ESP_LOGI(TAG, "Stopping UAC host");
    s_stop_requested = true;
    g_streaming = false;
    cdc_close();

    // Send stop event
    if (s_event_queue) {
        uac_event_t evt = {};
        evt.type = UAC_EVT_STOP;
        xQueueSend(s_event_queue, &evt, pdMS_TO_TICKS(100));
    }

    // Wait for tasks to finish
    int timeout = 50;  // 5 seconds
    while ((s_stream_task_handle || s_uac_task_handle || s_usb_task_handle) && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout--;
    }

    if (s_event_queue) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }

    s_state = UAC_STATE_IDLE;
    taskENTER_CRITICAL(&s_latest_waterfall_row_lock);
    memset(s_latest_waterfall_row, 0, sizeof(s_latest_waterfall_row));
    s_latest_waterfall_row_valid = false;
    taskEXIT_CRITICAL(&s_latest_waterfall_row_lock);
    snprintf(s_status_string, sizeof(s_status_string), "Idle");
    ESP_LOGI(TAG, "UAC host stopped");
}

// ===== UAC TX test bench =====
//
// Validates the experimental 364/364 PTX FIFO split by streaming a fixed
// 1.5 kHz pure tone (full-scale 24-bit) to the QMX speaker endpoint at
// 48 kHz/stereo. wMaxPacketSize = 288 B (48 stereo samples × 6 B), so
// the PTX FIFO must hold one full packet plus enough headroom for the
// host scheduler. 364 B = 1.21x MPS — should be sufficient if the host
// task isn't delayed >1 ms between IN packets.
//
// Hooked into radio_control_qmx_get_ops()'s begin_tx / end_tx so a
// normal Mini-FT8 TX slot drives the test (~13 s of continuous OUT
// streaming). Logs packet count and write-error count when stopped.

// SPK_* constants and spk_event_cb forward declaration are at the top
// of the file (above the uac_lib_task that now pre-opens the speaker).

static void spk_event_cb(uac_host_device_handle_t /*dev*/,
                         const uac_host_device_event_t event,
                         void* /*arg*/) {
    if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        s_spk_write_errors++;
        ESP_LOGW(TAG, "spk transfer error");
    } else if (event == UAC_HOST_DEVICE_EVENT_TX_DONE) {
        s_spk_tx_done_count++;
    }
}

static void spk_writer_task(void* /*arg*/) {
    // Permanent task. Created once at speaker enum (when heap is
    // fresh enough for the 4 KB stack alloc). Loops forever, gated
    // on s_spk_writer_run — each TX flips it true; uac_tx_test_stop
    // flips it false. Pump buffer is also pre-allocated at enum.
    //
    // Pump buffer is rendered fresh each iteration via the Q15 NCO
    // (dds_q15). Frequency is updated externally by tx_tick on FT8
    // symbol boundaries via uac_tx_set_tone_hz(); the NCO snapshots
    // the current increment at the start of each render block so
    // mid-block frequency changes wait one block (~48 ms < 1 FT8
    // symbol of 160 ms, so we never miss a symbol).
    const uint32_t frames_per_block = SPK_PUMP_BYTES / SPK_FRAME_BYTES;  // 2304
    while (!s_spk_writer_stop) {
        if (!s_spk_writer_run) {
            // Idle between TX cycles. Sleep cheaply.
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        // Render one block worth of samples from the NCO. tx_tick has
        // already set the right frequency for the current FT8 symbol.
        dds_render_24bit_stereo(s_spk_pump_buf, frames_per_block);
        int64_t t0 = esp_timer_get_time();
        esp_err_t err = uac_host_device_write(s_spk_handle, s_spk_pump_buf, SPK_PUMP_BYTES,
                                              pdMS_TO_TICKS(SPK_WRITE_TIMEOUT_MS));
        uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (elapsed_ms > s_spk_write_max_ms) s_spk_write_max_ms = elapsed_ms;
        if      (elapsed_ms < 30)  s_spk_write_lt_30ms++;
        else if (elapsed_ms < 55)  s_spk_write_30_55ms++;
        else if (elapsed_ms < 100) s_spk_write_55_100ms++;
        else                       s_spk_write_gt_100ms++;
        if (err == ESP_OK) {
            s_spk_packets_sent += SPK_PUMP_PACKETS;
            s_spk_write_ok_count++;
        } else if (err == ESP_ERR_TIMEOUT) {
            // Ringbuffer was still full at end of timeout — backpressure
            // is what we WANT (means the consumer is keeping up and the
            // writer doesn't get ahead). Healthy steady-state log.
            s_spk_write_to_count++;
        } else {
            s_spk_write_errors++;
            if (s_spk_write_errors <= 5) {
                ESP_LOGW(TAG, "spk write err=%s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // Yield so IDLE0 can run (watchdog appeasement).
        vTaskDelay(1);
    }

    s_spk_writer_task = NULL;
    vTaskDelete(NULL);
}

bool uac_tx_test_start(void) {
    // Everything (handle, iface claim, xfer URBs, ringbuffer, pump
    // buffer, writer task) was allocated at TX_CONNECTED enum time
    // when heap was fresh. Per-TX cost is now just flipping the run
    // flag and resuming the stream — no allocs.
    if (s_spk_writer_run) {
        return true;  // TX already in flight
    }
    if (!s_spk_known || !s_spk_handle || !s_spk_writer_task) {
        ESP_LOGW(TAG, "uac_tx_test: speaker not ready (enum failed?)");
        return false;
    }

    s_spk_packets_sent = 0;
    s_spk_write_errors = 0;
    s_spk_tx_done_count = 0;
    s_spk_write_ok_count = 0;
    s_spk_write_to_count = 0;
    s_spk_write_max_ms = 0;
    s_spk_write_lt_30ms = 0;
    s_spk_write_30_55ms = 0;
    s_spk_write_55_100ms = 0;
    s_spk_write_gt_100ms = 0;

    esp_err_t err = uac_host_device_resume(s_spk_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uac_tx_test: resume spk: %s", esp_err_to_name(err));
        return false;
    }

    // Start each TX slot from phase 0 with a known-silent frequency.
    // tx_tick will set the first symbol's frequency right after this
    // returns. Continuous-phase from there until uac_tx_test_stop().
    dds_reset_phase();
    dds_set_freq_hz(0.0);

    s_spk_writer_run = true;

    ESP_LOGI(TAG, "uac_tx_test: pump started (NCO-driven FT8 audio)");
    return true;
}

void uac_tx_test_stop(void) {
    // No-op if no TX session is in flight.
    if (!s_spk_writer_run) return;

    // Drop the FT8 schedule so the next TX starts from a clean state
    // (otherwise s_ft8_active stays true and the next start would
    // immediately resume mid-message).
    dds_ft8_end();

    // Tell the permanent writer task to go idle. It checks this flag
    // each iteration and falls through to vTaskDelay; we don't kill
    // the task itself — it's the same task across all TXes.
    s_spk_writer_run = false;
    vTaskDelay(pdMS_TO_TICKS(20));  // let the current write iteration settle

    // Suspend the stream — halts URB submission on the bus but keeps
    // iface claimed, EP allocated, xfer URBs and ringbuffer alive.
    // The whole shebang gets released only on USB host teardown
    // (uac_host_device_stop + close in the UAC task cleanup path).
    if (s_spk_handle) {
        uac_host_device_suspend(s_spk_handle);
    }

    ESP_LOGI(TAG, "uac_tx_test: stopped, packets=%u err=%u tx_done=%u "
                  "writes_ok=%u writes_to=%u",
             (unsigned)s_spk_packets_sent, (unsigned)s_spk_write_errors,
             (unsigned)s_spk_tx_done_count,
             (unsigned)s_spk_write_ok_count,
             (unsigned)s_spk_write_to_count);
    ESP_LOGI(TAG, "uac_tx_test: write timing max=%u ms; "
                  "buckets <30ms:%u  30-55ms:%u  55-100ms:%u  >=100ms:%u",
             (unsigned)s_spk_write_max_ms,
             (unsigned)s_spk_write_lt_30ms,
             (unsigned)s_spk_write_30_55ms,
             (unsigned)s_spk_write_55_100ms,
             (unsigned)s_spk_write_gt_100ms);
}

void uac_tx_set_tone_hz(float hz) {
    dds_set_freq_hz(static_cast<double>(hz));
}

void uac_tx_begin_ft8(float base_hz, const uint8_t* symbols) {
    dds_ft8_begin(static_cast<double>(base_hz), symbols);
}

void uac_tx_end_ft8(void) {
    dds_ft8_end();
}

const char* uac_get_status_string(void) {
    return s_status_string;
}

const char* uac_get_debug_line1(void) {
    return s_debug_line1;
}

const char* uac_get_debug_line2(void) {
    return s_debug_line2;
}

bool cat_cdc_ready(void) {
    return s_cdc_handle != NULL;
}

esp_err_t cat_cdc_send(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!s_cdc_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    return cdc_acm_host_data_tx_blocking(s_cdc_handle, data, len, timeout_ms);
}
