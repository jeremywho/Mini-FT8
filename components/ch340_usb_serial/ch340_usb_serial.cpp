// CH340 USB-serial transport for the truSDX — backed by Espressif's MAINTAINED
// usb_host_cdc_acm + usb_host_ch34x_vcp drivers (C API, no C++ exceptions needed).
//
// This replaces a hand-rolled USB-host CH340 driver whose connect / in-place
// reconnect / stream-teardown lifecycle was the source of a long bug saga
// (enumeration races, interface_claim INVALID_STATE wedges, stream-dies-after-start).
// A standalone spike proved the maintained driver opens reliably, sustains the
// ~7.8 KB/s UA1 audio stream for the full window, and reconnects in place — all of
// which the custom driver failed at. The PUBLIC Ch340UsbSerial interface is unchanged,
// so audio_trusdx_serial.cpp (CAT sequence, UA1; retry, resample/FFT/decode) is untouched.
//
// RX model bridge: the maintained driver delivers RX via a data callback (on the
// cdc-acm client task); we copy into a lock-free SPSC ring that readBytes() drains
// (single producer = callback, single consumer = stream task), mirroring the old
// driver's polled readBytes() contract.

#include "ch340_usb_serial.h"

#include <cstring>
#include <new>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.h"

namespace {
const char* TAG = "CH340VCP";

// Install state at PROCESS scope: Ch340UsbSerial::end() deletes the Impl, so the USB
// host / cdc-acm install + the host daemon task (created once) must outlive it. Kept
// installed across end()/begin() so reconnect is a fast device re-open (the spike's
// validated close+reopen pattern), not a full host teardown.
bool g_usb_host_installed = false;
bool g_cdc_installed = false;
TaskHandle_t g_daemon_task = nullptr;

void host_daemon_task(void* arg) {
    (void)arg;
    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        // ALL_FREE: keep looping so the device can be re-opened (reconnect).
    }
}
}  // namespace

struct Ch340UsbSerial::Impl {
    cdc_acm_dev_hdl_t cdc_hdl_ = nullptr;
    bool open_ = false;
    volatile bool disconnected_ = false;
    int last_open_err_ = 0;
    uint32_t baud_ = 115200;

    static constexpr size_t kRxBufSize = 8192;
    uint8_t rx_buf_[kRxBufSize] = {};
    volatile size_t rx_head_ = 0;   // producer: on_rx (cdc-acm task)
    volatile size_t rx_tail_ = 0;   // consumer: readBytes (stream task)
    volatile uint32_t total_rx_ = 0;
    volatile uint32_t rx_dropped_ = 0;  // diag: bytes the ring couldn't hold (full)

    // --- callbacks (run on the cdc-acm client task) ---
    static bool on_rx(const uint8_t* data, size_t len, void* arg) {
        Impl* self = static_cast<Impl*>(arg);
        if (self && data) self->push(data, len);
        return true;  // processed -> flush the cdc-acm RX buffer
    }
    static void on_event(const cdc_acm_host_dev_event_data_t* e, void* arg) {
        Impl* self = static_cast<Impl*>(arg);
        if (self && e && e->type == CDC_ACM_HOST_DEVICE_DISCONNECTED) {
            self->disconnected_ = true;
        }
    }

    void push(const uint8_t* data, size_t len) {
        size_t i = 0;
        for (; i < len; ++i) {
            size_t next = (rx_head_ + 1) % kRxBufSize;
            if (next == rx_tail_) break;  // ring full -> drop the rest (audio tolerates it)
            rx_buf_[rx_head_] = data[i];
            rx_head_ = next;
        }
        rx_dropped_ += static_cast<uint32_t>(len - i);  // diag: bytes the full ring couldn't hold
        total_rx_ += static_cast<uint32_t>(len);
    }

    bool isReady() const { return open_ && !disconnected_ && cdc_hdl_ != nullptr; }

    esp_err_t begin(uint32_t baud) {
        baud_ = baud;
        disconnected_ = false;
        rx_head_ = rx_tail_ = 0;
        total_rx_ = 0;
        last_open_err_ = 0;
        open_ = false;

        if (!g_usb_host_installed) {
            usb_host_config_t hc = {};
            hc.skip_phy_setup = false;
            hc.intr_flags = ESP_INTR_FLAG_LEVEL1;   // DEFAULT: root port powered at install
            esp_err_t err = usb_host_install(&hc);
            if (err != ESP_OK) { ESP_LOGE(TAG, "usb_host_install %s", esp_err_to_name(err)); last_open_err_ = err; return err; }
            g_usb_host_installed = true;
        }
        if (!g_daemon_task) {
            xTaskCreatePinnedToCore(host_daemon_task, "usbh_daemon", 4096, nullptr, 10, &g_daemon_task, 0);
        }
        if (!g_cdc_installed) {
            esp_err_t err = cdc_acm_host_install(nullptr);
            if (err != ESP_OK) { ESP_LOGE(TAG, "cdc_acm_host_install %s", esp_err_to_name(err)); last_open_err_ = err; return err; }
            g_cdc_installed = true;
        }

        cdc_acm_host_device_config_t dc = {};
        dc.connection_timeout_ms = 3000;  // caller (audio_trusdx_serial) retries begin() in a loop
        dc.out_buffer_size = 256;
        dc.in_buffer_size = 2048;
        dc.event_cb = on_event;
        dc.data_cb = on_rx;
        dc.user_arg = this;

        esp_err_t err = ch34x_vcp_open(CH34X_PID_AUTO, 0, &dc, &cdc_hdl_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ch34x_vcp_open %s", esp_err_to_name(err));
            last_open_err_ = err;
            cdc_hdl_ = nullptr;
            return err;
        }

        cdc_acm_line_coding_t lc = {};
        lc.dwDTERate = baud_;
        lc.bCharFormat = 0;  // 1 stop bit
        lc.bParityType = 0;  // none
        lc.bDataBits = 8;
        cdc_acm_host_line_coding_set(cdc_hdl_, &lc);
        // DTR/RTS low: deliberately DO NOT pulse the truSDX ATmega reset on open (the
        // spike showed the stream is reliable without a reset, and avoids the
        // UA1;-before-reboot dead-pipe race).
        cdc_acm_host_set_control_line_state(cdc_hdl_, false, false);

        open_ = true;
        ESP_LOGI(TAG, "ch34x open OK");
        return ESP_OK;
    }

    void end() {
        if (cdc_hdl_) {
            cdc_acm_host_close(cdc_hdl_);
            cdc_hdl_ = nullptr;
        }
        open_ = false;
        disconnected_ = false;
        // Leave the USB host + cdc-acm driver + daemon installed for a fast reconnect.
    }

    int readBytes(uint8_t* out, size_t max_len) {
        if (!out || max_len == 0) return 0;
        size_t n = 0;
        while (n < max_len && rx_tail_ != rx_head_) {
            out[n++] = rx_buf_[rx_tail_];
            rx_tail_ = (rx_tail_ + 1) % kRxBufSize;
        }
        return static_cast<int>(n);
    }

    int writeBytes(const uint8_t* data, size_t len) {
        if (!isReady() || !data || len == 0) return -1;
        esp_err_t err = cdc_acm_host_data_tx_blocking(cdc_hdl_, data, len, 1000);
        return (err == ESP_OK) ? static_cast<int>(len) : -1;
    }
};

// ----------------------------------------------------------------------------------
// Ch340UsbSerial — thin PIMPL wrapper (unchanged public interface).
// ----------------------------------------------------------------------------------

Ch340UsbSerial::Ch340UsbSerial() : impl_(nullptr) {}
Ch340UsbSerial::~Ch340UsbSerial() { end(); }

esp_err_t Ch340UsbSerial::begin(uint32_t baud) {
    if (!impl_) {
        impl_ = new (std::nothrow) Impl();
        if (!impl_) return ESP_ERR_NO_MEM;
    }
    return impl_->begin(baud);
}

void Ch340UsbSerial::poll() {
    // cdc-acm + the host daemon pump their own tasks; nothing to drive here.
}

bool Ch340UsbSerial::isReady() const { return impl_ && impl_->isReady(); }
bool Ch340UsbSerial::isConnected() const { return impl_ && impl_->isReady(); }

int Ch340UsbSerial::writeBytes(const uint8_t* data, size_t len) { return impl_ ? impl_->writeBytes(data, len) : -1; }
int Ch340UsbSerial::readBytes(uint8_t* out, size_t max_len) { return impl_ ? impl_->readBytes(out, max_len) : 0; }
int Ch340UsbSerial::writeString(const char* s) { return s ? writeBytes(reinterpret_cast<const uint8_t*>(s), std::strlen(s)) : -1; }

void Ch340UsbSerial::end() {
    if (impl_) {
        impl_->end();
        delete impl_;
        impl_ = nullptr;
    }
}

uint16_t Ch340UsbSerial::vid() const { return (impl_ && impl_->open_) ? NANJING_QINHENG_MICROE_VID : 0; }
uint16_t Ch340UsbSerial::pid() const { return (impl_ && impl_->open_) ? CH340_PID_1 : 0; }
uint8_t Ch340UsbSerial::bulkInEndpoint() const { return 0; }
uint8_t Ch340UsbSerial::bulkOutEndpoint() const { return 0; }
int Ch340UsbSerial::interfaceNumber() const { return 0; }

// Diagnostics for the connect HUD / boot-trace. Mapped onto the maintained driver's
// simpler state: 5=Ready(open), 7=Disconnected, 0=Idle.
int Ch340UsbSerial::driverState() const {
    if (!impl_) return 0;
    if (impl_->disconnected_) return 7;
    return impl_->open_ ? 5 : 0;
}
int Ch340UsbSerial::bulkInStatus() const { return 0; }
uint32_t Ch340UsbSerial::totalRxBytes() const { return impl_ ? impl_->total_rx_ : 0; }
uint32_t Ch340UsbSerial::droppedRxBytes() const { return impl_ ? impl_->rx_dropped_ : 0; }

int Ch340UsbSerial::knownDeviceCount() const {
    if (!g_usb_host_installed) return 0;
    uint8_t addrs[8] = {};
    int num = 0;
    if (usb_host_device_addr_list_fill(static_cast<int>(sizeof(addrs)), addrs, &num) == ESP_OK) return num;
    return -1;
}
bool Ch340UsbSerial::sawNewDevEvent() const { return impl_ && impl_->open_; }
int Ch340UsbSerial::lastOpenStage() const { return (impl_ && impl_->open_) ? 5 : 0; }
int Ch340UsbSerial::lastOpenErr() const { return impl_ ? impl_->last_open_err_ : 0; }
