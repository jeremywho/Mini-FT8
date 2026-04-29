#include "ch340_usb_serial.h"

#include <new>
#include <stdio.h>
#include <string.h>

#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"

#ifndef CH340_DEBUG
// Keep CH340 transport quiet by default; define CH340_DEBUG=1 for low-level USB logs.
#define CH340_DEBUG 0
#endif

#if CH340_DEBUG
#define CH340_LOG(fmt, ...) do { printf("[ch340] " fmt "\n", ##__VA_ARGS__); fflush(stdout); } while (0)
#else
#define CH340_LOG(fmt, ...) do { } while (0)
#endif

namespace {

static constexpr uint8_t CH341_REQ_WRITE_REG = 0x9A;
static constexpr uint8_t CH341_REQ_MODEM_CTRL = 0xA4;

struct Ch340CtrlCmd {
    uint8_t request;
    uint16_t value;
    uint16_t index;
    const char* name;
};

static const Ch340CtrlCmd kCh340InitSeq[] = {
    { CH341_REQ_WRITE_REG,  0x1312, 0xCC83, "baud 115200" },
    { CH341_REQ_WRITE_REG,  0x2523, 0x00C3, "8N1 rx/tx" },
    { CH341_REQ_MODEM_CTRL, 0xFFFC, 0x0000, "modem dtr/rts" },
};

static constexpr int kCh340InitSeqLen = sizeof(kCh340InitSeq) / sizeof(kCh340InitSeq[0]);
static constexpr size_t kRxBufSize = 512;
static constexpr size_t kDefaultBulkInBytes = 64;

#if CH340_DEBUG
static const char* transferTypeName(uint8_t bmAttributes) {
    switch (bmAttributes & 0x03) {
        case 0x00: return "Control";
        case 0x01: return "Iso";
        case 0x02: return "Bulk";
        case 0x03: return "Intr";
        default: return "?";
    }
}

static const char* endpointDirName(uint8_t bEndpointAddress) {
    return (bEndpointAddress & 0x80) ? "IN" : "OUT";
}
#endif

}  // namespace

struct Ch340UsbSerial::Impl {
    enum class DriverState {
        Idle,
        HostStarted,
        DeviceOpened,
        InterfaceClaimed,
        InitRunning,
        Ready,
        Error,
        Disconnected,
    };

    enum class InitState {
        Idle,
        SendNext,
        WaitDone,
        Done,
        Failed,
    };

    esp_err_t begin(uint32_t baud);
    void poll();
    void end();

    bool isReady() const { return state_ == DriverState::Ready; }
    bool isConnected() const { return device_connected_ || dev_hdl_ != nullptr; }

    int writeBytes(const uint8_t* data, size_t len);
    int readBytes(uint8_t* out, size_t max_len);
    int writeString(const char* s);

    void resetProbeState();
    void resetInitState();
    void resetRxBuffer();
    void freeCompletedTransfers();
    void closeDevice(bool release_interface);

    void parseDeviceDescriptor(const usb_device_desc_t* dev_desc);
    void parseConfigDescriptor(const usb_config_desc_t* cfg_desc);
    bool claimInterface();
    void openAndProbeDevice();

    bool submitControl(const Ch340CtrlCmd& cmd);
    void startInit();
    void pumpInit();

    bool submitBulkIn();
    void processBulkInDone();
    void processBulkOutDone();
    void handleDeviceGone();

    void rxPush(const uint8_t* data, size_t len);

    static void daemonTask(void* arg);
    static void clientEventCb(const usb_host_client_event_msg_t* event_msg, void* arg);
    static void ctrlCb(usb_transfer_t* transfer);
    static void bulkOutCb(usb_transfer_t* transfer);
    static void bulkInCb(usb_transfer_t* transfer);

    DriverState state_ = DriverState::Idle;
    InitState init_state_ = InitState::Idle;

    usb_host_client_handle_t client_hdl_ = nullptr;
    usb_device_handle_t dev_hdl_ = nullptr;
    TaskHandle_t daemon_task_hdl_ = nullptr;

    bool host_installed_ = false;
    bool daemon_stop_ = false;
    bool device_connected_ = false;
    bool device_gone_pending_ = false;
    uint8_t dev_addr_ = 0;

    uint16_t vid_ = 0;
    uint16_t pid_ = 0;
    int ch340_interface_ = -1;
    int ch340_alt_setting_ = 0;
    uint8_t bulk_in_ep_ = 0;
    uint8_t bulk_out_ep_ = 0;
    uint16_t bulk_in_mps_ = 0;
    uint16_t bulk_out_mps_ = 0;
    bool interface_claimed_ = false;

    uint32_t baud_ = 115200;
    int init_index_ = 0;
    usb_transfer_t* ctrl_xfer_ = nullptr;
    bool ctrl_active_ = false;
    bool ctrl_done_ = false;
    usb_transfer_status_t ctrl_status_ = USB_TRANSFER_STATUS_COMPLETED;
    int ctrl_actual_ = 0;

    usb_transfer_t* bulk_out_xfer_ = nullptr;
    bool bulk_out_active_ = false;
    bool bulk_out_done_ = false;
    usb_transfer_status_t bulk_out_status_ = USB_TRANSFER_STATUS_COMPLETED;
    int bulk_out_actual_ = 0;

    usb_transfer_t* bulk_in_xfer_ = nullptr;
    bool bulk_in_active_ = false;
    bool bulk_in_done_ = false;
    usb_transfer_status_t bulk_in_status_ = USB_TRANSFER_STATUS_COMPLETED;
    int bulk_in_actual_ = 0;
    size_t bulk_in_transfer_bytes_ = kDefaultBulkInBytes;

    uint8_t rx_buf_[kRxBufSize] = {};
    size_t rx_head_ = 0;
    size_t rx_tail_ = 0;
};

Ch340UsbSerial::Ch340UsbSerial() : impl_(nullptr) {
}

Ch340UsbSerial::~Ch340UsbSerial() {
    end();
}

esp_err_t Ch340UsbSerial::begin(uint32_t baud) {
    if (!impl_) {
        impl_ = new (std::nothrow) Impl();
        if (!impl_) {
            return ESP_ERR_NO_MEM;
        }
    }
    return impl_->begin(baud);
}

void Ch340UsbSerial::poll() {
    if (impl_) {
        impl_->poll();
    }
}

bool Ch340UsbSerial::isReady() const {
    return impl_ && impl_->isReady();
}

bool Ch340UsbSerial::isConnected() const {
    return impl_ && impl_->isConnected();
}

int Ch340UsbSerial::writeBytes(const uint8_t* data, size_t len) {
    return impl_ ? impl_->writeBytes(data, len) : -1;
}

int Ch340UsbSerial::readBytes(uint8_t* out, size_t max_len) {
    return impl_ ? impl_->readBytes(out, max_len) : 0;
}

int Ch340UsbSerial::writeString(const char* s) {
    return impl_ ? impl_->writeString(s) : -1;
}

void Ch340UsbSerial::end() {
    if (!impl_) {
        return;
    }

    impl_->end();
    delete impl_;
    impl_ = nullptr;
}

uint16_t Ch340UsbSerial::vid() const {
    return impl_ ? impl_->vid_ : 0;
}

uint16_t Ch340UsbSerial::pid() const {
    return impl_ ? impl_->pid_ : 0;
}

uint8_t Ch340UsbSerial::bulkInEndpoint() const {
    return impl_ ? impl_->bulk_in_ep_ : 0;
}

uint8_t Ch340UsbSerial::bulkOutEndpoint() const {
    return impl_ ? impl_->bulk_out_ep_ : 0;
}

int Ch340UsbSerial::interfaceNumber() const {
    return impl_ ? impl_->ch340_interface_ : -1;
}

esp_err_t Ch340UsbSerial::Impl::begin(uint32_t baud) {
    if (client_hdl_) {
        return ESP_OK;
    }

    baud_ = baud;
    if (baud_ != 115200) {
        CH340_LOG("only proven 115200 init sequence is implemented, requested %lu", static_cast<unsigned long>(baud_));
    }

    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    host_config.root_port_unpowered = false;
    host_config.enum_filter_cb = nullptr;

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        CH340_LOG("HOST install err %s", esp_err_to_name(err));
        state_ = DriverState::Error;
        return err;
    }
    host_installed_ = true;
    state_ = DriverState::HostStarted;
    CH340_LOG("HOST installed");

    daemon_stop_ = false;
    BaseType_t task_ok = xTaskCreatePinnedToCore(
        daemonTask,
        "ch340_usb_daemon",
        4096,
        this,
        20,
        &daemon_task_hdl_,
        0
    );
    if (task_ok != pdPASS) {
        CH340_LOG("HOST daemon task create failed");
        usb_host_uninstall();
        host_installed_ = false;
        state_ = DriverState::Error;
        return ESP_ERR_NO_MEM;
    }

    usb_host_client_config_t client_config = {};
    client_config.is_synchronous = false;
    client_config.max_num_event_msg = 5;
    client_config.async.client_event_callback = clientEventCb;
    client_config.async.callback_arg = this;

    err = usb_host_client_register(&client_config, &client_hdl_);
    if (err != ESP_OK) {
        CH340_LOG("CLIENT reg err %s", esp_err_to_name(err));
        end();
        state_ = DriverState::Error;
        return err;
    }

    CH340_LOG("CLIENT registered");
    CH340_LOG("waiting for CH340/CH341 USB serial device");
    return ESP_OK;
}

void Ch340UsbSerial::Impl::poll() {
    if (!client_hdl_) {
        return;
    }

    esp_err_t err = usb_host_client_handle_events(client_hdl_, 0);
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        CH340_LOG("CLIENT ev err %s", esp_err_to_name(err));
    }

    processBulkOutDone();
    processBulkInDone();
    pumpInit();
    handleDeviceGone();

    if (device_connected_ && dev_hdl_ == nullptr && !device_gone_pending_) {
        openAndProbeDevice();
    }

    if (isReady() && !bulk_in_active_ && !bulk_in_done_) {
        submitBulkIn();
    }
}

void Ch340UsbSerial::Impl::end() {
    if (!host_installed_ && !client_hdl_) {
        return;
    }

    for (int i = 0; i < 20; ++i) {
        poll();
        if (!ctrl_active_ && !bulk_out_active_ && !bulk_in_active_) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    closeDevice(true);

    if (client_hdl_) {
        esp_err_t err = usb_host_client_deregister(client_hdl_);
        if (err != ESP_OK) {
            CH340_LOG("CLIENT dereg err %s", esp_err_to_name(err));
        }
        client_hdl_ = nullptr;
    }

    if (host_installed_) {
        daemon_stop_ = true;
        usb_host_lib_unblock();
        for (int i = 0; i < 50 && daemon_task_hdl_ != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        esp_err_t err = usb_host_device_free_all();
        if (err == ESP_ERR_NOT_FINISHED) {
            for (int i = 0; i < 20; ++i) {
                uint32_t flags = 0;
                usb_host_lib_handle_events(pdMS_TO_TICKS(10), &flags);
                if (flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
                    break;
                }
            }
        }

        err = usb_host_uninstall();
        if (err != ESP_OK) {
            CH340_LOG("HOST uninstall err %s", esp_err_to_name(err));
        }
        host_installed_ = false;
    }

    state_ = DriverState::Idle;
}

int Ch340UsbSerial::Impl::writeBytes(const uint8_t* data, size_t len) {
    if (!data && len > 0) {
        return -1;
    }
    if (len == 0) {
        return 0;
    }
    if (!isReady() || !dev_hdl_ || bulk_out_ep_ == 0) {
        return -1;
    }
    if (bulk_out_xfer_ != nullptr || bulk_out_active_) {
        return -2;
    }

    esp_err_t err = usb_host_transfer_alloc(len, 0, &bulk_out_xfer_);
    if (err != ESP_OK) {
        CH340_LOG("OUT alloc err %s", esp_err_to_name(err));
        bulk_out_xfer_ = nullptr;
        return -3;
    }

    memcpy(bulk_out_xfer_->data_buffer, data, len);
    bulk_out_xfer_->device_handle = dev_hdl_;
    bulk_out_xfer_->bEndpointAddress = bulk_out_ep_;
    bulk_out_xfer_->callback = bulkOutCb;
    bulk_out_xfer_->context = this;
    bulk_out_xfer_->num_bytes = static_cast<int>(len);

    bulk_out_done_ = false;
    bulk_out_active_ = true;
    bulk_out_status_ = USB_TRANSFER_STATUS_COMPLETED;
    bulk_out_actual_ = 0;

    err = usb_host_transfer_submit(bulk_out_xfer_);
    if (err != ESP_OK) {
        CH340_LOG("OUT submit err %s", esp_err_to_name(err));
        usb_host_transfer_free(bulk_out_xfer_);
        bulk_out_xfer_ = nullptr;
        bulk_out_active_ = false;
        return -4;
    }

    CH340_LOG("OUT submitted %u bytes", static_cast<unsigned>(len));
    return static_cast<int>(len);
}

int Ch340UsbSerial::Impl::readBytes(uint8_t* out, size_t max_len) {
    if (!out || max_len == 0) {
        return 0;
    }

    size_t copied = 0;
    while (copied < max_len && rx_tail_ != rx_head_) {
        out[copied++] = rx_buf_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1) % kRxBufSize;
    }
    return static_cast<int>(copied);
}

int Ch340UsbSerial::Impl::writeString(const char* s) {
    if (!s) {
        return -1;
    }
    return writeBytes(reinterpret_cast<const uint8_t*>(s), strlen(s));
}

void Ch340UsbSerial::Impl::resetProbeState() {
    ch340_interface_ = -1;
    ch340_alt_setting_ = 0;
    bulk_in_ep_ = 0;
    bulk_out_ep_ = 0;
    bulk_in_mps_ = 0;
    bulk_out_mps_ = 0;
    interface_claimed_ = false;
}

void Ch340UsbSerial::Impl::resetInitState() {
    init_state_ = InitState::Idle;
    init_index_ = 0;
    ctrl_active_ = false;
    ctrl_done_ = false;
    ctrl_status_ = USB_TRANSFER_STATUS_COMPLETED;
    ctrl_actual_ = 0;

    if (ctrl_xfer_) {
        usb_host_transfer_free(ctrl_xfer_);
        ctrl_xfer_ = nullptr;
    }
}

void Ch340UsbSerial::Impl::resetRxBuffer() {
    rx_head_ = 0;
    rx_tail_ = 0;
}

void Ch340UsbSerial::Impl::freeCompletedTransfers() {
    if (ctrl_xfer_ && !ctrl_active_) {
        usb_host_transfer_free(ctrl_xfer_);
        ctrl_xfer_ = nullptr;
    }
    if (bulk_out_xfer_ && !bulk_out_active_) {
        usb_host_transfer_free(bulk_out_xfer_);
        bulk_out_xfer_ = nullptr;
    }
    if (bulk_in_xfer_ && !bulk_in_active_) {
        usb_host_transfer_free(bulk_in_xfer_);
        bulk_in_xfer_ = nullptr;
    }
}

void Ch340UsbSerial::Impl::closeDevice(bool release_interface) {
    if (dev_hdl_ && interface_claimed_ && release_interface) {
        esp_err_t err = usb_host_interface_release(client_hdl_, dev_hdl_, static_cast<uint8_t>(ch340_interface_));
        if (err != ESP_OK) {
            CH340_LOG("RELEASE err %s", esp_err_to_name(err));
        }
    }

    interface_claimed_ = false;

    freeCompletedTransfers();

    if (dev_hdl_) {
        esp_err_t err = usb_host_device_close(client_hdl_, dev_hdl_);
        if (err != ESP_OK) {
            CH340_LOG("CLOSE err %s", esp_err_to_name(err));
        }
        dev_hdl_ = nullptr;
    }

    device_connected_ = false;
    device_gone_pending_ = false;
    dev_addr_ = 0;
    vid_ = 0;
    pid_ = 0;
    resetProbeState();
    resetInitState();
    resetRxBuffer();

    bulk_out_active_ = false;
    bulk_out_done_ = false;
    bulk_in_active_ = false;
    bulk_in_done_ = false;
    state_ = host_installed_ ? DriverState::HostStarted : DriverState::Idle;
}

void Ch340UsbSerial::Impl::parseDeviceDescriptor(const usb_device_desc_t* dev_desc) {
    if (!dev_desc) {
        CH340_LOG("ERR dev desc null");
        return;
    }

    vid_ = dev_desc->idVendor;
    pid_ = dev_desc->idProduct;

    CH340_LOG("VID %04X PID %04X", vid_, pid_);
    CH340_LOG("CLASS %02X USB %04X", dev_desc->bDeviceClass, dev_desc->bcdUSB);

    if (vid_ == 0x1A86) {
        CH340_LOG("WCH VID detected");
    }
    if (vid_ == 0x1A86 && pid_ == 0x7523) {
        CH340_LOG("likely CH340/CH341");
    }
}

void Ch340UsbSerial::Impl::parseConfigDescriptor(const usb_config_desc_t* cfg_desc) {
    if (!cfg_desc) {
        CH340_LOG("ERR cfg desc null");
        return;
    }

    resetProbeState();

    CH340_LOG("CFG intf=%u power=%umA",
              static_cast<unsigned>(cfg_desc->bNumInterfaces),
              static_cast<unsigned>(cfg_desc->bMaxPower * 2));

    const uint8_t* p = reinterpret_cast<const uint8_t*>(cfg_desc);
    const uint8_t* end = p + cfg_desc->wTotalLength;

    int current_interface = -1;
    int current_alt = 0;
    bool current_vendor = false;

    while (p + 2 <= end) {
        const uint8_t len = p[0];
        const uint8_t type = p[1];

        if (len == 0) {
            CH340_LOG("ERR desc len 0");
            break;
        }
        if (p + len > end) {
            CH340_LOG("ERR desc overrun");
            break;
        }

        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            const auto* intf = reinterpret_cast<const usb_intf_desc_t*>(p);

            current_interface = intf->bInterfaceNumber;
            current_alt = intf->bAlternateSetting;
            current_vendor = (intf->bInterfaceClass == 0xFF);

            CH340_LOG("INTF %u alt %u eps %u",
                      static_cast<unsigned>(intf->bInterfaceNumber),
                      static_cast<unsigned>(intf->bAlternateSetting),
                      static_cast<unsigned>(intf->bNumEndpoints));
            CH340_LOG("IClass %02X/%02X/%02X",
                      intf->bInterfaceClass,
                      intf->bInterfaceSubClass,
                      intf->bInterfaceProtocol);

            if (current_vendor && ch340_interface_ < 0) {
                ch340_interface_ = current_interface;
                ch340_alt_setting_ = current_alt;
                CH340_LOG("Use intf %d alt %d", ch340_interface_, ch340_alt_setting_);
            }
        } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT) {
            const auto* ep = reinterpret_cast<const usb_ep_desc_t*>(p);

            const bool is_bulk = ((ep->bmAttributes & 0x03) == 0x02);
            const bool is_in = (ep->bEndpointAddress & 0x80) != 0;

            CH340_LOG("EP %02X %s %s mps %u",
                      ep->bEndpointAddress,
                      endpointDirName(ep->bEndpointAddress),
                      transferTypeName(ep->bmAttributes),
                      static_cast<unsigned>(ep->wMaxPacketSize));

            if (current_vendor && is_bulk) {
                if (is_in && bulk_in_ep_ == 0) {
                    bulk_in_ep_ = ep->bEndpointAddress;
                    bulk_in_mps_ = ep->wMaxPacketSize;
                } else if (!is_in && bulk_out_ep_ == 0) {
                    bulk_out_ep_ = ep->bEndpointAddress;
                    bulk_out_mps_ = ep->wMaxPacketSize;
                }
            }
        }

        p += len;
    }

    CH340_LOG("SUM i=%d in=%02X/%u", ch340_interface_, bulk_in_ep_, static_cast<unsigned>(bulk_in_mps_));
    CH340_LOG("SUM out=%02X/%u", bulk_out_ep_, static_cast<unsigned>(bulk_out_mps_));

    if (ch340_interface_ >= 0 && bulk_in_ep_ != 0 && bulk_out_ep_ != 0) {
        CH340_LOG("DESC PASS");
    } else {
        CH340_LOG("DESC FAIL");
    }
}

bool Ch340UsbSerial::Impl::claimInterface() {
    if (!dev_hdl_) {
        CH340_LOG("CLAIM fail no dev");
        return false;
    }
    if (ch340_interface_ < 0) {
        CH340_LOG("CLAIM fail no intf");
        return false;
    }
    if (bulk_in_ep_ == 0 || bulk_out_ep_ == 0) {
        CH340_LOG("CLAIM fail no eps");
        return false;
    }

    CH340_LOG("CLAIM intf=%d alt=%d", ch340_interface_, ch340_alt_setting_);

    esp_err_t err = usb_host_interface_claim(
        client_hdl_,
        dev_hdl_,
        static_cast<uint8_t>(ch340_interface_),
        static_cast<uint8_t>(ch340_alt_setting_)
    );
    if (err != ESP_OK) {
        CH340_LOG("CLAIM err %s", esp_err_to_name(err));
        return false;
    }

    interface_claimed_ = true;
    state_ = DriverState::InterfaceClaimed;
    CH340_LOG("CLAIM PASS");
    return true;
}

void Ch340UsbSerial::Impl::openAndProbeDevice() {
    CH340_LOG("OPEN addr=%u", static_cast<unsigned>(dev_addr_));

    esp_err_t err = usb_host_device_open(client_hdl_, dev_addr_, &dev_hdl_);
    if (err != ESP_OK) {
        CH340_LOG("OPEN err %s", esp_err_to_name(err));
        return;
    }
    state_ = DriverState::DeviceOpened;

    const usb_device_desc_t* dev_desc = nullptr;
    err = usb_host_get_device_descriptor(dev_hdl_, &dev_desc);
    if (err == ESP_OK && dev_desc) {
        parseDeviceDescriptor(dev_desc);
    } else {
        CH340_LOG("GET dev desc err %s", esp_err_to_name(err));
    }

    const usb_config_desc_t* cfg_desc = nullptr;
    err = usb_host_get_active_config_descriptor(dev_hdl_, &cfg_desc);
    if (err == ESP_OK && cfg_desc) {
        parseConfigDescriptor(cfg_desc);
    } else {
        CH340_LOG("GET cfg desc err %s", esp_err_to_name(err));
    }

    if (!claimInterface()) {
        CH340_LOG("INIT skip no claim");
        state_ = DriverState::Error;
        return;
    }

    startInit();
}

bool Ch340UsbSerial::Impl::submitControl(const Ch340CtrlCmd& cmd) {
    if (!dev_hdl_) {
        CH340_LOG("CTRL no device");
        return false;
    }
    if (ctrl_xfer_ != nullptr || ctrl_active_) {
        CH340_LOG("CTRL busy");
        return false;
    }

    esp_err_t err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t), 0, &ctrl_xfer_);
    if (err != ESP_OK) {
        CH340_LOG("CTRL alloc err %s", esp_err_to_name(err));
        ctrl_xfer_ = nullptr;
        return false;
    }

    auto* setup = reinterpret_cast<usb_setup_packet_t*>(ctrl_xfer_->data_buffer);
    setup->bmRequestType = 0x40;
    setup->bRequest = cmd.request;
    setup->wValue = cmd.value;
    setup->wIndex = cmd.index;
    setup->wLength = 0;

    ctrl_xfer_->device_handle = dev_hdl_;
    ctrl_xfer_->bEndpointAddress = 0;
    ctrl_xfer_->callback = ctrlCb;
    ctrl_xfer_->context = this;
    ctrl_xfer_->num_bytes = sizeof(usb_setup_packet_t);

    ctrl_done_ = false;
    ctrl_active_ = true;
    ctrl_status_ = USB_TRANSFER_STATUS_COMPLETED;
    ctrl_actual_ = 0;

    CH340_LOG("CTRL send %s", cmd.name);
    CH340_LOG("REQ %02X V%04X I%04X", cmd.request, cmd.value, cmd.index);

    err = usb_host_transfer_submit_control(client_hdl_, ctrl_xfer_);
    if (err != ESP_OK) {
        CH340_LOG("CTRL submit err %s", esp_err_to_name(err));
        usb_host_transfer_free(ctrl_xfer_);
        ctrl_xfer_ = nullptr;
        ctrl_active_ = false;
        return false;
    }

    return true;
}

void Ch340UsbSerial::Impl::startInit() {
    if (!interface_claimed_) {
        CH340_LOG("INIT fail no claim");
        init_state_ = InitState::Failed;
        state_ = DriverState::Error;
        return;
    }

    init_index_ = 0;
    init_state_ = InitState::SendNext;
    state_ = DriverState::InitRunning;
    CH340_LOG("INIT start 115200");
}

void Ch340UsbSerial::Impl::pumpInit() {
    if (init_state_ == InitState::Idle || init_state_ == InitState::Done || init_state_ == InitState::Failed) {
        return;
    }

    if (init_state_ == InitState::SendNext) {
        if (init_index_ >= kCh340InitSeqLen) {
            init_state_ = InitState::Done;
            state_ = DriverState::Ready;
            CH340_LOG("INIT PASS");
            return;
        }

        if (!submitControl(kCh340InitSeq[init_index_])) {
            init_state_ = InitState::Failed;
            state_ = DriverState::Error;
            CH340_LOG("INIT submit fail");
            return;
        }

        init_state_ = InitState::WaitDone;
        return;
    }

    if (init_state_ != InitState::WaitDone || !ctrl_done_) {
        return;
    }

    CH340_LOG("CTRL done st=%d act=%d", static_cast<int>(ctrl_status_), ctrl_actual_);

    if (ctrl_xfer_) {
        usb_host_transfer_free(ctrl_xfer_);
        ctrl_xfer_ = nullptr;
    }
    ctrl_done_ = false;

    if (ctrl_status_ != USB_TRANSFER_STATUS_COMPLETED) {
        CH340_LOG("INIT fail status=%d", static_cast<int>(ctrl_status_));
        init_state_ = InitState::Failed;
        state_ = DriverState::Error;
        return;
    }

    init_index_++;
    init_state_ = InitState::SendNext;
}

bool Ch340UsbSerial::Impl::submitBulkIn() {
    if (!dev_hdl_ || bulk_in_ep_ == 0) {
        return false;
    }
    if (bulk_in_active_) {
        return true;
    }

    if (!bulk_in_xfer_) {
        const size_t mps = bulk_in_mps_ > 0 ? bulk_in_mps_ : kDefaultBulkInBytes;
        bulk_in_transfer_bytes_ = ((kDefaultBulkInBytes + mps - 1) / mps) * mps;
        if (bulk_in_transfer_bytes_ == 0) {
            bulk_in_transfer_bytes_ = mps;
        }

        esp_err_t err = usb_host_transfer_alloc(bulk_in_transfer_bytes_, 0, &bulk_in_xfer_);
        if (err != ESP_OK) {
            CH340_LOG("IN alloc err %s", esp_err_to_name(err));
            bulk_in_xfer_ = nullptr;
            return false;
        }
    }

    memset(bulk_in_xfer_->data_buffer, 0, bulk_in_transfer_bytes_);
    bulk_in_xfer_->device_handle = dev_hdl_;
    bulk_in_xfer_->bEndpointAddress = bulk_in_ep_;
    bulk_in_xfer_->callback = bulkInCb;
    bulk_in_xfer_->context = this;
    bulk_in_xfer_->num_bytes = static_cast<int>(bulk_in_transfer_bytes_);

    bulk_in_done_ = false;
    bulk_in_active_ = true;
    bulk_in_status_ = USB_TRANSFER_STATUS_COMPLETED;
    bulk_in_actual_ = 0;

    esp_err_t err = usb_host_transfer_submit(bulk_in_xfer_);
    if (err != ESP_OK) {
        CH340_LOG("IN submit err %s", esp_err_to_name(err));
        bulk_in_active_ = false;
        return false;
    }

    return true;
}

void Ch340UsbSerial::Impl::processBulkInDone() {
    if (!bulk_in_done_) {
        return;
    }

    CH340_LOG("IN done st=%d act=%d", static_cast<int>(bulk_in_status_), bulk_in_actual_);

    if (bulk_in_status_ == USB_TRANSFER_STATUS_COMPLETED && bulk_in_xfer_ && bulk_in_actual_ > 0) {
        rxPush(bulk_in_xfer_->data_buffer, static_cast<size_t>(bulk_in_actual_));
    }

    bulk_in_done_ = false;

    if (bulk_in_status_ == USB_TRANSFER_STATUS_NO_DEVICE || device_gone_pending_) {
        if (bulk_in_xfer_) {
            usb_host_transfer_free(bulk_in_xfer_);
            bulk_in_xfer_ = nullptr;
        }
        return;
    }

    if (bulk_in_status_ != USB_TRANSFER_STATUS_COMPLETED) {
        CH340_LOG("IN status err %d", static_cast<int>(bulk_in_status_));
        if (bulk_in_xfer_) {
            usb_host_transfer_free(bulk_in_xfer_);
            bulk_in_xfer_ = nullptr;
        }
        return;
    }

    if (isReady()) {
        submitBulkIn();
    }
}

void Ch340UsbSerial::Impl::processBulkOutDone() {
    if (!bulk_out_done_) {
        return;
    }

    CH340_LOG("OUT done st=%d act=%d", static_cast<int>(bulk_out_status_), bulk_out_actual_);

    if (bulk_out_xfer_) {
        usb_host_transfer_free(bulk_out_xfer_);
        bulk_out_xfer_ = nullptr;
    }

    bulk_out_done_ = false;
}

void Ch340UsbSerial::Impl::handleDeviceGone() {
    if (!device_gone_pending_) {
        return;
    }

    if (ctrl_active_ || bulk_out_active_ || bulk_in_active_) {
        return;
    }

    CH340_LOG("cleanup disconnected device");
    closeDevice(false);
    state_ = DriverState::Disconnected;
    if (host_installed_) {
        state_ = DriverState::HostStarted;
    }
}

void Ch340UsbSerial::Impl::rxPush(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        const size_t next_head = (rx_head_ + 1) % kRxBufSize;
        if (next_head == rx_tail_) {
            rx_tail_ = (rx_tail_ + 1) % kRxBufSize;
        }
        rx_buf_[rx_head_] = data[i];
        rx_head_ = next_head;
    }
}

void Ch340UsbSerial::Impl::daemonTask(void* arg) {
    auto* self = static_cast<Impl*>(arg);

    while (!self->daemon_stop_) {
        uint32_t event_flags = 0;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(50), &event_flags);

        if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
            CH340_LOG("HOST err %s", esp_err_to_name(err));
        }

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            CH340_LOG("HOST no clients");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            CH340_LOG("HOST all free");
        }
    }

    self->daemon_task_hdl_ = nullptr;
    vTaskDelete(nullptr);
}

void Ch340UsbSerial::Impl::clientEventCb(const usb_host_client_event_msg_t* event_msg, void* arg) {
    auto* self = static_cast<Impl*>(arg);
    if (!self || !event_msg) {
        return;
    }

    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            self->dev_addr_ = event_msg->new_dev.address;
            self->device_connected_ = true;
            self->device_gone_pending_ = false;
            CH340_LOG("EVENT dev addr=%u", static_cast<unsigned>(self->dev_addr_));
            break;

        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            CH340_LOG("EVENT dev gone");
            self->device_connected_ = false;
            self->dev_addr_ = 0;
            self->device_gone_pending_ = true;
            break;

        default:
            CH340_LOG("EVENT unknown %d", static_cast<int>(event_msg->event));
            break;
    }
}

void Ch340UsbSerial::Impl::ctrlCb(usb_transfer_t* transfer) {
    auto* self = static_cast<Impl*>(transfer->context);
    if (!self) {
        return;
    }

    self->ctrl_status_ = transfer->status;
    self->ctrl_actual_ = transfer->actual_num_bytes;
    self->ctrl_done_ = true;
    self->ctrl_active_ = false;
}

void Ch340UsbSerial::Impl::bulkOutCb(usb_transfer_t* transfer) {
    auto* self = static_cast<Impl*>(transfer->context);
    if (!self) {
        return;
    }

    self->bulk_out_status_ = transfer->status;
    self->bulk_out_actual_ = transfer->actual_num_bytes;
    self->bulk_out_done_ = true;
    self->bulk_out_active_ = false;
}

void Ch340UsbSerial::Impl::bulkInCb(usb_transfer_t* transfer) {
    auto* self = static_cast<Impl*>(transfer->context);
    if (!self) {
        return;
    }

    self->bulk_in_status_ = transfer->status;
    self->bulk_in_actual_ = transfer->actual_num_bytes;
    self->bulk_in_done_ = true;
    self->bulk_in_active_ = false;
}
