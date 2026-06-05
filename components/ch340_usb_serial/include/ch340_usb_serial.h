#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

class Ch340UsbSerial {
public:
    Ch340UsbSerial();
    ~Ch340UsbSerial();

    Ch340UsbSerial(const Ch340UsbSerial&) = delete;
    Ch340UsbSerial& operator=(const Ch340UsbSerial&) = delete;

    esp_err_t begin(uint32_t baud = 115200);
    void poll();

    bool isReady() const;
    bool isConnected() const;

    int writeBytes(const uint8_t* data, size_t len);
    int readBytes(uint8_t* out, size_t max_len);
    int writeString(const char* s);

    void end();

    uint16_t vid() const;
    uint16_t pid() const;
    uint8_t bulkInEndpoint() const;
    uint8_t bulkOutEndpoint() const;
    int interfaceNumber() const;

    // Diagnostics for the connect-state HUD. driverState(): 0=Idle 1=HostStarted
    // 2=DeviceOpened 3=InterfaceClaimed 4=InitRunning 5=Ready 6=Error 7=Disconnected.
    // bulkInStatus(): last USB bulk-IN transfer status (0=COMPLETED). totalRxBytes():
    // cumulative bytes received from the device since begin().
    int driverState() const;
    int bulkInStatus() const;
    uint32_t totalRxBytes() const;
    uint32_t droppedRxBytes() const;  // diag: cumulative bytes dropped because the RX ring was full

    // Diagnostics for the boot/connect trace. Asks the USB Host stack which device
    // addresses it currently knows about, INDEPENDENT of the NEW_DEV event. This is
    // the key measurement for the enumeration-race bug: in a stuck state, a non-zero
    // count means the stack DID enumerate the device (so the lost NEW_DEV event is
    // the problem and an addr-list adopt would fix it); a zero count means the device
    // never enumerated at all (deeper PHY/edge problem). Returns the number of
    // devices the stack reports (0 if none / host not installed).
    int knownDeviceCount() const;
    // True if NEW_DEV has been received since begin() (i.e. device_connected_ latched).
    bool sawNewDevEvent() const;

    // Open/probe diagnostics (for the reconnect bug). lastOpenStage(): how far
    // openAndProbeDevice() got — 0=not attempted, 1=open, 2=get_dev_desc,
    // 3=get_cfg_desc, 4=claim, 5=startInit/done. lastOpenErr(): esp_err_t of the
    // step that failed (0 if none). These survive after the attempt so the connect
    // code can log exactly which call broke on a reused host.
    int lastOpenStage() const;
    int lastOpenErr() const;

private:
    struct Impl;
    Impl* impl_;
};
