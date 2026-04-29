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

private:
    struct Impl;
    Impl* impl_;
};
