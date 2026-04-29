#include "audio_source.h"

#include "stream_uac.h"

#include "esp_log.h"

static const char* TAG = "AUDIO_SRC";
static audio_source_backend_t s_backend = AUDIO_SOURCE_QMX_UAC;

void audio_source_set_backend(audio_source_backend_t backend) {
    s_backend = backend;
}

audio_source_backend_t audio_source_get_backend(void) {
    return s_backend;
}

const char* audio_source_backend_name(audio_source_backend_t backend) {
    switch (backend) {
    case AUDIO_SOURCE_QMX_UAC:
        return "qmx_uac";
    case AUDIO_SOURCE_USB_UAC_GENERIC:
        return "usb_uac_generic";
    default:
        return "unknown";
    }
}

bool audio_source_start(void) {
    uac_stream_profile_t profile = UAC_PROFILE_QMX;
    if (s_backend == AUDIO_SOURCE_USB_UAC_GENERIC) {
        profile = UAC_PROFILE_GENERIC_USB;
    }

    ESP_LOGI(TAG, "Start audio source backend=%s", audio_source_backend_name(s_backend));
    return uac_start_with_profile(profile);
}

void audio_source_stop(void) {
    uac_stop();
}

bool audio_source_is_streaming(void) {
    return uac_is_streaming();
}

bool audio_source_qmx_detected(void) {
    return uac_qmx_detected();
}

const char* audio_source_get_status_string(void) {
    return uac_get_status_string();
}

const char* audio_source_get_debug_line1(void) {
    return uac_get_debug_line1();
}

const char* audio_source_get_debug_line2(void) {
    return uac_get_debug_line2();
}
