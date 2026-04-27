#pragma once

#include <stdbool.h>

bool mic_stream_start(void);
void mic_stream_stop(void);
bool mic_stream_is_streaming(void);
const char* mic_stream_get_status_string(void);
const char* mic_stream_get_debug_line1(void);
const char* mic_stream_get_debug_line2(void);
