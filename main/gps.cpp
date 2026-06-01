#include "gps.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uart_port_t kGpsUartNum = UART_NUM_1;
// GPS UART pins. uart_set_pin() takes (TX, RX) from the ESP's perspective:
// kGpsTxPin = ESP transmits -> module's GPS-RX; kGpsRxPin = ESP receives <- module's GPS-TX.
//
// Default targets the LoRa+GPS Cap for Cardputer ADV (SX1262 + ATGM336H/AT6668),
// whose GPS UART is on G13/G15 per M5 docs: module GPS-TX=G15, GPS-RX=G13, 115200 baud.
// So the ESP must RECEIVE on G15 and TRANSMIT on G13. These pins are free on the ADV
// (old keyboard-scan pins; the ADV uses an I2C keypad). The truSDX uses native USB
// (GPIO19/20), so GPS + radio coexist.
//
// Build with -DGPS_ON_PORTA to instead use the Grove PortA pins (G1/G2) for a
// PortA GPS unit (ESP RX=G1, TX=G2).
#ifdef GPS_ON_PORTA
constexpr gpio_num_t kGpsTxPin = GPIO_NUM_2;
constexpr gpio_num_t kGpsRxPin = GPIO_NUM_1;
#else
constexpr gpio_num_t kGpsTxPin = GPIO_NUM_13;  // ESP TX -> Cap GPS-RX
constexpr gpio_num_t kGpsRxPin = GPIO_NUM_15;  // ESP RX <- Cap GPS-TX
#endif
constexpr int kGpsBaudFast = 115200;
constexpr int kGpsBaudSlow = 9600;
constexpr size_t kGpsLineMax = 128;
constexpr uint32_t kProbeWindowMs = 2500;

const char* kTag = "GPS";

gps_state_t s_state = {};
std::string s_line_buffer;
uint32_t s_probe_start_ms = 0;
uint32_t s_probe_rx_bytes = 0;
bool s_probe_decodable = false;
int s_reported_good_baud = kGpsBaudFast;
bool s_pending_baud_update = false;
int s_pending_baud_value = 0;

inline uint32_t now_ms() {
  return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

int normalize_baud(int baud) {
  return (baud == kGpsBaudSlow) ? kGpsBaudSlow : kGpsBaudFast;
}

int other_baud(int baud) {
  return (normalize_baud(baud) == kGpsBaudFast) ? kGpsBaudSlow : kGpsBaudFast;
}

std::vector<std::string> split_csv(const std::string& s) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= s.size()) {
    size_t comma = s.find(',', start);
    if (comma == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, comma - start));
    start = comma + 1;
  }
  return out;
}

int from_hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool nmea_checksum_ok(const std::string& line, std::string* payload_out) {
  if (line.size() < 9 || line[0] != '$') return false;
  size_t star = line.find('*');
  if (star == std::string::npos || star + 2 >= line.size()) return false;
  const int h0 = from_hex(line[star + 1]);
  const int h1 = from_hex(line[star + 2]);
  if (h0 < 0 || h1 < 0) return false;
  uint8_t expected = (uint8_t)((h0 << 4) | h1);
  uint8_t parity = 0;
  for (size_t i = 1; i < star; ++i) parity ^= (uint8_t)line[i];
  if (parity != expected) return false;
  if (payload_out) *payload_out = line.substr(1, star - 1);
  return true;
}

double parse_nmea_coord(const std::string& val, const std::string& dir) {
  if (val.empty() || dir.empty()) return 0.0;
  double raw = std::strtod(val.c_str(), nullptr);
  int deg = (int)(raw / 100.0);
  double mins = raw - (double)(deg * 100);
  double out = (double)deg + mins / 60.0;
  if (dir[0] == 'S' || dir[0] == 'W') out = -out;
  return out;
}

std::string lat_lon_to_grid(double lat, double lon) {
  if (lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0) return "";
  lon += 180.0;
  lat += 90.0;
  std::string grid = "    ";
  grid[0] = (char)('A' + (int)(lon / 20.0));
  grid[1] = (char)('A' + (int)(lat / 10.0));
  lon = std::fmod(lon, 20.0);
  lat = std::fmod(lat, 10.0);
  grid[2] = (char)('0' + (int)(lon / 2.0));
  grid[3] = (char)('0' + (int)(lat / 1.0));
  return grid;
}

void rearm_probe_window() {
  s_probe_start_ms = now_ms();
  s_probe_rx_bytes = 0;
  s_probe_decodable = false;
  s_line_buffer.clear();
}

bool configure_uart(int baud) {
  uart_config_t cfg = {};
  cfg.baud_rate = normalize_baud(baud);
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#ifdef UART_SCLK_REF_TICK
  cfg.source_clk = UART_SCLK_REF_TICK;
#else
  cfg.source_clk = UART_SCLK_DEFAULT;
#endif
  esp_err_t err = uart_param_config(kGpsUartNum, &cfg);
  if (err != ESP_OK) return false;
  err = uart_set_pin(kGpsUartNum, kGpsTxPin, kGpsRxPin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) return false;
  err = uart_set_line_inverse(kGpsUartNum, (uart_signal_inv_t)0);
  if (err != ESP_OK) return false;
  err = uart_set_baudrate(kGpsUartNum, normalize_baud(baud));
  if (err != ESP_OK) return false;
  uart_flush_input(kGpsUartNum);
  return true;
}

bool ensure_uart_driver() {
  esp_err_t err = uart_driver_install(kGpsUartNum, 2048, 0, 0, nullptr, 0);
  if (err == ESP_ERR_INVALID_STATE) {
    uart_driver_delete(kGpsUartNum);
    err = uart_driver_install(kGpsUartNum, 2048, 0, 0, nullptr, 0);
  }
  return err == ESP_OK;
}

void switch_baud_internal(int baud) {
  baud = normalize_baud(baud);
  if (uart_set_baudrate(kGpsUartNum, baud) != ESP_OK) return;
  uart_flush_input(kGpsUartNum);
  s_state.active_baud = baud;
  s_state.baud_locked = false;
  rearm_probe_window();
  ESP_LOGI(kTag, "GPS probe switch baud=%d", baud);
}

void lock_current_baud_if_needed() {
  if (s_state.baud_locked) return;
  s_state.baud_locked = true;
  if (s_state.active_baud != s_reported_good_baud) {
    s_reported_good_baud = s_state.active_baud;
    s_pending_baud_value = s_state.active_baud;
    s_pending_baud_update = true;
  }
  ESP_LOGI(kTag, "GPS baud lock=%d", s_state.active_baud);
}

bool parse_sentence(const std::string& raw_line) {
  std::string payload;
  if (!nmea_checksum_ok(raw_line, &payload)) return false;

  std::vector<std::string> parts = split_csv(payload);
  if (parts.empty()) return false;

  bool recognized = false;
  const std::string& type = parts[0];
  if (type.size() >= 3 && type.compare(type.size() - 3, 3, "RMC") == 0 && parts.size() >= 10) {
    recognized = true;
    if (parts[2] == "A") {
      s_state.valid_fix = true;
      s_state.latitude = parse_nmea_coord(parts[3], parts[4]);
      s_state.longitude = parse_nmea_coord(parts[5], parts[6]);
      s_state.grid_square = lat_lon_to_grid(s_state.latitude, s_state.longitude);
      if (parts[1].size() >= 6) {
        s_state.time_utc = parts[1].substr(0, 2) + ":" + parts[1].substr(2, 2) + ":" + parts[1].substr(4, 2);
      }
      if (parts[9].size() >= 6) {
        s_state.date_utc = "20" + parts[9].substr(4, 2) + "-" + parts[9].substr(2, 2) + "-" + parts[9].substr(0, 2);
      }
    } else {
      s_state.valid_fix = false;
    }
  } else if (type.size() >= 3 && type.compare(type.size() - 3, 3, "GGA") == 0 && parts.size() >= 8) {
    recognized = true;
    if (!parts[7].empty()) s_state.satellites = std::atoi(parts[7].c_str());
  }

  s_state.last_rx_ms = now_ms();
  s_probe_decodable = true;
  (void)recognized;
  lock_current_baud_if_needed();
  return true;
}

void ingest_uart_bytes(const uint8_t* data, int len) {
  if (len <= 0) return;
  s_probe_rx_bytes += (uint32_t)len;
  for (int i = 0; i < len; ++i) {
    char c = (char)data[i];
    if (c == '\r' || c == '\n') {
      if (!s_line_buffer.empty()) {
        parse_sentence(s_line_buffer);
        s_line_buffer.clear();
      }
      continue;
    }
    if ((unsigned char)c < 32 || (unsigned char)c > 126) continue;
    s_line_buffer.push_back(c);
    if (s_line_buffer.size() > kGpsLineMax) s_line_buffer.clear();
  }
}

}  // namespace

void gps_start(int preload_baud) {
  preload_baud = normalize_baud(preload_baud);
  if (s_state.running) return;

  if (!ensure_uart_driver()) {
    ESP_LOGW(kTag, "GPS UART driver install failed");
    return;
  }
  if (!configure_uart(preload_baud)) {
    ESP_LOGW(kTag, "GPS UART config failed");
    uart_driver_delete(kGpsUartNum);
    return;
  }

  s_state = {};
  s_state.running = true;
  s_state.active_baud = preload_baud;
  s_state.baud_locked = false;
  s_reported_good_baud = preload_baud;
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  rearm_probe_window();
  ESP_LOGI(kTag, "GPS started on UART%d TX=G%d RX=G%d preload=%d",
           (int)kGpsUartNum, (int)kGpsTxPin, (int)kGpsRxPin, preload_baud);
}

void gps_stop() {
  if (!s_state.running) return;
  uart_flush_input(kGpsUartNum);
  uart_driver_delete(kGpsUartNum);
  s_state = {};
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  s_line_buffer.clear();
  s_probe_start_ms = 0;
  s_probe_rx_bytes = 0;
  s_probe_decodable = false;
  ESP_LOGI(kTag, "GPS stopped");
}

void gps_tick() {
  if (!s_state.running) return;

  uint8_t buf[256];
  int len = uart_read_bytes(kGpsUartNum, buf, sizeof(buf), 0);
  if (len > 0) ingest_uart_bytes(buf, len);

  if (!s_state.baud_locked) {
    uint32_t now = now_ms();
    if (s_probe_rx_bytes > 0 &&
        (now - s_probe_start_ms) >= kProbeWindowMs &&
        !s_probe_decodable) {
      switch_baud_internal(other_baud(s_state.active_baud));
    }
  }
}

gps_state_t gps_get_state() {
  return s_state;
}

bool gps_take_baud_update(int* out_baud) {
  if (!s_pending_baud_update) return false;
  if (out_baud) *out_baud = s_pending_baud_value;
  s_pending_baud_update = false;
  s_pending_baud_value = 0;
  return true;
}
