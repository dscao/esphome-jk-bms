#include "jk_bms_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 12, 0)
#define ADDR_STR(x) x
#else
#define ADDR_STR(x) (x).c_str()
#endif

namespace esphome {
namespace jk_bms_ble {

static const char *const TAG = "jk_bms_ble";

static const uint8_t MAX_NO_RESPONSE_COUNT = 10;

static const uint8_t FRAME_VERSION_JK04 = 0x01;
static const uint8_t FRAME_VERSION_JK02_24S = 0x02;
static const uint8_t FRAME_VERSION_JK02_32S = 0x03;
static const uint8_t FRAME_VERSION_JK_PB = 0x04; // 新增 PB 协议标识

static const uint16_t JK_BMS_SERVICE_UUID = 0xFFE0;
static const uint16_t JK_BMS_CHARACTERISTIC_UUID = 0xFFE1;

static const uint8_t COMMAND_CELL_INFO = 0x96;
static const uint8_t COMMAND_DEVICE_INFO = 0x97;

static const uint16_t MIN_RESPONSE_SIZE = 300;
static const uint16_t MAX_RESPONSE_SIZE = 384 + 16;

static const uint8_t ERRORS_SIZE = 16;
static constexpr const char *const ERRORS[ERRORS_SIZE] = {
    "Charge Overtemperature",               // 0000 0000 0000 0001
    "Charge Undertemperature",              // 0000 0000 0000 0010
    "Coprocessor communication error",      // 0000 0000 0000 0100
    "Cell Undervoltage",                    // 0000 0000 0000 1000
    "Battery pack undervoltage",            // 0000 0000 0001 0000
    "Discharge overcurrent",                // 0000 0000 0010 0000
    "Discharge short circuit",              // 0000 0000 0100 0000
    "Discharge overtemperature",            // 0000 0000 1000 0000
    "Wire resistance",                      // 0000 0001 0000 0000
    "Mosfet overtemperature",               // 0000 0010 0000 0000
    "Cell count is not equal to settings",  // 0000 0100 0000 0000
    "Current sensor anomaly",               // 0000 1000 0000 0000
    "Cell Overvoltage",                     // 0001 0000 0000 0000
    "Battery pack overvoltage",             // 0010 0000 0000 0000
    "Charge overcurrent protection",        // 0100 0000 0000 0000
    "Charge short circuit",                 // 1000 0000 0000 0000
};

uint8_t crc(const uint8_t data[], const uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc + data[i];
  }
  return crc;
}

void JkBmsBle::dump_config() {
  ESP_LOGCONFIG(TAG, "JkBmsBle");
  // ... 此处逻辑保持原样 ...
}

#ifdef USE_ESP32
void JkBmsBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                   esp_ble_gattc_cb_param_t *param) {
  // ... 此处逻辑保持原样 ...
}

void JkBmsBle::update() {
  this->track_online_status_();
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Not connected", ADDR_STR(this->parent_->address_str()));
    return;
  }
  if (!this->status_notification_received_) {
    ESP_LOGI(TAG, "Request status notification");
    this->write_register(COMMAND_CELL_INFO, 0x00000000, 0x00);
  }
}

bool JkBmsBle::write_register(uint8_t address, uint32_t value, uint8_t length) {
  // ... 此处逻辑保持原样 ...
}
#endif

void JkBmsBle::assemble(const uint8_t *data, uint16_t length) {
  if (this->frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Frame dropped because of invalid length");
    this->frame_buffer_.clear();
  }
  if (length >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
    this->frame_buffer_.clear();
  }
  this->frame_buffer_.insert(this->frame_buffer_.end(), data, data + length);
  if (this->frame_buffer_.size() >= MIN_RESPONSE_SIZE) {
    const uint8_t *raw = &this->frame_buffer_[0];
    const uint16_t frame_size = 300; 
    uint8_t computed_crc = crc(raw, frame_size - 1);
    uint8_t remote_crc = raw[frame_size - 1];
    if (computed_crc != remote_crc) {
      ESP_LOGW(TAG, "CRC check failed! 0x%02X != 0x%02X", computed_crc, remote_crc);
      this->frame_buffer_.clear();
      return;
    }
    std::vector<uint8_t> data(this->frame_buffer_.begin(), this->frame_buffer_.end());
    this->decode_(data);
    this->frame_buffer_.clear();
  }
}

void JkBmsBle::decode_(const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();
  uint8_t frame_type = data[4];
  switch (frame_type) {
    case 0x01:
      if (this->protocol_version_ == PROTOCOL_VERSION_JK04) {
        this->decode_jk04_settings_(data);
      } else {
        this->decode_jk02_settings_(data);
      }
      break;
    case 0x02:
      if (this->protocol_version_ == PROTOCOL_VERSION_JK04) {
        this->decode_jk04_cell_info_(data);
      } else if (this->protocol_version_ == PROTOCOL_VERSION_JK_PB) {
        this->decode_jk_pb_cell_info_(data); // 跳转到新版 PB 专用解析
      } else {
        this->decode_jk02_cell_info_(data);
      }
      break;
    case 0x03:
      this->decode_device_info_(data);
      break;
    default:
      ESP_LOGW(TAG, "Unsupported message type (0x%02X)", data[4]);
  }
}

// =========================================================
// 终极绝杀：针对新版 PB 协议的专用 C++ 解析函数实现
// =========================================================
void JkBmsBle::decode_jk_pb_cell_info_(const std::vector<uint8_t> &data) {
  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
  };

  if (data.size() < 200) return;
  ESP_LOGI(TAG, "PB (New Protocol) data frame received");

  // 1. 电芯电压解析 (位置 6 开始，强制读取 32 串空间)
  uint8_t cells_enabled = 0;
  float min_v = 100.0f, max_v = -100.0f, sum_v = 0.0f;
  uint8_t min_idx = 0, max_idx = 0;

  for (uint8_t i = 0; i < 32; i++) {
    float v = (float) jk_get_16bit(i * 2 + 6) * 0.001f;
    if (v > 0.5f && v < 5.0f) {
      sum_v += v;
      cells_enabled++;
      if (v < min_v) { min_v = v; min_idx = i + 1; }
      if (v > max_v) { max_v = v; max_idx = i + 1; }
      this->publish_state_(this->cells_[i].cell_voltage_sensor_, v);
      // 均衡线路电阻 (位置 82 起)
      float res = (float) jk_get_16bit(i * 2 + 82) * 0.001f;
      this->publish_state_(this->cells_[i].cell_resistance_sensor_, res);
    }
  }

  if (cells_enabled > 0) {
    this->publish_state_(this->min_cell_voltage_sensor_, min_v);
    this->publish_state_(this->max_cell_voltage_sensor_, max_v);
    this->publish_state_(this->delta_cell_voltage_sensor_, max_v - min_v);
    this->publish_state_(this->average_cell_voltage_sensor_, sum_v / cells_enabled);
    this->publish_state_(this->min_voltage_cell_sensor_, (float) min_idx);
    this->publish_state_(this->max_voltage_cell_sensor_, (float) max_idx);
  }

  // 2. 核心遥测解析 (严格按照 PB 坐标)
  
  // MOS 温度 (144)
  float mos_t = (float)((int16_t)jk_get_16bit(144)) * 0.1f;
  this->publish_state_(this->power_tube_temperature_sensor_, mos_t);

  // 总电压 (150, mV -> V)
  float total_v = (float) jk_get_32bit(150) * 0.001f;
  this->publish_state_(this->total_voltage_sensor_, total_v);

  // 实时功率 (154, W)
  float pwr = (float)((int32_t)jk_get_32bit(154)) * 0.001f;
  this->publish_state_(this->power_sensor_, pwr);
  this->publish_state_(this->charging_power_sensor_, std::max(0.0f, pwr));
  this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, pwr)));

  // 总电流 (158, mA -> A)
  float curr = (float)((int32_t)jk_get_32bit(158)) * 0.001f;
  this->publish_state_(this->current_sensor_, curr);

  // 电池探头 (162, 164)
  float t1 = (float)((int16_t)jk_get_16bit(162)) * 0.1f;
  float t2 = (float)((int16_t)jk_get_16bit(164)) * 0.1f;
  if (t1 > -50.0f && t1 < 100.0f) this->publish_state_(this->temperatures_[0].temperature_sensor_, t1);
  if (t2 > -50.0f && t2 < 100.0f) this->publish_state_(this->temperatures_[1].temperature_sensor_, t2);

  // SOC (173, Byte)
  this->publish_state_(this->state_of_charge_sensor_, (float)data[173]);

  // 容量 (174: 剩余, 178: 总设置)
  this->publish_state_(this->capacity_remaining_sensor_, (float)jk_get_32bit(174) * 0.001f);
  this->publish_state_(this->total_battery_capacity_setting_sensor_, (float)jk_get_32bit(178) * 0.001f);

  // 循环次数 (182)
  this->publish_state_(this->charging_cycles_sensor_, (float)jk_get_32bit(182));

  // 运行时间 (166)
  uint32_t run_time = jk_get_32bit(166);
  this->publish_state_(this->total_runtime_sensor_, (float)run_time);
  this->publish_state_(this->total_runtime_formatted_text_sensor_, format_total_runtime_(run_time));

  // 3. 开关状态 (198, 199, 200)
  this->publish_state_(this->charging_binary_sensor_, data[198] == 0x01);
  this->publish_state_(this->discharging_binary_sensor_, data[199] == 0x01);
  this->publish_state_(this->balancing_binary_sensor_, data[200] == 0x01);

  this->status_notification_received_ = true;
}

// ... (此处省略原有的 decode_jk02_cell_info_ 以后的一大段逻辑，保持原样即可) ...

}  // namespace jk_bms_ble
}  // namespace esphome
