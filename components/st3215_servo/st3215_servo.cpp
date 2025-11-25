#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// =====================================================================
// Torque Switch
// =====================================================================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->set_torque(state);
  }
  this->publish_state(state);
}

// =====================================================================
// checksum
// =====================================================================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  // součet od ID po poslední byte před checksumem
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// =====================================================================
// Send raw packet: FF FF ID LEN INST PARAMS... CHK
// LEN = params + 2 (INST + PARAMS), stejně jako v tvých rámcích
// =====================================================================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(5 + params.size() + 1);

  p.push_back(0xFF);
  p.push_back(0xFF);
  p.push_back(id);
  p.push_back(params.size() + 2);  // LEN
  p.push_back(cmd);
  for (auto b : params) p.push_back(b);
  p.push_back(checksum_(p.data(), p.size()));

  this->write_array(p);
  this->flush();
}

// =====================================================================
// Read registers (0x02)
// params: [addr, len]
// → přesně odpovídá: FF FF 01 04 02 38 02 BE
// =====================================================================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(8 + len);

  while (millis() - start < 50) {
    while (this->available()) buf.push_back(this->read());

    if (buf.size() >= 6) {
      // sync na 0xFF 0xFF
      size_t i = 0;
      while (i + 1 < buf.size() && !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) i++;
      if (i > 0) buf.erase(buf.begin(), buf.begin() + i);

      if (buf.size() < 4) continue;

      uint8_t rlen = buf[3];
      if (buf.size() < (size_t)(rlen + 4)) continue;

      uint8_t chk = buf[rlen + 3];
      uint8_t calc = checksum_(buf.data(), rlen + 3);
      if (chk != calc) {
        ESP_LOGW(TAG, "Bad checksum resp (got %02X calc %02X)", chk, calc);
        buf.clear();
        continue;
      }

      uint8_t err = buf[4];
      if (err != 0) {
        ESP_LOGW(TAG, "Servo error %02X", err);
        return false;
      }

      out.assign(buf.begin() + 5, buf.begin() + 5 + len);
      return true;
    }
    delay(1);
  }
  return false;
}

// =====================================================================
// WRITE registrů (0x03)
// ⚠️ DŮLEŽITÉ: Waveshare/STS zápis je:
//   [INST=0x03] [addr] [data...]
// žádný byte "data_len"
// Torque ON: FF FF 01 04 03 28 01 CE
// =====================================================================
void St3215Servo::write_registers_(uint8_t addr,
                                   const std::vector<uint8_t> &data) {
  std::vector<uint8_t> params;
  params.reserve(1 + data.size());
  params.push_back(addr);
  params.insert(params.end(), data.begin(), data.end());
  send_packet_(servo_id_, 0x03, params);
}

// =====================================================================
// SPECIAL multiturn WritePos na 0x2A BEZ data_len
// přesně podle tvého funkčního rámce:
//   FF FF 01 0A 03 2A 32 00 00 01 00 2C 01 67
// params: [0x2A, acc, posL, posH, turnsL, turnsH, speedL, speedH]
// =====================================================================
void St3215Servo::send_multiturn_pos_(uint8_t acc,
                                      uint16_t pos,
                                      int16_t turns,
                                      uint16_t speed) {
  std::vector<uint8_t> params = {
      0x2A,
      acc,
      (uint8_t)(pos & 0xFF),
      (uint8_t)((pos >> 8) & 0xFF),
      (uint8_t)(turns & 0xFF),
      (uint8_t)((turns >> 8) & 0xFF),
      (uint8_t)(speed & 0xFF),
      (uint8_t)((speed >> 8) & 0xFF),
  };
  // LEN = 8 + 2 = 10 → 0x0A, přesně jako v tvém vzorovém rámci
  this->send_packet_(this->servo_id_, 0x03, params);
}

// =====================================================================
// Torque switch wiring
// =====================================================================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (torque_switch_ != nullptr) {
    torque_switch_->set_parent(this);
    torque_switch_->publish_state(torque_on_);
  }
}

void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 setup id=%u", servo_id_);

  // 1) Servo mode (pozicní, tak jak máš ve funkční sekvenci)
  // FF FF 01 04 03 21 00 D6
  write_registers_(0x21, {0x00});
  delay(10);

  // 2) Torque ON
  // FF FF 01 04 03 28 01 CE
  set_torque(true);
}

void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
  ESP_LOGCONFIG(TAG, "  Max angle: %.1f deg", max_angle_);
  ESP_LOGCONFIG(TAG, "  Turns full open: %.3f turns", turns_full_open_);
}

// =====================================================================
// update(): čteme 0x38 (pozice) + 0x3A (turns)
// =====================================================================
void St3215Servo::update() {
  // 1) RAW POSITION (0x38)
  std::vector<uint8_t> data;
  if (!read_registers_(servo_id_, 0x38, 2, data))
    return;

  uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

  // 2) WHOLE TURNS (0x3A)
  int16_t whole_turns = 0;
  std::vector<uint8_t> tdata;
  if (read_registers_(servo_id_, 0x3A, 2, tdata)) {
    whole_turns = (int16_t)(tdata[0] | (tdata[1] << 8));
  }

  // 3) Kombinace – celé otáčky + zlomek z raw
  turns_unwrapped_ = whole_turns + (raw / RAW_PER_TURN);

  // 4) Úhel v rámci 1 otáčky
  float angle_deg = (raw / RAW_PER_TURN) * 360.0f;

  // 5) Publikace
  if (angle_sensor_)
    angle_sensor_->publish_state(angle_deg);
  if (turns_sensor_)
    turns_sensor_->publish_state(turns_unwrapped_);

  if (percent_sensor_ && turns_full_open_ > 0.0f) {
    float pct = (turns_unwrapped_ / turns_full_open_) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    percent_sensor_->publish_state(pct);
  }
}

// =====================================================================
// Torque ON/OFF přes 0x28
// =====================================================================
void St3215Servo::set_torque(bool on) {
  torque_on_ = on;
  // TORQUE_ENABLE = 0x28, 1 byte
  // ON: FF FF 01 04 03 28 01 CE
  // OFF: FF FF 01 04 03 28 00 CF
  write_registers_(0x28, {(uint8_t)(on ? 1 : 0)});

  if (torque_switch_) torque_switch_->publish_state(on);
  if (torque_sensor_) torque_sensor_->publish_state(on ? 1.0f : 0.0f);
}

void St3215Servo::stop() {
  // STOP = torque OFF, žádný skok na pozici
  set_torque(false);
}

// -----------------------------
// Movement helpers
// -----------------------------
void St3215Servo::rotate(bool cw, int speed) {
  float delta = cw ? CW_CCW_STEP_TURNS : -CW_CCW_STEP_TURNS;
  move_relative(delta, speed);
}

void St3215Servo::move_relative(float turns_delta, int speed) {
  if (speed < 0) speed = 0;
  if (speed > 3400) speed = 3400;
  if (!torque_on_) set_torque(true);

  // Cíl v absolutním počtu otáček dle servo počítadla
  float target_turns = turns_unwrapped_ + turns_delta;
  int32_t target_raw_total = (int32_t)lroundf(target_turns * RAW_PER_TURN);

  if (target_raw_total < 0) target_raw_total = 0;
  if (target_raw_total > 0x7FFFFFFF) target_raw_total = 0x7FFFFFFF;

  uint16_t pos = (uint16_t)(target_raw_total % (int32_t)RAW_PER_TURN);      // 0..4095
  int16_t  turns_cnt = (int16_t)(target_raw_total / (int32_t)RAW_PER_TURN); // celé otáčky
  uint16_t spd = (uint16_t)speed;

  // acc = DEFAULT_ACC (0x32) → přesně jako ve tvém testovacím +1 turn rámci
  this->send_multiturn_pos_(DEFAULT_ACC, pos, turns_cnt, spd);
}

void St3215Servo::move_to_turns(float turns, int speed) {
  float delta = turns - turns_unwrapped_;
  move_relative(delta, speed);
}

void St3215Servo::set_angle(float angle_deg, int speed) {
  float turns_target = angle_deg / 360.0f;
  float delta = turns_target - turns_unwrapped_;
  move_relative(delta, speed);
}

void St3215Servo::move_to_percent(float percent, int speed) {
  if (turns_full_open_ <= 0.0f) return;
  float turns_target = (percent / 100.0f) * turns_full_open_;
  float delta = turns_target - turns_unwrapped_;
  move_relative(delta, speed);
}

}  // namespace st3215_servo
}  // namespace esphome
