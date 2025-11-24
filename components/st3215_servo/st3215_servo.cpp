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
// Helper: checksum for STS packets
// Frame: FF FF ID LEN INST PARAMS... CHK
// checksum = ~(sum from ID through last param) & 0xFF
// =====================================================================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  // sum from ID (index 2) to last byte before checksum
  for (size_t i = 2; i < len; i++) {
    sum += data[i];
  }
  return (~sum) & 0xFF;
}

// =====================================================================
// Send raw packet
// =====================================================================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                              const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(5 + params.size() + 1);

  p.push_back(0xFF);
  p.push_back(0xFF);
  p.push_back(id);
  p.push_back(params.size() + 2);  // LEN includes cmd + checksum
  p.push_back(cmd);

  for (auto b : params)
    p.push_back(b);

  p.push_back(checksum_(p.data(), p.size()));
  this->write_array(p);
  this->flush();
}

// =====================================================================
// Read registers
// Read inst = 0x02, params: [addr, len]
// Response: FF FF ID LEN ERR DATA... CHK
// =====================================================================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(8 + len);

  while (millis() - start < 50) {
    while (this->available()) {
      buf.push_back(this->read());
    }

    if (buf.size() >= 6) {
      // seek header
      size_t i = 0;
      while (i + 1 < buf.size() &&
             !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) {
        i++;
      }
      if (i > 0)
        buf.erase(buf.begin(), buf.begin() + i);

      if (buf.size() < 4)
        continue;

      uint8_t rlen = buf[3];
      if (buf.size() < (size_t)(rlen + 4))
        continue;  // not full yet

      // verify checksum
      uint8_t chk = buf[rlen + 3];
      uint8_t calc = checksum_(buf.data(), rlen + 3);
      if (chk != calc) {
        ESP_LOGW(TAG, "Bad checksum resp (got %02X calc %02X)", chk, calc);
        buf.clear();
        continue;
      }

      if (buf[2] != id) {
        ESP_LOGW(TAG, "Resp from other id %u", buf[2]);
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
// Write registers helper (STS requires [addr, data_len, data...])
// =====================================================================
void St3215Servo::write_registers_(uint8_t addr,
                                   const std::vector<uint8_t> &data) {
  std::vector<uint8_t> params;
  params.reserve(2 + data.size());
  params.push_back(addr);
  params.push_back((uint8_t)data.size());  // IMPORTANT length byte
  params.insert(params.end(), data.begin(), data.end());
  send_packet_(servo_id_, 0x03, params);
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

// =====================================================================
// setup
// =====================================================================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 setup id=%u", servo_id_);

  // Make sure torque is enabled at boot
  set_torque(true);
}

// =====================================================================
// dump_config
// =====================================================================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
  ESP_LOGCONFIG(TAG, "  Max angle: %.1f deg", max_angle_);
  ESP_LOGCONFIG(TAG, "  Turns full open: %.3f turns", turns_full_open_);
}

// =====================================================================
// update (multiturn / unwrapped turns) from present position at 0x38
// =====================================================================
void St3215Servo::update() {
  std::vector<uint8_t> data;
  if (!read_registers_(servo_id_, 0x38, 2, data))
    return;

  uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

  float angle_deg = (raw / RAW_PER_TURN) * 360.0f;
  float turns_now = raw / RAW_PER_TURN;

  if (have_last_) {
    int16_t diff = (int16_t)raw - (int16_t)last_raw_pos_;
    if (diff > 2048)
      turns_unwrapped_ -= 1.0f;
    else if (diff < -2048)
      turns_unwrapped_ += 1.0f;

    turns_unwrapped_ += diff / RAW_PER_TURN;
  } else {
    turns_unwrapped_ = turns_now;
    have_last_ = true;
  }

  last_raw_pos_ = raw;

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
// set_torque (addr 0x28)
// =====================================================================
void St3215Servo::set_torque(bool on) {
  torque_on_ = on;
  write_registers_(0x28, {(uint8_t)(on ? 1 : 0)});

  if (torque_switch_)
    torque_switch_->publish_state(on);
}

// =====================================================================
// stop
// =====================================================================
void St3215Servo::stop() {
  // safe stop = torque off
  set_torque(false);
}

// =====================================================================
// rotate (relative +/- step turns)
// =====================================================================
void St3215Servo::rotate(bool cw, int speed) {
  float delta = cw ? CW_CCW_STEP_TURNS : -CW_CCW_STEP_TURNS;
  move_relative(delta, speed);
}

// =====================================================================
// move_relative using STS WritePosEx (start at 0x29, 7 bytes)
// data: [acc, posL, posH, 0, 0, speedL, speedH]
// =====================================================================
void St3215Servo::move_relative(float turns_delta, int speed) {
  if (speed < 0) speed = 0;
  if (speed > 3400) speed = 3400;

  if (!torque_on_)
    set_torque(true);

  float target_turns = turns_unwrapped_ + turns_delta;
  int32_t target_raw = (int32_t)lroundf(target_turns * RAW_PER_TURN);

  if (target_raw < 0) target_raw = 0;
  if (target_raw > 65535) target_raw = 65535;

  uint16_t pos = (uint16_t)target_raw;
  uint16_t spd = (uint16_t)speed;

  std::vector<uint8_t> data = {
      (uint8_t)DEFAULT_ACC,
      (uint8_t)(pos & 0xFF),
      (uint8_t)((pos >> 8) & 0xFF),
      0x00,
      0x00,
      (uint8_t)(spd & 0xFF),
      (uint8_t)((spd >> 8) & 0xFF),
  };

  write_registers_(0x29, data);
}

// =====================================================================
// set_angle
// =====================================================================
void St3215Servo::set_angle(float angle_deg, int speed) {
  float turns_target = angle_deg / 360.0f;
  float delta = turns_target - turns_unwrapped_;
  move_relative(delta, speed);
}

// =====================================================================
// move_to_percent
// =====================================================================
void St3215Servo::move_to_percent(float percent, int speed) {
  if (turns_full_open_ <= 0.0f)
    return;

  float turns_target = (percent / 100.0f) * turns_full_open_;
  float delta = turns_target - turns_unwrapped_;
  move_relative(delta, speed);
}

}  // namespace st3215_servo
}  // namespace esphome
