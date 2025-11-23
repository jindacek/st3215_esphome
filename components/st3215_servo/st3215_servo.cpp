#include "st3215_servo.h"
#include "esphome/core/log.h"

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// =====================================================================
//  TORQUE SWITCH HANDLER
// =====================================================================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->set_torque(state);
  }
  this->publish_state(state);
}

// =====================================================================
//  SET TORQUE SWITCH
// =====================================================================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (torque_switch_ != nullptr) {
    torque_switch_->set_parent(this);
    torque_switch_->publish_state(torque_on_);
  }
}

// =====================================================================
//  CHECKSUM
// =====================================================================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len; i++)
    sum += data[i];
  return (~sum) & 0xFF;
}

// =====================================================================
//  SEND PACKET
// =====================================================================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(6 + params.size());

  p.push_back(0xFF);
  p.push_back(0xFF);
  p.push_back(id);
  p.push_back(params.size() + 2);  // LENGTH = params + cmd + checksum
  p.push_back(cmd);

  for (auto b : params)
    p.push_back(b);

  p.push_back(checksum_(p.data(), p.size()));

  this->write_array(p);
  this->flush();
}

// =====================================================================
//  READ REGISTERS
// =====================================================================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(16);

  while (millis() - start < 50) {
    while (this->available()) {
      buf.push_back(this->read());
    }

    if (buf.size() >= 6) {
      size_t i = 0;
      while (i + 1 < buf.size() &&
             !(buf[i] == 0xFF && buf[i + 1] == 0xFF))
        i++;

      if (i > 0)
        buf.erase(buf.begin(), buf.begin() + i);

      if (buf.size() < 4)
        continue;

      uint8_t rlen = buf[3];
      if (buf.size() < (rlen + 4))
        continue;

      uint8_t chk = buf[rlen + 3];
      uint8_t calc = checksum_(buf.data(), rlen + 3);

      if (chk != calc) {
        buf.clear();
        continue;
      }

      if (buf[2] != id)
        return false;

      uint8_t err = buf[4];
      if (err != 0)
        return false;

      out.assign(buf.begin() + 5, buf.begin() + 5 + len);
      return true;
    }
  }

  return false;
}

// =====================================================================
//  SETUP
// =====================================================================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 setup id=%u", servo_id_);

  // Torque limit (max)
  send_packet_(servo_id_, 0x03, {0x29, 100});

  // Torque enable
  set_torque(true);
}

// =====================================================================
//  CONFIG
// =====================================================================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
  ESP_LOGCONFIG(TAG, "  Max angle: %.1f deg", max_angle_);
  ESP_LOGCONFIG(TAG, "  Turns full open: %.3f turns", turns_full_open_);
}

// =====================================================================
//  UPDATE (MULTITURN)
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
//  SET TORQUE  <-- TADY BYLO PRÁVĚ NĚCO CHYBĚLO
// =====================================================================
void St3215Servo::set_torque(bool on) {
  torque_on_ = on;

  send_packet_(servo_id_, 0x03, {0x28, (uint8_t)(on ? 1 : 0)});

  if (torque_switch_)
    torque_switch_->publish_state(on);
}

// =====================================================================
//  STOP
// =====================================================================
void St3215Servo::stop() {
  set_torque(false);
}

// =====================================================================
//  ROTATE (CW / CCW)
// =====================================================================
void St3215Servo::rotate(bool cw, int speed) {
  float delta = cw ? CW_CCW_STEP_TURNS : -CW_CCW_STEP_TURNS;
  move_relative(delta, speed);
}

// =====================================================================
//  MOVE RELATIVE
// =====================================================================
void St3215Servo::move_relative(float turns_delta, int speed) {
  if (!torque_on_)
    set_torque(true);

  float target_turns = turns_unwrapped_ + turns_delta;
  int32_t target_raw = (int32_t) lroundf(target_turns * RAW_PER_TURN);

  if (target_raw < 0)
    target_raw = 0;
  if (target_raw > 65535)
    target_raw = 65535;

  uint16_t pos = (uint16_t)target_raw;
  uint16_t spd = (uint16_t)speed;

  std::vector<uint8_t> params = {
      0x2E,
      (uint8_t)(pos & 0xFF),
      (uint8_t)((pos >> 8) & 0xFF),
      (uint8_t)(spd & 0xFF),
      (uint8_t)((spd >> 8) & 0xFF),
      DEFAULT_ACC,
      0x00
  };

  send_packet_(servo_id_, 0x03, params);
}

// =====================================================================
//  SET ANGLE
// =====================================================================
void St3215Servo::set_angle(float angle_deg, int speed) {
  float turns_target = angle_deg / 360.0f;
  float delta = turns_target - turns_unwrapped_;
  move_relative(delta, speed);
}

// =====================================================================
//  MOVE TO PERCENT
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
