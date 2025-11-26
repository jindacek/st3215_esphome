#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ================= Torque switch =================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque(state);
  this->publish_state(state);
}

// ================= Setup =========================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 ready, id=%u", servo_id_);

  // MOTOR MODE ONLY (no torque at boot)
  const uint8_t motor_mode[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x21, 0x01, 0xD5};
  this->write_array(motor_mode, sizeof(motor_mode));
  this->flush();

  torque_on_ = false;
}

// ================= Config ========================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
}

// ================= update() ======================
void St3215Servo::update() {

  // ---- RAW POSITION ----
  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) return;
  uint16_t raw = pos[0] | (pos[1] << 8);

  // ---- WHOLE TURNS ----
  std::vector<uint8_t> turns;
  int16_t whole = 0;
  if (read_registers_(servo_id_, 0x3A, 2, turns))
    whole = turns[0] | (turns[1] << 8);

  float total = whole + (raw / RAW_PER_TURN);
  float deg   = (raw / RAW_PER_TURN) * 360.0f;

  // ---- Publish ----
  if (angle_sensor_) angle_sensor_->publish_state(deg);
  if (turns_sensor_) turns_sensor_->publish_state(total);

  if (percent_sensor_ && turns_full_open_ > 0) {
    float pct = (total / turns_full_open_) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    percent_sensor_->publish_state(pct);
  }

  turns_unwrapped_ = total;
}

// ================= Wiring ========================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (s) {
    s->set_parent(this);
    s->publish_state(torque_on_);
  }
}

// ================= Torque =====================
void St3215Servo::set_torque(bool on) {

  const uint8_t torque_on[]  = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
  const uint8_t torque_off[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};

  if (on) {

    // STOP FIRST (prevent runaway)
    const uint8_t stop[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};
    this->write_array(stop, sizeof(stop));
    delay(5);

    this->write_array(torque_on, sizeof(torque_on));
    torque_on_ = true;

  } else {
    this->write_array(torque_off, sizeof(torque_off));
    torque_on_ = false;
  }

  this->flush();
  if (torque_switch_) torque_switch_->publish_state(torque_on_);
}

// ================= STOP BUTTON =====================
void St3215Servo::stop() {
  const uint8_t stop_cmd[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};
  this->write_array(stop_cmd, sizeof(stop_cmd));
  this->flush();
}

// ================= ROTATE =========================
void St3215Servo::rotate(bool cw, int) {

  if (cw) {
    const uint8_t cmd_cw[] = {
      0xFF,0xFF, servo_id_, 0x0A, 0x03, 0x2A,
      0x32, 0x00, 0x00, 0x03, 0x00,
      0x2C, 0x01, 0x65
    };
    this->write_array(cmd_cw, sizeof(cmd_cw));
  } else {
    const uint8_t cmd_ccw[] = {
      0xFF,0xFF, servo_id_, 0x0A, 0x03, 0x2A,
      0x32, 0x00, 0x00, 0x03, 0x00,
      0xD4, 0xFE, 0xC0
    };
    this->write_array(cmd_ccw, sizeof(cmd_ccw));
  }

  this->flush();
}

} // namespace
} // namespace
