#include "st3215_servo.h"
#include "esphome/core/log.h"

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// =====================================================================
// Torque Switch — ČISTÉ RAW RÁMCE
// =====================================================================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_ != nullptr) {
    parent_->set_torque(state);
  }
  this->publish_state(state);
}

// =====================================================================
// SETUP
// =====================================================================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 ready (RAW MODE), id=%u", servo_id_);

  // MOTOR MODE
  const uint8_t motor_mode[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x21, 0x01, 0xD5};
  this->write_array(motor_mode, sizeof(motor_mode));
  this->flush();
  delay(10);

  // Torque ON po startu
  const uint8_t torque_on[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
  this->write_array(torque_on, sizeof(torque_on));
  this->flush();

  torque_on_ = true;

  if (torque_switch_) torque_switch_->publish_state(true);
}

// =====================================================================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo RAW driver:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
}

// =====================================================================
// POVINNÝ update() — ale _nic nedělá_
// =====================================================================
void St3215Servo::update() {
  // Záměrně prázdné
}

// =====================================================================
// Torque switch wiring — jen propojení objektů
// =====================================================================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (torque_switch_) {
    torque_switch_->set_parent(this);
    torque_switch_->publish_state(torque_on_);
  }
}

// =====================================================================
// TORQUE ON / OFF — JEN RÁMCE
// =====================================================================
void St3215Servo::set_torque(bool on) {

  const uint8_t torque_on[]  = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
  const uint8_t torque_off[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};

  if (on) {
    this->write_array(torque_on, sizeof(torque_on));
    torque_on_ = true;
  } else {
    this->write_array(torque_off, sizeof(torque_off));
    torque_on_ = false;
  }

  this->flush();
}

// =====================================================================
// STOP = RELEASE — NESAHÁ NA SWITCH
// =====================================================================
void St3215Servo::stop() {
  const uint8_t stop_cmd[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};
  this->write_array(stop_cmd, sizeof(stop_cmd));
  this->flush();
}

// =====================================================================
// ROTATE — JEN ČISTÉ RÁMCE
// =====================================================================
void St3215Servo::rotate(bool cw, int speed) {

  if (cw) {
    // ----- DOLŮ -----
    const uint8_t cmd_cw[] = {
      0xFF,0xFF, servo_id_, 0x0A, 0x03, 0x2A,
      0x32, 0x00, 0x00, 0x03, 0x00,
      0x2C, 0x01, 0x65
    };
    this->write_array(cmd_cw, sizeof(cmd_cw));
  } else {
    // ----- NAHORU -----
    const uint8_t cmd_ccw[] = {
      0xFF,0xFF, servo_id_, 0x0A, 0x03, 0x2A,
      0x32, 0x00, 0x00, 0x03, 0x00,
      0xD4, 0xFE, 0xC0
    };
    this->write_array(cmd_ccw, sizeof(cmd_ccw));
  }

  this->flush();
}

}  // namespace st3215_servo
}  // namespace esphome
