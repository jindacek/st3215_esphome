#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ------------------ Torque Switch ------------------
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque(state);
  publish_state(state);
}

// ------------------ Checksum ------------------
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// ------------------ Send Packet ------------------
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.push_back(0xFF);
  p.push_back(0xFF);
  p.push_back(id);
  p.push_back(params.size() + 2);
  p.push_back(cmd);
  for (auto b : params) p.push_back(b);
  p.push_back(checksum_(p.data(), p.size()));

  write_array(p);
  flush();
}

// ------------------ Read Registers ------------------
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;

  while (millis() - start < 50) {
    while (available()) buf.push_back(read());

    if (buf.size() >= 6) {
      size_t i = 0;
      while (i + 1 < buf.size() && !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) i++;
      if (i > 0) buf.erase(buf.begin(), buf.begin() + i);

      if (buf.size() < 4) continue;
      uint8_t rlen = buf[3];
      if (buf.size() < (size_t)(rlen + 4)) continue;

      uint8_t chk = buf[rlen + 3];
      uint8_t calc = checksum_(buf.data(), rlen + 3);

      if (chk != calc) {
        buf.clear();
        continue;
      }

      if (buf[4] != 0) return false;
      out.assign(buf.begin() + 5, buf.begin() + 5 + len);
      return true;
    }
    delay(1);
  }
  return false;
}

// ------------------ Setup ------------------
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 init ID=%u", servo_id_);

  // MOTOR MODE
  const uint8_t motor_mode[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x21,0x01,0xD5};
  write_array(motor_mode, sizeof(motor_mode));
  flush();
  delay(10);

  // Torque ON
  const uint8_t torque_on[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x01,0xCE};
  write_array(torque_on, sizeof(torque_on));
  flush();

  torque_on_ = true;
}

// ------------------ Dump ------------------
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
}

// ------------------ UPDATE ------------------
void St3215Servo::update() {
  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) return;

  uint16_t raw = pos[0] | (pos[1] << 8);
  float frac = raw / RAW_PER_TURN;

  turns_unwrapped_ = frac;

  float total = turns_unwrapped_ - zero_offset_;
  float deg = frac * 360.0f;

  if (angle_sensor_) angle_sensor_->publish_state(deg);
  if (turns_sensor_) turns_sensor_->publish_state(total);

  if (percent_sensor_ && has_max_) {
    float pct = (total / max_turns_) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    percent_sensor_->publish_state(pct);
  }

  // auto-stop
  if (has_zero_ && total <= 0) stop();
  if (has_max_ && total >= max_turns_) stop();
}

// ------------------ Torque ------------------
void St3215Servo::set_torque(bool on) {
  const uint8_t torque_on[]  = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x01,0xCE};
  const uint8_t torque_off[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x00,0xCF};

  if (on) write_array(torque_on, sizeof(torque_on));
  else    write_array(torque_off, sizeof(torque_off));

  flush();
  torque_on_ = on;
}

// ------------------ STOP ------------------
void St3215Servo::stop() {
  const uint8_t stop_cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0x00,0x00,0x92};
  write_array(stop_cmd, sizeof(stop_cmd));
  flush();
}

// ------------------ ROTATE ------------------
void St3215Servo::rotate(bool cw) {
  if (cw) {
    const uint8_t cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0x2C,0x01,0x65};
    write_array(cmd, sizeof(cmd));
  } else {
    const uint8_t cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0xD4,0xFE,0xC0};
    write_array(cmd, sizeof(cmd));
  }
  flush();
}

// ------------------ LIMITS ------------------
void St3215Servo::set_zero() {
  zero_offset_ = turns_unwrapped_;
  has_zero_ = true;
  ESP_LOGI(TAG, "ZERO set");
}

void St3215Servo::set_max() {
  if (!has_zero_) return;
  max_turns_ = turns_unwrapped_ - zero_offset_;
  has_max_ = true;
  ESP_LOGI(TAG, "MAX set");
}

// ------------------ SWITCH ------------------
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  s->set_parent(this);
  s->publish_state(torque_on_);
}

}  // namespace st3215_servo
}  // namespace esphome
