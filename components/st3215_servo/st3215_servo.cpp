#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ================= Torque Switch =================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque(state);
  this->publish_state(state);
}

// ================= Checksum =====================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// ================= Send frame ===================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(6 + params.size());

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

// ================= READ REGISTERS ================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;

  while (millis() - start < 50) {
    while (this->available()) buf.push_back(this->read());

    if (buf.size() >= 6) {

      // sync FF FF
      size_t i = 0;
      while (i + 1 < buf.size() && !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) i++;
      if (i > 0) buf.erase(buf.begin(), buf.begin() + i);

      if (buf.size() < 6) continue;

      uint8_t rlen = buf[3];
      if (buf.size() < (size_t)(rlen + 4)) continue;

      uint8_t chk = buf[rlen + 3];
      uint8_t calc = checksum_(buf.data(), rlen + 3);
      if (chk != calc) {
        ESP_LOGW(TAG, "Bad checksum");
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

// ================= SETUP ========================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 ready, id=%u", servo_id_);

  // MOTOR MODE only (servo must NOT start moving)
  const uint8_t motor_mode[] = {0xFF,0xFF, servo_id_, 0x04, 0x03, 0x21, 0x01, 0xD5};
  this->write_array(motor_mode, sizeof(motor_mode));
  this->flush();

  torque_on_ = false;
}

// ================= CONFIG =======================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo:");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
}

// ================= UPDATE (READ POS + TURNS) ====
void St3215Servo::update() {

  // POSITION
  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) return;
  uint16_t raw = pos[0] | (pos[1] << 8);

  // WHOLE TURNS
  std::vector<uint8_t> trn;
  int16_t whole = 0;
  if (read_registers_(servo_id_, 0x3A, 2, trn))
    whole = trn[0] | (trn[1] << 8);

  float total = whole + raw / RAW_PER_TURN;
  float deg   = raw * 360.0f / RAW_PER_TURN;

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

// ================= Wiring =======================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (s) {
    s->set_parent(this);
    s->publish_state(torque_on_);
  }
}

// ================= Torque =======================
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
  if (torque_switch_) torque_switch_->publish_state(torque_on_);
}


// ================= STOP =========================
// ================= STOP =========================
void St3215Servo::stop() {
  const uint8_t stop_cmd[] = {
    0xFF, 0xFF,
    servo_id_,
    0x0A,
    0x03,
    0x2A,
    0x32,
    0x00, 0x00,   // pos ignore
    0x03, 0x00,   // turns ignore
    0x00, 0x00,   // SPEED = 0 → STOP
    0x92
  };
  this->write_array(stop_cmd, sizeof(stop_cmd));
  this->flush();
}


// ================= ROTATE =======================
void St3215Servo::rotate(bool cw, int) {

  if (cw) {
    // DOLŮ
    const uint8_t cmd_cw[] = {
      0xFF,0xFF, servo_id_, 0x0A, 0x03, 0x2A,
      0x32, 0x00, 0x00, 0x03, 0x00,
      0x2C, 0x01, 0x65
    };
    this->write_array(cmd_cw, sizeof(cmd_cw));
  } else {
    // NAHORU
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
