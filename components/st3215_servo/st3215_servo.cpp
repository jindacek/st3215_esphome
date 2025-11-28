#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ================= TORQUE SWITCH =================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque(state);
  publish_state(state);
}

// ================= CHECKSUM =================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  // sčítá se od ID (index 2) po poslední byte před checksumem
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// ================= SEND PACKET =================
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

  write_array(p);
  flush();
}

// ================= READ REGISTERS =================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(8 + len);

  while (millis() - start < 40) {
    while (available()) buf.push_back(read());

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
  ESP_LOGW(TAG, "read_registers_ timeout (addr=0x%02X len=%u)", addr, len);
  return false;
}

// ================= SETUP =================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 setup ID=%u", servo_id_);

  // MOTOR mode
  const uint8_t motor_mode[] = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x21, 0x01, 0xD5};
  write_array(motor_mode, sizeof(motor_mode));
  flush();
  delay(10);

  // Torque ON při startu
  const uint8_t torque_on[] = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
  write_array(torque_on, sizeof(torque_on));
  flush();

  torque_on_ = true;
}

// ================= CONFIG =================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo");
  ESP_LOGCONFIG(TAG, "  ID: %u", servo_id_);
  ESP_LOGCONFIG(TAG, "  Max angle: %.1f deg", max_angle_);
  ESP_LOGCONFIG(TAG, "  Turns full open (legacy): %.3f", turns_full_open_);
}

// ================= UPDATE (soft multi-turn + SW koncáky) =================
void St3215Servo::update() {
  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) return;

  uint16_t raw = pos[0] | (pos[1] << 8);

  // soft multi-turn z 0x38
  if (!have_last_) {
    last_raw_ = raw;
    have_last_ = true;
  } else {
    int diff = (int)raw - (int)last_raw_;
    if (diff > 2048)       // přechod 4095 -> 0
      turns_unwrapped_ -= 1.0f;
    else if (diff < -2048) // přechod 0 -> 4095
      turns_unwrapped_ += 1.0f;
    last_raw_ = raw;
  }

  // aktualizace zlomkové části dle aktuálního RAW
  float frac = raw / RAW_PER_TURN;  // 0..1
  turns_unwrapped_ = std::floor(turns_unwrapped_) + frac;

  float angle = frac * 360.0f;
  float total = turns_unwrapped_ - zero_offset_;

  if (angle_sensor_) angle_sensor_->publish_state(angle);
  if (turns_sensor_) turns_sensor_->publish_state(total);

  if (percent_sensor_ && has_max_ && max_turns_ > 0.0f) {
    float pct = (total / max_turns_) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    percent_sensor_->publish_state(pct);
  }

  // Soft koncáky – bezpečnost
  if (has_zero_ && total <= 0.0f) {
    ESP_LOGD(TAG, "Soft limit: ZERO reached -> stop()");
    stop();
  }
  if (has_max_ && total >= max_turns_) {
    ESP_LOGD(TAG, "Soft limit: MAX reached -> stop()");
    stop();
  }
}

// ================= TORQUE =================
void St3215Servo::set_torque(bool on) {
  const uint8_t torque_on[]  = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
  const uint8_t torque_off[] = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x28, 0x00, 0xCF};

  if (on) {
    write_array(torque_on, sizeof(torque_on));
  } else {
    write_array(torque_off, sizeof(torque_off));
  }
  flush();

  torque_on_ = on;
  if (torque_switch_) torque_switch_->publish_state(on);
}

// ================= STOP (speed-mode STOP přes 0x2E) =================
void St3215Servo::stop() {
  // FF FF 01 05 03 2E 00 00 C8 – ověřeno v HTerm
  uint8_t frame[9];
  frame[0] = 0xFF;
  frame[1] = 0xFF;
  frame[2] = servo_id_;
  frame[3] = 0x05;   // LEN
  frame[4] = 0x03;   // WRITE
  frame[5] = 0x2E;   // GOAL_SPEED_L
  frame[6] = 0x00;   // lo
  frame[7] = 0x00;   // hi
  frame[8] = checksum_(frame, 8);

  write_array(frame, sizeof(frame));
  flush();
}

// ================= ROTATE (bez parametru – default) =================
void St3215Servo::rotate(bool cw) {
  // Default rychlost, když se nepředá nic z YAML (např. 600)
  rotate(cw, 600);
}

// ================= ROTATE (s rychlostí) =================
// speed > 0  → CW (po směru)
// speed < 0  → CCW (proti směru)
// My to mapujeme tak, že:
//   cw = true  → použijeme +speed
//   cw = false → použijeme -speed
void St3215Servo::rotate(bool cw, int speed) {
  if (speed < 0) speed = -speed;
  if (speed > 2500) speed = 2500;  // bezpečný limit dle tvých testů

  // Motor mode pro jistotu (kdyby někdo přepnul jinam)
  const uint8_t motor_mode[] = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x21, 0x01, 0xD5};
  write_array(motor_mode, sizeof(motor_mode));
  flush();
  delay(2);

  // Torque ON, kdyby byl vypnutý
  if (!torque_on_) {
    const uint8_t torque_on[] = {0xFF, 0xFF, servo_id_, 0x04, 0x03, 0x28, 0x01, 0xCE};
    write_array(torque_on, sizeof(torque_on));
    flush();
    torque_on_ = true;
    if (torque_switch_) torque_switch_->publish_state(true);
  }

  uint16_t spd = (uint16_t) speed;
  uint8_t lo = spd & 0xFF;
  uint8_t hi = (spd >> 8) & 0x7F;  // 0..127, horní bit je sign

  // CW = kladná rychlost, CCW = záporná
  if (!cw) {
    hi |= 0x80;  // nastavíme sign bit → záporná rychlost
  }

  uint8_t frame[9];
  frame[0] = 0xFF;
  frame[1] = 0xFF;
  frame[2] = servo_id_;
  frame[3] = 0x05;   // LEN
  frame[4] = 0x03;   // WRITE
  frame[5] = 0x2E;   // GOAL_SPEED_L
  frame[6] = lo;
  frame[7] = hi;
  frame[8] = checksum_(frame, 8);

  write_array(frame, sizeof(frame));
  flush();
}

// ================= CALIBRATION =================
void St3215Servo::set_zero() {
  zero_offset_ = turns_unwrapped_;
  has_zero_ = true;
  ESP_LOGI(TAG, "ZERO calibrated at %.3f turns", zero_offset_);
}

void St3215Servo::set_max() {
  if (!has_zero_) {
    ESP_LOGW(TAG, "Cannot set MAX – ZERO not calibrated");
    return;
  }
  max_turns_ = turns_unwrapped_ - zero_offset_;
  has_max_ = true;
  ESP_LOGI(TAG, "MAX calibrated, range=%.3f turns", max_turns_);
}

// ================= SWITCH BINDING =================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  if (torque_switch_) {
    torque_switch_->set_parent(this);
    torque_switch_->publish_state(torque_on_);
  }
}

}  // namespace st3215_servo
}  // namespace esphome
