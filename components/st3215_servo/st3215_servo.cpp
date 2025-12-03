#include "st3215_servo.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ===== PARAMETRY RAMPY =====
static constexpr int   SPEED_MAX     = 2500;
static constexpr int   SPEED_MIN     = 100;
static constexpr int   ACCEL_RATE    = 25;
static constexpr uint32_t RAMP_DT_MS = 30;
static constexpr float DECEL_ZONE    = 0.90f;
static constexpr float STOP_EPS      = 0.03f;
static constexpr int POSITION_SPEED = 1300;

// ================= TORQUE SWITCH =================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque(state);
  publish_state(state);
}

// ================= CALIB STATE =================
void St3215Servo::update_calib_state_(CalibState s) {
  calib_state_ = s;
  if (calib_state_sensor_) {
    calib_state_sensor_->publish_state((int) s);
  }
}

// ================= PERSISTENTNÍ ÚLOŽIŠTĚ =================
bool St3215Servo::load_calibration_() {
  const uint32_t base = 0x1000u + static_cast<uint32_t>(servo_id_) * 3u;

  auto pref_zero = global_preferences->make_preference<float>(base + 0);
  auto pref_max  = global_preferences->make_preference<float>(base + 1);
  auto pref_pos  = global_preferences->make_preference<float>(base + 2);

  float z = 0.0f;
  float m = 0.0f;
  float p = 0.0f;

  if (!pref_zero.load(&z) || !pref_max.load(&m) || !pref_pos.load(&p)) {
    ESP_LOGI(TAG, "No stored calibration for servo %u", servo_id_);
    return false;
  }

  if (m < 0.3f) {
    ESP_LOGW(TAG, "Stored calibration invalid (max_turns=%.3f) – ignoring", m);
    return false;
  }

  zero_offset_ = z;
  max_turns_   = m;
  has_zero_    = true;
  has_max_     = true;

  stored_turns_ = p;
  has_stored_turns_ = true;

  ESP_LOGI(TAG, "Loaded calibration: zero=%.3f max=%.3f pos=%.3f",
           zero_offset_, max_turns_, stored_turns_);
  return true;
}

void St3215Servo::save_calibration_() {
  if (!has_zero_ || !has_max_) return;

  const uint32_t base = 0x1000u + static_cast<uint32_t>(servo_id_) * 3u;

  global_preferences->make_preference<float>(base + 0).save(&zero_offset_);
  global_preferences->make_preference<float>(base + 1).save(&max_turns_);
  global_preferences->make_preference<float>(base + 2).save(&turns_unwrapped_);

  ESP_LOGI(TAG, "Calibration saved: zero=%.3f max=%.3f pos=%.3f",
           zero_offset_, max_turns_, turns_unwrapped_);
}

// ================= CHECKSUM =================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// ================= SEND PACKET =================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(6 + params.size());

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

// ================= READ REGISTERS =================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(32);

  while (millis() - start < 150) {

    while (available()) {
      buf.push_back(read());
      if (buf.size() > 64)
        buf.erase(buf.begin(), buf.begin() + (buf.size() - 64));
    }

    while (buf.size() >= 2 && !(buf[0] == 0xFF && buf[1] == 0xFF))
      buf.erase(buf.begin());

    if (buf.size() < 6) continue;

    uint8_t rid = buf[2];
    uint8_t rlen = buf[3];
    size_t full_len = rlen + 4;

    if (buf.size() < full_len) continue;

    if (rid != id || buf[4] != 0) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    uint8_t chk = buf[full_len - 1];
    if (chk != checksum_(buf.data(), full_len - 1)) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    out.assign(buf.begin() + 5, buf.begin() + 5 + len);
    return true;
  }
  return false;
}

// ================= SETUP =================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 init ID=%u", servo_id_);

  // Motor mode
  send_packet_(servo_id_, 0x03, {0x21, 0x01});

  // Torque ON
  send_packet_(servo_id_, 0x03, {0x28, 0x01});
  torque_on_ = true;

  bool loaded = load_calibration_();
  if (!has_zero_ || !has_max_) {
    update_calib_state_(CALIB_IDLE);
    ESP_LOGI(TAG, loaded ? "Stored calibration incomplete" : "Nutná kalibrace");
  } else {
    update_calib_state_(CALIB_DONE);
    ESP_LOGI(TAG, "Ready (max_turns=%.2f)", max_turns_);
  }
}

// ================= UPDATE =================
void St3215Servo::update() {
  uint32_t &last_uart_recovery = last_uart_recovery_;

  if (encoder_fault_) {
    uint32_t now = millis();
    if (now - last_uart_recovery >= 5000) {
      ESP_LOGW(TAG, "UART recovery...");
      flush();
      delay(10);
      while (available()) read();
      encoder_fail_count_ = 0;
      encoder_fault_ = false;
      last_uart_recovery = now;
    } else return;
  }

  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) {
    encoder_fail_count_++;
    if (encoder_fail_count_ >= ENCODER_FAIL_LIMIT && !encoder_fault_) {
      ESP_LOGE(TAG, "ENCODER FAULT → STOP");
      stop();
      encoder_fault_ = true;
    }
    return;
  }

  encoder_fail_count_ = 0;

  uint16_t raw = pos[0] | (pos[1] << 8);
  if (!have_last_) {
    last_raw_ = raw;
    turns_unwrapped_ = raw / RAW_PER_TURN;
    have_last_ = true;
    return;
  }

  int diff = (int) raw - (int) last_raw_;
  if (abs(diff) > 2048) {
    if (diff > 0) turns_base_--;
    else turns_base_++;
  }

  last_raw_ = raw;
  turns_unwrapped_ = turns_base_ + (raw / RAW_PER_TURN);

  float angle = (raw / RAW_PER_TURN) * 360.0f;
  float total = fabsf(turns_unwrapped_ - zero_offset_);

  if (angle_sensor_) angle_sensor_->publish_state(angle);
  if (turns_sensor_) turns_sensor_->publish_state(total);

  if (percent_sensor_ && has_zero_ && has_max_) {
    float pct = 100.0f - (total / max_turns_) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    percent_sensor_->publish_state(pct);

    if (position_mode_ && fabs(pct - target_percent_) < 1.5f) {
      ESP_LOGI(TAG, "TARGET %.1f %%", pct);
      stop();
      position_mode_ = false;
    }
  }
}

// ================= STOP =================
void St3215Servo::stop() {
  target_speed_ = 0;
  current_speed_ = 0;
  position_mode_ = false;

  std::vector<uint8_t> p = {
    0x2A, 0x32,
    0x00, 0x00,
    0x03,
    0x00, 0x00, 0x00
  };

  send_packet_(servo_id_, 0x03, p);

  moving_ = false;
  if (open_switch_) open_switch_->publish_state(false);
  if (close_switch_) close_switch_->publish_state(false);

  if (has_zero_ && has_max_) save_calibration_();
}

// ================= ROTATE =================
void St3215Servo::rotate(bool cw, int speed) {
  moving_ = true;
  moving_cw_ = cw;
  if (speed < 0) speed = -speed;
  if (speed > SPEED_MAX) speed = SPEED_MAX;
  target_speed_ = speed;
}

// ================= MOVE PERCENT =================
void St3215Servo::move_to_percent(float pct) {
  if (!has_zero_ || !has_max_) return;
  target_percent_ = pct;
  position_mode_ = true;

  float current = percent_sensor_ ? percent_sensor_->state : 0;
  if (fabs(current - pct) < 1.0f) { stop(); return; }

  rotate(pct < current, POSITION_SPEED);
}

// ================= TORQUE =================
void St3215Servo::set_torque(bool on) {
  send_packet_(servo_id_, 0x03, {0x28, (uint8_t)(on ? 1 : 0)});
  torque_on_ = on;
  if (torque_switch_) torque_switch_->publish_state(on);
}

// ================= CALIBRATION =================
void St3215Servo::start_calibration() {
  calibration_active_ = true;
  has_zero_ = false;
  has_max_ = false;
  update_calib_state_(CALIB_WAIT_TOP);
}

void St3215Servo::confirm_calibration_step() {
  if (!calibration_active_) return;

  float current = turns_unwrapped_;
  if (calib_state_ == CALIB_WAIT_TOP) {
    zero_offset_ = current;
    has_zero_ = true;
    update_calib_state_(CALIB_WAIT_BOTTOM);
    return;
  }

  if (calib_state_ == CALIB_WAIT_BOTTOM) {
    max_turns_ = fabsf(current - zero_offset_);
    has_max_ = true;
    calibration_active_ = false;
    update_calib_state_(CALIB_DONE);
    save_calibration_();
  }
}

// ================= SWITCH =================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  torque_switch_->set_parent(this);
  torque_switch_->publish_state(torque_on_);
}

}  // namespace st3215_servo
}  // namespace esphome
