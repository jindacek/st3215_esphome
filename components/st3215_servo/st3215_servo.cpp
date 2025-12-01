#include "st3215_servo.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";

// ===== PARAMETRY RAMPY =====
static constexpr int   SPEED_MAX     = 2500;
static constexpr int   SPEED_MIN     = 100;
static constexpr int   ACCEL_RATE    = 30;     // změna rychlosti za krok
static constexpr uint32_t RAMP_DT_MS = 30;     // perioda rampy
static constexpr float DECEL_ZONE    = 0.40f;  // kdy začít brzdit (otáčky před koncem)
static constexpr float STOP_EPS      = 0.05f;  // hystereze koncáku

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
  while (available()) (void) read();
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(32);

  while (millis() - start < 50) {

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

    if (rlen < 2 || rlen > 20) {
      buf.erase(buf.begin());
      continue;
    }

    size_t full_len = rlen + 4;
    if (buf.size() < full_len) continue;

    uint8_t err = buf[4];
    if (rid != id || err != 0) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    uint8_t chk = buf[full_len - 1];
    uint8_t calc = checksum_(buf.data(), full_len - 1);
    if (chk != calc) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    uint8_t data_len = rlen - 2;
    if (data_len < len) {
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

  const uint8_t motor_mode[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x21,0x01,0xD5};
  write_array(motor_mode, sizeof(motor_mode));
  flush();

  const uint8_t torque_on[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x01,0xCE};
  write_array(torque_on, sizeof(torque_on));
  flush();

  torque_on_ = true;

  // ===== INIT STAVU KALIBRACE =====
  if (!has_zero_ || !has_max_) {
    update_calib_state_(CALIB_IDLE);
    ESP_LOGI(TAG, "Nutná kalibrace rolety");
  } else {
    update_calib_state_(CALIB_DONE);
    ESP_LOGI(TAG, "Roleta připravena (max_turns=%.2f)", max_turns_);
  }
}


// ================= CONFIG =================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo ID=%u", servo_id_);
}

// ================= UPDATE =================
void St3215Servo::update() {
  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos))
    return;

  uint16_t raw = pos[0] | (pos[1] << 8);
  if (raw >= 4096 || raw == 0xFFFF)
    return;

  if (!have_last_) {
    last_raw_ = raw;
    turns_base_ = 0;
    turns_unwrapped_ = raw / RAW_PER_TURN;
    have_last_ = true;
    return;
  }

  int diff = (int) raw - (int) last_raw_;
  if (abs(diff) > 3800)
    return;

  if (diff > 2048)
    turns_base_--;
  else if (diff < -2048)
    turns_base_++;

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
  }

  // ===== RAMP ENGINE =====
  uint32_t now = millis();
  if (now - last_ramp_update_ >= RAMP_DT_MS) {
    last_ramp_update_ = now;

    // statická proměnná pro prediktivní brzdění (pamatuje si předchozí dist)
    static float last_dist = 0.0f;

    // vypočítat vzdálenost ke konci
    float dist = 0.0f;
    if (has_zero_ && has_max_) {
      // dolů (CW) → směrem k max_turns_
      // nahoru (CCW) → směrem k nule
      dist = moving_cw_
          ? fabsf(max_turns_ - total)   // spodní konec
          : total;                      // horní konec
    }

    // rychlost přibližování (kladná, když se blížíme k dorazu)
    float dist_delta = last_dist - dist;
    last_dist = dist;

    // základní cílová rychlost vychází z uživatelem chtěné rychlosti
    int effective = target_speed_;

    if (has_zero_ && has_max_) {
      // 1) Základní brzdění podle vzdálenosti (S-křivka)
      if (dist < DECEL_ZONE) {
        float k = dist / DECEL_ZONE;
        if (k < 0) k = 0;
        if (k > 1) k = 1;

        float smooth = k * k * k;  // hezky měkké brzdění
        effective = SPEED_MIN + (int)((target_speed_ - SPEED_MIN) * smooth);
      }

      // 2) Prediktivní brzdění – když se blížíme moc rychle, uber ještě víc
      // dist_delta ~ kolik otáček za krok rampy ujedeš směrem k dorazu
      float predictive_brake = dist_delta * 2.2f;  // koeficient pro doladění

      if (predictive_brake > 0.01f) {
        // čím rychleji se blížíš, tím víc strhneme rychlost dolů
        effective -= (int)(predictive_brake * 800.0f);
        if (effective < SPEED_MIN)
          effective = SPEED_MIN;
      }
    }

    // Aplikace rampy (akcelerace/decelerace rychlosti)
    if (current_speed_ < effective) {
      current_speed_ += ACCEL_RATE;
      if (current_speed_ > effective) current_speed_ = effective;
    } else if (current_speed_ > effective) {
      current_speed_ -= ACCEL_RATE;
      if (current_speed_ < effective) current_speed_ = effective;
    }

    // Odeslání nové rychlosti do serva
    if (moving_ && current_speed_ >= SPEED_MIN) {
      uint8_t lo = current_speed_ & 0xFF;
      uint8_t hi = (current_speed_ >> 8) & 0x7F;
      if (!moving_cw_) hi |= 0x80;
      std::vector<uint8_t> p = {0x2E, lo, hi};
      send_packet_(servo_id_, 0x03, p);
    }
  }

  // ===== SOFT KONCÁKY =====
  if (moving_) {

    if (has_zero_ && total <= STOP_EPS && !moving_cw_) {
      ESP_LOGI(TAG, "SW KONCÁK: 100 %% – STOP");
      stop();
    }

    if (has_max_ && total >= (max_turns_ - STOP_EPS) && moving_cw_) {
      ESP_LOGI(TAG, "SW KONCÁK: 0 %% – STOP");
      stop();
    }
  }
}

// ================= TORQUE =================
void St3215Servo::set_torque(bool on) {
  const uint8_t torque_on[]  = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x01,0xCE};
  const uint8_t torque_off[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x00,0xCF};
  if (on) write_array(torque_on, sizeof(torque_on));
  else write_array(torque_off, sizeof(torque_off));
  flush();
  torque_on_ = on;
  if (torque_switch_) torque_switch_->publish_state(on);
}

// ================= STOP =================
void St3215Servo::stop() {
  target_speed_ = 0;
  const uint8_t stop_cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0x00,0x00,0x92};
  write_array(stop_cmd, sizeof(stop_cmd));
  flush();
  moving_ = false;
  if (open_switch_)  open_switch_->publish_state(false);
  if (close_switch_) close_switch_->publish_state(false);
}

// ================= ROTATE =================
void St3215Servo::rotate(bool cw, int speed) {
  moving_ = true;
  moving_cw_ = cw;
  if (speed < 0) speed = -speed;
  if (speed > SPEED_MAX) speed = SPEED_MAX;
  target_speed_ = speed;
}

// ================= CALIBRATION =================
void St3215Servo::start_calibration() {
  calibration_active_ = true;
  has_zero_ = false;
  has_max_ = false;
  zero_offset_ = 0.0f;
  max_turns_ = 0.0f;
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
    float span = fabsf(current - zero_offset_);
    if (span < 0.3f) {
      update_calib_state_(CALIB_ERROR);
      return;
    }
    max_turns_ = span;
    has_max_ = true;
    calibration_active_ = false;
    update_calib_state_(CALIB_DONE);
  }
}

void St3215Servo::set_zero() {
  zero_offset_ = turns_unwrapped_;
  has_zero_ = true;
}

void St3215Servo::set_max() {
  if (!has_zero_) return;
  float span = fabsf(turns_unwrapped_ - zero_offset_);
  if (span < 0.3f) return;
  max_turns_ = span;
  has_max_ = true;
}

// ================= SWITCH =================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  torque_switch_->set_parent(this);
  torque_switch_->publish_state(torque_on_);
}

}  // namespace st3215_servo
}  // namespace esphome
