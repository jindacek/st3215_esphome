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
  send_packet_(id, 0x02, {addr, len});
  uint32_t start = millis();
  std::vector<uint8_t> buf;

  while (millis() - start < 40) {
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
  if (!read_registers_(servo_id_, 0x38, 2, pos)) return;

  uint16_t raw = pos[0] | (pos[1] << 8);

  if (!have_last_) {
    last_raw_ = raw;
    turns_base_ = 0;
    turns_unwrapped_ = raw / RAW_PER_TURN;
    have_last_ = true;
    return;
  }

    // OCHRANA PROTI NESMYSLNÉ HODNOTĚ RAW
    if (raw > 4095) {
      ESP_LOGW("st3215", "RAW out of range: %u (reset unwrap)", raw);
      have_last_ = false;
      return;
    }
    
    int diff = (int) raw - (int) last_raw_;
    
    // filtrování skoku – ALE dovolíme rekonstrukci po více chybách
    static uint8_t glitch_counter = 0;
    
    if (abs(diff) > 3000) {
      glitch_counter++;
      ESP_LOGW("st3215", "RAW glitch #%u: last=%u now=%u", glitch_counter, last_raw_, raw);
    
      // po 3 chybách přijmeme nový RAW jako realitu
      if (glitch_counter >= 3) {
        ESP_LOGW("st3215", "RAW resync after glitches (accepting new raw)");
        last_raw_ = raw;
        glitch_counter = 0;
      }
      return;
    }
    
    // validní stav → reset počítadla chyb
    glitch_counter = 0;


  if (diff > 2048) turns_base_--;
  else if (diff < -2048) turns_base_++;

  last_raw_ = raw;
  turns_unwrapped_ = turns_base_ + (raw / RAW_PER_TURN);

  float angle = (raw / RAW_PER_TURN) * 360.0f;
  float total = fabsf(turns_unwrapped_ - zero_offset_);

  if (angle_sensor_) angle_sensor_->publish_state(angle);
  if (turns_sensor_) turns_sensor_->publish_state(total);

  if (percent_sensor_ && has_zero_ && has_max_) {
    float percent = 100.0f - (total / max_turns_) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    percent_sensor_->publish_state(percent);
  }

  // SW koncáky – se směrem
  if (moving_) {
    if (has_zero_ && total <= 0.01f && !moving_cw_) {
      ESP_LOGI(TAG, "SW KONCÁK: horní – stop");
      stop();
    }

    if (has_max_ && total >= (max_turns_ - 0.01f) && moving_cw_) {
      ESP_LOGI(TAG, "SW KONCÁK: spodní – stop");
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
  moving_ = false;
  const uint8_t stop_cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0x00,0x00,0x92};
  write_array(stop_cmd, sizeof(stop_cmd));
  ESP_LOGW(TAG, "STOP COMMAND CALLED");
  flush();
}

// ================= ROTATE =================
void St3215Servo::rotate(bool cw, int speed) {
  moving_ = true;
  moving_cw_ = cw;

  if (speed < 0) speed = -speed;
  uint16_t mag = speed;
  if (mag > 3000) mag = 3000;

  uint8_t lo = mag & 0xFF;
  uint8_t hi = (mag >> 8) & 0x7F;
  if (!cw) hi |= 0x80;

  std::vector<uint8_t> p = {0x2E, lo, hi};
  send_packet_(servo_id_, 0x03, p);
}

// ================= CALIBRATION =================
void St3215Servo::start_calibration() {
  calibration_active_ = true;
  has_zero_ = false;
  has_max_ = false;
  zero_offset_ = 0.0f;
  max_turns_ = 0.0f;

  // multiturn NEresetujeme

  update_calib_state_(CALIB_WAIT_TOP);
  ESP_LOGI(TAG, "Kalibrace zahájena – najeď na HORNÍ polohu");
}

// ===== POTVRZENÍ KALIBRACE =====
void St3215Servo::confirm_calibration_step() {

  if (!calibration_active_) {
    ESP_LOGW(TAG, "Kalibrace: potvrzení bez aktivní kalibrace – ignoruji");
    return;
  }

  float current = turns_unwrapped_;

  if (calib_state_ == CALIB_WAIT_TOP) {
    zero_offset_ = current;
    has_zero_ = true;

    if (turns_sensor_) turns_sensor_->publish_state(0.0f);
    if (percent_sensor_) percent_sensor_->publish_state(100.0f);

    ESP_LOGI(TAG, "Kalibrace TOP: zero_offset = %.3f", zero_offset_);

    update_calib_state_(CALIB_WAIT_BOTTOM);
    ESP_LOGI(TAG, "Kalibrace: najeď na SPODNÍ polohu a potvrď");
    return;
  }

  if (calib_state_ == CALIB_WAIT_BOTTOM) {

    if (!has_zero_) {
      calibration_active_ = false;
      update_calib_state_(CALIB_ERROR);
      return;
    }

    float span = fabsf(current - zero_offset_);

    if (span < 0.3f) {
      calibration_active_ = false;
      update_calib_state_(CALIB_ERROR);
      return;
    }

    max_turns_ = span;
    has_max_ = true;

    if (turns_sensor_) turns_sensor_->publish_state(max_turns_);
    if (percent_sensor_) percent_sensor_->publish_state(0.0f);

    calibration_active_ = false;
    update_calib_state_(CALIB_DONE);

    ESP_LOGI(TAG, "Kalibrace BOTTOM: max_turns = %.3f", max_turns_);
    return;
  }

  calibration_active_ = false;
  update_calib_state_(CALIB_ERROR);
}

void St3215Servo::set_zero() {
  zero_offset_ = turns_unwrapped_;
  has_zero_ = true;
  ESP_LOGI(TAG, "ZERO SET (%.3f)", zero_offset_);
}

void St3215Servo::set_max() {
  if (!has_zero_) return;
  float span = fabsf(turns_unwrapped_ - zero_offset_);
  if (span < 0.3f) return;
  max_turns_ = span;
  has_max_ = true;
  ESP_LOGI(TAG, "MAX SET = %.3f", max_turns_);
}

// ================= SWITCH =================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  torque_switch_->set_parent(this);
  torque_switch_->publish_state(torque_on_);
}

}  // namespace st3215_servo
}  // namespace esphome
