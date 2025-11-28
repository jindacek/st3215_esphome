#include "st3215_servo.h"
#include "esphome/core/log.h"

namespace esphome {
namespace st3215_servo {

static const char *TAG = "st3215";

void St3215Servo::update_calib_state_(CalibState s) {
  calib_state_ = s;
  if (calib_state_sensor_) {
    calib_state_sensor_->publish_state((float) s);
  }
}

void St3215Servo::setup() {
  if (!has_zero_ || max_turns_ <= 0) {
    calibrated_ = false;
    update_calib_state_(CALIB_IDLE);
    ESP_LOGI(TAG, "Nutná kalibrace rolety");
  } else {
    calibrated_ = true;
    update_calib_state_(CALIB_READY);
    ESP_LOGI(TAG, "Roleta připravena, max_turns=%.2f", max_turns_);
  }
}

void St3215Servo::loop() {
  if (!moving_) return;
}

void St3215Servo::rotate(bool cw, int speed) {
  moving_ = true;
  direction_ = cw;
  speed_ = speed;
  ESP_LOGD(TAG, "Rotate %s speed %d", cw ? "CW" : "CCW", speed);
}

void St3215Servo::stop() {
  moving_ = false;
  ESP_LOGD(TAG, "STOP");
}

void St3215Servo::set_zero() {
  zero_turns_ = current_turns_;
  has_zero_ = true;
  ESP_LOGI(TAG, "ZERO uložen");
}

void St3215Servo::set_max() {
  max_turns_ = current_turns_ - zero_turns_;
  if (max_turns_ > 0.1f) {
    has_max_ = true;
    ESP_LOGI(TAG, "MAX uložen: %.2f otáček", max_turns_);
  } else {
    has_max_ = false;
    ESP_LOGW(TAG, "MAX poloha neplatná");
  }
}

void St3215Servo::start_calibration() {
  calibration_active_ = true;
  has_zero_ = false;
  has_max_ = false;
  calibrated_ = false;
  update_calib_state_(CALIB_WAIT_ZERO);
  ESP_LOGI(TAG, "Kalibrace zahájena – najeď na SPODNÍ polohu a potvrď");
}

void St3215Servo::confirm_calibration_step() {
  if (!calibration_active_) return;

  switch (calib_state_) {
    case CALIB_WAIT_ZERO:
      set_zero();
      update_calib_state_(CALIB_WAIT_MAX);
      ESP_LOGI(TAG, "ZERO OK – najeď na HORNÍ polohu a potvrď");
      break;

    case CALIB_WAIT_MAX:
      set_max();
      if (has_max_) {
        calibration_active_ = false;
        calibrated_ = true;
        update_calib_state_(CALIB_READY);
        ESP_LOGI(TAG, "Kalibrace dokončena");
      } else {
        calibration_active_ = false;
        update_calib_state_(CALIB_ERROR);
        ESP_LOGW(TAG, "Kalibrace selhala");
      }
      break;

    default:
      break;
  }
}

void St3215Servo::publish_angle_(float angle) {
  if (angle_sensor_) angle_sensor_->publish_state(angle);
}

void St3215Servo::publish_turns_(float turns) {
  if (turns_sensor_) turns_sensor_->publish_state(turns);
}

void St3215Servo::publish_percent_(float pct) {
  if (percent_sensor_) percent_sensor_->publish_state(pct);
}

void St3215TorqueSwitch::write_state(bool state) {
  if (!parent_) return;

  // zatím jen log, dokud nebude implementovaný skutečný torque protokol
  ESP_LOGI("st3215", "Torque switch: %s", state ? "ON" : "OFF");

  publish_state(state);
}

}  // namespace st3215_servo
}  // namespace esphome
