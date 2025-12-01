#include "st3215_cover.h"
#include "esphome/core/log.h"

namespace esphome {
namespace st3215_servo {

static const char *TAG = "st3215_cover";

St3215Cover::St3215Cover(St3215Servo *servo, float *open_speed, float *close_speed)
  : servo_(servo), open_speed_(open_speed), close_speed_(close_speed) {}

// COVER vlastnosti
cover::CoverTraits St3215Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  return traits;
}

// Povely z Home Assistant
void St3215Cover::control(const cover::CoverCall &call) {

  // STOP
  if (call.get_stop()) {
    ESP_LOGI(TAG, "STOP");
    servo_->stop();
    moving_ = false;
    return;
  }

  // POZICE %
  if (call.get_position().has_value()) {
    float target = *call.get_position();  // 0.0–1.0
    float percent = target * 100.0f;

    // aktuální pozice
    float current = this->position.value_or(0.0f) * 100.0f;

    float diff = percent - current;

    if (fabsf(diff) < 1.0f) {
      ESP_LOGI(TAG, "Pozice už sedí: %.1f %%", percent);
      return;
    }

    bool open = diff > 0;

    int speed = open
      ? (int)*open_speed_
      : (int)*close_speed_;

    ESP_LOGI(TAG, "Goto %.1f %% speed=%d", percent, speed);
    servo_->rotate(open, speed);
    moving_ = true;
    direction_open_ = open;
    return;
  }

  // OPEN
  if (call.get_open()) {
    ESP_LOGI(TAG, "OPEN");
    servo_->rotate(false, (int)*open_speed_);
    moving_ = true;
    direction_open_ = true;
    return;
  }

  // CLOSE
  if (call.get_close()) {
    ESP_LOGI(TAG, "CLOSE");
    servo_->rotate(true, (int)*close_speed_);
    moving_ = true;
    direction_open_ = false;
    return;
  }
}

}  // namespace st3215_servo
}  // namespace esphome
