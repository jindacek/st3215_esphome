#include "st3215_cover.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215_servo {

static const char *TAG = "st3215_cover";

St3215Cover::St3215Cover(St3215Servo *servo,
                         number::Number *open_speed,
                         number::Number *close_speed)
  : servo_(servo),
    open_speed_(open_speed),
    close_speed_(close_speed) {}

// COVER vlastnosti
cover::CoverTraits St3215Cover::get_traits() {
  auto traits = cover::CoverTraits();

  traits.set_is_assumed_state(true);
  traits.set_supports_position(false);
  traits.set_supports_stop(true);

  return traits;
}

// Povely z Home Assistant (NOVÉ API)
void St3215Cover::control(const cover::CoverCall &call) {

  if (servo_ == nullptr) {
    ESP_LOGW(TAG, "Servo pointer is null");
    return;
  }

  auto op = call.get_operation();

  // STOP
  if (op == cover::COVER_OPERATION_STOP) {
    ESP_LOGI(TAG, "STOP");
    servo_->stop();
    moving_ = false;
    return;
  }

  // OPEN
  if (op == cover::COVER_OPERATION_OPEN) {
    int speed = 1000;
    if (open_speed_ && open_speed_->has_state())
      speed = static_cast<int>(open_speed_->state);

    ESP_LOGI(TAG, "OPEN (speed=%d)", speed);

    // roleta DOLŮ (CW)
    servo_->rotate(true, speed);
    moving_ = true;
    direction_open_ = true;
    return;
  }

  // CLOSE
  if (op == cover::COVER_OPERATION_CLOSE) {
    int speed = 1000;
    if (close_speed_ && close_speed_->has_state())
      speed = static_cast<int>(close_speed_->state);

    ESP_LOGI(TAG, "CLOSE (speed=%d)", speed);

    // roleta NAHORU (CCW)
    servo_->rotate(false, speed);
    moving_ = true;
    direction_open_ = false;
    return;
  }

  ESP_LOGW(TAG, "Unknown cover operation");
}

}  // namespace st3215_servo
}  // namespace esphome
