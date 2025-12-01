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

  // necháme to jako "assumed state" – stav si drží HA
  traits.set_is_assumed_state(true);

  // zatím jen OPEN/CLOSE/STOP – bez slideru polohy
  traits.set_supports_position(false);
  traits.set_supports_stop(true);

  return traits;
}

// Povely z Home Assistant
void St3215Cover::control(const cover::CoverCall &call) {

  if (servo_ == nullptr) {
    ESP_LOGW(TAG, "Servo pointer is null");
    return;
  }

  // STOP
  if (call.get_stop()) {
    ESP_LOGI(TAG, "STOP");
    servo_->stop();
    moving_ = false;
    return;
  }

  // OPEN
  if (call.get_open()) {
    int speed = 1000;
    if (open_speed_ != nullptr && open_speed_->has_state())
      speed = static_cast<int>(open_speed_->state);

    ESP_LOGI(TAG, "OPEN (speed=%d)", speed);

    // podle tvého předchozího nastavení:
    // "Roleta DOLŮ (CW – držet ON)" používala rotate(true, open_speed)
    // → open = CW = true
    servo_->rotate(true, speed);
    moving_ = true;
    direction_open_ = true;
    return;
  }

  // CLOSE
  if (call.get_close()) {
    int speed = 1000;
    if (close_speed_ != nullptr && close_speed_->has_state())
      speed = static_cast<int>(close_speed_->state);

    ESP_LOGI(TAG, "CLOSE (speed=%d)", speed);

    // "Roleta NAHORU (CCW – držet ON)" používala rotate(false, close_speed)
    // → close = CCW = false
    servo_->rotate(false, speed);
    moving_ = true;
    direction_open_ = false;
    return;
  }

  // POZICE – zatím neřešíme (nemáme podporu slideru)
  if (call.get_position().has_value()) {
    ESP_LOGI(TAG, "Position command received, but position is not supported yet");
    return;
  }
}

}  // namespace st3215_servo
}  // namespace esphome
