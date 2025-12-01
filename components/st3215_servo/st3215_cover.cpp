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
  traits.set_supports_position(true);   // slider povolen
  traits.set_supports_stop(true);
  return traits;
}

// Povely z Home Assistant (ESPHome 2025.10 kompatibilní)
void St3215Cover::control(const cover::CoverCall &call) {

  if (!servo_) return;

  // STOP → žádná požadovaná pozice
  if (!call.get_position().has_value()) {
    ESP_LOGI(TAG, "STOP");
    servo_->stop();
    return;
  }

  // CÍLOVÁ POZICE (0.0 – 1.0)
  float target = *call.get_position();
  float percent = target * 100.0f;

  // Aktuální pozici zatím nečteme – assumed mode
  ESP_LOGI(TAG, "COVER set position %.1f %%", percent);

  // 1.0 = otevřít
  if (target >= 0.99f) {

    int speed = 1000;
    if (open_speed_ && open_speed_->has_state())
      speed = static_cast<int>(open_speed_->state);

    servo_->rotate(true, speed);
    return;
  }

  // 0.0 = zavřít
  if (target <= 0.01f) {

    int speed = 1000;
    if (close_speed_ && close_speed_->has_state())
      speed = static_cast<int>(close_speed_->state);

    servo_->rotate(false, speed);
    return;
  }

  // mezi – jen log, později můžeme dodělat přesné dojetí
  ESP_LOGI(TAG, "Middle position requested – not implemented yet");
}

}  // namespace st3215_servo
}  // namespace esphome
