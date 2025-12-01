#pragma once

#include "esphome/components/cover/cover.h"
#include "st3215_servo.h"

namespace esphome {
namespace st3215_servo {

class St3215Cover : public cover::Cover, public Component {
 public:
  St3215Cover(St3215Servo *servo, float *open_speed, float *close_speed);

  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;
  void dump_config() override {}

 protected:
  St3215Servo *servo_;

  float *open_speed_;
  float *close_speed_;

  bool moving_{false};
  bool direction_open_{false};
};

}  // namespace st3215_servo
}  // namespace esphome
