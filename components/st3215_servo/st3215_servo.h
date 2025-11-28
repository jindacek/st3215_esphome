#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include <vector>

namespace esphome {
namespace st3215_servo {

static const float RAW_PER_TURN = 4096.0f;

enum CalibState {
  CALIB_IDLE = 0,
  CALIB_READY = 1,
  CALIB_WAIT_ZERO = 2,
  CALIB_WAIT_MAX = 3,
  CALIB_ERROR = 4
};

class St3215TorqueSwitch : public Component, public switch_::Switch {
 public:
  void set_parent(class St3215Servo *parent) { parent_ = parent; }

 protected:
  void write_state(bool state) override;

  class St3215Servo *parent_{nullptr};
};

class St3215Servo : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;

  void rotate(bool cw, int speed);
  void stop();

  void set_zero();
  void set_max();

  void start_calibration();
  void confirm_calibration_step();

  void set_servo_id(int id) { servo_id_ = id; }
  void set_max_angle(float a) { max_angle_ = a; }
  void set_turns_full_open(float t) { max_turns_ = t; }

  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }

  void set_calib_state_sensor(sensor::Sensor *s) { calib_state_sensor_ = s; }

  void set_torque_switch(switch_::Switch *s) { torque_switch_ = s; }

 protected:
  void publish_angle_(float angle);
  void publish_turns_(float turns);
  void publish_percent_(float pct);

  void update_calib_state_(CalibState s);

  int servo_id_{1};
  float max_angle_{240.0f};
  float max_turns_{0};

  bool moving_{false};
  bool direction_{true};
  int speed_{600};

  float current_turns_{0};
  float zero_turns_{0};

  bool has_zero_{false};
  bool has_max_{false};
  bool calibrated_{false};

  bool calibration_active_{false};
  CalibState calib_state_{CALIB_IDLE};

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};
  sensor::Sensor *calib_state_sensor_{nullptr};

  switch_::Switch *torque_switch_{nullptr};
};

}  // namespace st3215_servo
}  // namespace esphome
