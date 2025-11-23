#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace st3215_servo {

class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_max_angle(float deg) { max_angle_ = deg; }
  void set_turns_full_open(float turns) { turns_full_open_ = turns; }

  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }
  void set_torque_switch(switch_::Switch *s) { torque_switch_ = s; }

  void setup() override;
  void update() override;
  void dump_config() override;

  // Commands
  void rotate(bool cw, int speed);
  void stop();
  void move_relative(float turns_delta, int speed);
  void set_angle(float angle_deg, int speed);
  void move_to_percent(float percent, int speed);
  void set_torque(bool on);

 protected:
  // protocol helpers
  uint8_t checksum_(const uint8_t *data, size_t len);
  void send_packet_(uint8_t id, uint8_t cmd, const std::vector<uint8_t> &params);
  bool read_registers_(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> &out);

  uint8_t servo_id_{1};
  float max_angle_{240.0f};
  float turns_full_open_{0.0f};

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};
  switch_::Switch *torque_switch_{nullptr};

  // state
  uint16_t last_raw_pos_{0};
  float turns_unwrapped_{0.0f};
  bool have_last_{false};
  bool torque_on_{true};

  static constexpr float RAW_PER_TURN = 4096.0f;  // confirmed by your readings
  static constexpr float CW_CCW_STEP_TURNS = 10.5f;  // per click default
  static constexpr int DEFAULT_ACC = 50;
};

template<typename... Ts>
class St3215RotateAction : public Action<Ts...> {
 public:
  explicit St3215RotateAction(St3215Servo *parent) : parent_(parent) {}
  void set_cw(bool cw) { cw_ = cw; }
  void set_speed(int speed) { speed_ = speed; }
  void play(Ts... x) override { parent_->rotate(cw_, speed_); }
 protected:
  St3215Servo *parent_;
  bool cw_{true};
  int speed_{600};
};

template<typename... Ts>
class St3215StopAction : public Action<Ts...> {
 public:
  explicit St3215StopAction(St3215Servo *parent) : parent_(parent) {}
  void play(Ts... x) override { parent_->stop(); }
 protected:
  St3215Servo *parent_;
};

template<typename... Ts>
class St3215MoveRelativeAction : public Action<Ts...> {
 public:
  explicit St3215MoveRelativeAction(St3215Servo *parent) : parent_(parent) {}
  void set_turns(float t) { turns_ = t; }
  void set_speed(int s) { speed_ = s; }
  void play(Ts... x) override { parent_->move_relative(turns_, speed_); }
 protected:
  St3215Servo *parent_;
  float turns_{0};
  int speed_{600};
};

template<typename... Ts>
class St3215SetAngleAction : public Action<Ts...> {
 public:
  explicit St3215SetAngleAction(St3215Servo *parent) : parent_(parent) {}
  void set_angle(float a) { angle_ = a; }
  void set_speed(int s) { speed_ = s; }
  void play(Ts... x) override { parent_->set_angle(angle_, speed_); }
 protected:
  St3215Servo *parent_;
  float angle_{0};
  int speed_{600};
};

template<typename... Ts>
class St3215MoveToPercentAction : public Action<Ts...> {
 public:
  explicit St3215MoveToPercentAction(St3215Servo *parent) : parent_(parent) {}
  void set_percent(float p) { percent_ = p; }
  void set_speed(int s) { speed_ = s; }
  void play(Ts... x) override { parent_->move_to_percent(percent_, speed_); }
 protected:
  St3215Servo *parent_;
  float percent_{0};
  int speed_{600};
};

}  // namespace st3215_servo
}  // namespace esphome
