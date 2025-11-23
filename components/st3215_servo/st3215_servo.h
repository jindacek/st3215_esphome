#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace st3215_servo {

static const uint8_t INST_PING  = 0x01;
static const uint8_t INST_READ  = 0x02;
static const uint8_t INST_WRITE = 0x03;

static const uint8_t ADDR_GOAL_POSITION_L    = 0x2A;
static const uint8_t ADDR_GOAL_TIME_L        = 0x2C;
static const uint8_t ADDR_GOAL_SPEED_L       = 0x2E;
static const uint8_t ADDR_PRESENT_POSITION_L = 0x38;

class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_max_angle(float deg) { max_angle_ = deg; }
  void set_turns_full_open(float turns) { turns_full_open_ = turns; }

  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }

  void setup() override;
  void update() override;

  void rotate(bool cw, int speed);
  void stop();
  void set_angle(float degrees, int speed = 600);
  void move_to_turns(float turns, int speed = 600);
  void move_to_percent(float percent, int speed = 600);

 protected:
  void send_write_(uint8_t addr, const std::vector<uint8_t> &data);
  bool send_read_(uint8_t addr, uint8_t len, std::vector<uint8_t> &out);
  void send_packet_(uint8_t inst, const std::vector<uint8_t> &params);
  bool read_response_(std::vector<uint8_t> &out);
  uint8_t checksum_(uint8_t id, uint8_t len, uint8_t inst, const std::vector<uint8_t> &params);

  void update_turn_counter_(float new_angle);

  uint8_t servo_id_{1};
  float max_angle_{240.0f};
  float turns_full_open_{0.0f};

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};

  float current_angle_{0.0f};
  float current_turns_{0.0f};
  float last_angle_{0.0f};
  bool has_last_angle_{false};
};


// ---------------- Actions (ESPHome 2025 compatible) ----------------

template<typename... Ts>
class St3215SetAngleAction : public Action<Ts...> {
 public:
  explicit St3215SetAngleAction(St3215Servo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, angle)
  TEMPLATABLE_VALUE(int, speed)

  void play(Ts... x) override {
    parent_->set_angle(angle_.value(x...), speed_.value(x...));
  }

 protected:
  St3215Servo *parent_;
};

template<typename... Ts>
class St3215RotateAction : public Action<Ts...> {
 public:
  explicit St3215RotateAction(St3215Servo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(int, speed)

  void set_direction(bool cw) { cw_ = cw; }

  void play(Ts... x) override {
    parent_->rotate(cw_, speed_.value(x...));
  }

 protected:
  St3215Servo *parent_;
  bool cw_{true};
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
class St3215MoveToTurnsAction : public Action<Ts...> {
 public:
  explicit St3215MoveToTurnsAction(St3215Servo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, turns)
  TEMPLATABLE_VALUE(int, speed)

  void play(Ts... x) override {
    parent_->move_to_turns(turns_.value(x...), speed_.value(x...));
  }

 protected:
  St3215Servo *parent_;
};

template<typename... Ts>
class St3215MoveToPercentAction : public Action<Ts...> {
 public:
  explicit St3215MoveToPercentAction(St3215Servo *parent) : parent_(parent) {}
  TEMPLATABLE_VALUE(float, percent)
  TEMPLATABLE_VALUE(int, speed)

  void play(Ts... x) override {
    parent_->move_to_percent(percent_.value(x...), speed_.value(x...));
  }

 protected:
  St3215Servo *parent_;
};

}  // namespace st3215_servo
}  // namespace esphome

