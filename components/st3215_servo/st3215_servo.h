#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include <vector>

namespace esphome {
namespace st3215_servo {

static const uint8_t INST_PING  = 0x01;
static const uint8_t INST_READ  = 0x02;
static const uint8_t INST_WRITE = 0x03;

static const uint8_t ADDR_TORQUE_ENABLE      = 0x28;
static const uint8_t ADDR_TORQUE_LIMIT       = 0x29;
static const uint8_t ADDR_GOAL_POSITION_L    = 0x2A;
static const uint8_t ADDR_GOAL_TIME_L        = 0x2C;
static const uint8_t ADDR_GOAL_SPEED_L       = 0x2E;
static const uint8_t ADDR_PRESENT_POSITION_L = 0x38;

class St3215Servo;  // forward declaration

// Torque switch entity
class St3215TorqueSwitch : public switch_::Switch, public Component {
 public:
  void write_state(bool state) override;
  void set_parent(St3215Servo *parent) { parent_ = parent; }

 protected:
  St3215Servo *parent_{nullptr};
};

class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_max_angle(float deg) { max_angle_ = deg; }
  void set_turns_full_open(float turns) { turns_full_open_ = turns; }

  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }

  void set_torque_switch(St3215TorqueSwitch *s);

  void setup() override;
  void update() override;
  void dump_config() override;

  void rotate(bool cw, int speed);
  void stop();
  void set_angle(float degrees, int speed = 600);
  void move_to_turns(float turns, int speed = 600);
  void move_to_percent(float percent, int speed = 600);
  void set_torque(bool on);

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

  St3215TorqueSwitch *torque_switch_{nullptr};
  bool torque_on_{true};

  float current_angle_{0.0f};
  float current_turns_{0.0f};
  float last_angle_{0.0f};
  bool has_last_angle_{false};
};

// ---------------- Actions (ESPHome 2025 compatible) ----------------
class St3215RotateAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void set_cw(bool cw) { cw_ = cw; }
  void set_speed(int speed) { speed_ = speed; }
  void play() override { if (parent_) parent_->rotate(cw_, speed_); }
 protected:
  St3215Servo *parent_{nullptr};
  bool cw_{true};
  int speed_{600};
};

class St3215StopAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void play() override { if (parent_) parent_->stop(); }
 protected:
  St3215Servo *parent_{nullptr};
};

class St3215SetAngleAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void set_angle(float a) { angle_ = a; }
  void set_speed(int s) { speed_ = s; }
  void play() override { if (parent_) parent_->set_angle(angle_, speed_); }
 protected:
  St3215Servo *parent_{nullptr};
  float angle_{0};
  int speed_{600};
};

class St3215MoveToTurnsAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void set_turns(float t) { turns_ = t; }
  void set_speed(int s) { speed_ = s; }
  void play() override { if (parent_) parent_->move_to_turns(turns_, speed_); }
 protected:
  St3215Servo *parent_{nullptr};
  float turns_{0};
  int speed_{600};
};

class St3215MoveToPercentAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void set_percent(float pr) { percent_ = pr; }
  void set_speed(int s) { speed_ = s; }
  void play() override { if (parent_) parent_->move_to_percent(percent_, speed_); }
 protected:
  St3215Servo *parent_{nullptr};
  float percent_{0};
  int speed_{600};
};

}  // namespace st3215_servo
}  // namespace esphome
