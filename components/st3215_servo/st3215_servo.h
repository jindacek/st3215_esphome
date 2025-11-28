#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/automation.h"
#include <vector>

namespace esphome {
namespace st3215_servo {

static const float RAW_PER_TURN = 4096.0f;

class St3215Servo;

// ===================== Torque Switch =====================
class St3215TorqueSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void write_state(bool state) override;

 protected:
  St3215Servo *parent_{nullptr};
};

// ===================== Servo =====================
class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  // default ctor kvůli auto-generated main.cpp
  St3215Servo()
      : PollingComponent(500), uart::UARTDevice(nullptr) {}

  // preferovaný ctor s UART parentem a ID
  St3215Servo(uart::UARTComponent *parent, uint8_t id)
      : PollingComponent(500), uart::UARTDevice(parent), servo_id_(id) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // nastavení z main.cpp / YAML
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_max_angle(float a) { max_angle_ = a; }
  void set_turns_full_open(float t) { turns_full_open_ = t; }

  // pohyb
  void rotate(bool cw);              // default speed (pro jednoduché použití)
  void rotate(bool cw, int speed);   // plná kontrola rychlosti (slider v HA)
  void stop();

  // torque
  void set_torque(bool on);
  void set_torque_switch(St3215TorqueSwitch *s);

  // kalibrace
  void set_zero();
  void set_max();

  // senzory
  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }

 protected:
  uint8_t servo_id_{1};
  float max_angle_{240};
  float turns_full_open_{0};

  bool torque_on_{true};

  uint16_t last_raw_{0};
  bool have_last_{false};
  float turns_unwrapped_{0};

  // kalibrace soft koncáků
  float zero_offset_{0};
  float max_turns_{0};
  bool has_zero_{false};
  bool has_max_{false};

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};

  St3215TorqueSwitch *torque_switch_{nullptr};

  // interní helpery
  uint8_t checksum_(const uint8_t *data, size_t len);
  void send_packet_(uint8_t id, uint8_t cmd, const std::vector<uint8_t> &params);
  bool read_registers_(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> &out);
};


// ===================== Rotate Action =====================
// Používá se v YAML jako:
//  - st3215_servo.rotate:
//      id: roleta_servo
//      cw: true/false
//      speed: !lambda 'return id(rychlost).state;'
class St3215RotateAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void set_speed(int speed) { speed_ = speed; }
  void set_cw(bool cw) { cw_ = cw; }

  void play() override {
    if (parent_ == nullptr) return;
    parent_->rotate(cw_, speed_);
  }

 protected:
  St3215Servo *parent_{nullptr};
  int speed_{600};
  bool cw_{true};
};

// ===================== Stop Action =====================
class St3215StopAction : public Action<> {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }

  void play() override {
    if (parent_ == nullptr) return;
    parent_->stop();
  }

 protected:
  St3215Servo *parent_{nullptr};
};

}  // namespace st3215_servo
}  // namespace esphome
