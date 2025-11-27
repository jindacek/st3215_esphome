#pragma once
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/button/button.h"
#include "esphome/core/helpers.h"
#include <vector>

namespace esphome {
namespace st3215_servo {

static const float RAW_PER_TURN = 4096.0f;
static const uint8_t DEFAULT_ACC = 0x32;

class St3215Servo;

class St3215TorqueSwitch : public switch_::Switch {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void write_state(bool state) override;

 protected:
  St3215Servo *parent_{nullptr};
};

class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  St3215Servo(uart::UARTComponent *parent, uint8_t id)
      : PollingComponent(500), uart::UARTDevice(parent), servo_id_(id) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // movement
  void rotate(bool cw);
  void stop();

  // torque
  void set_torque(bool on);
  void set_torque_switch(St3215TorqueSwitch *s);

  // calibration
  void set_zero();
  void set_max();

  // sensors
  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }
  void set_turns_full_open(float t) { turns_full_open_ = t; }

 protected:
  uint8_t servo_id_{1};
  bool torque_on_{true};

  // soft limits
  float zero_offset_{0};
  float max_turns_{0};
  bool has_zero_{false};
  bool has_max_{false};

  float turns_unwrapped_{0};

  // sensors
  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};

  St3215TorqueSwitch *torque_switch_{nullptr};
  float turns_full_open_{0};

  // helpers
  uint8_t checksum_(const uint8_t *data, size_t len);
  void send_packet_(uint8_t id, uint8_t cmd, const std::vector<uint8_t> &params);
  bool read_registers_(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> &out);
};

}  // namespace st3215_servo
}  // namespace esphome
