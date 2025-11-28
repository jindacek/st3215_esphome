#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/helpers.h"
#include <vector>

// -----------------------------------------------------

namespace esphome {
namespace st3215_servo {

static const float RAW_PER_TURN = 4096.0f;

class St3215Servo;

// ======================== Torque Switch ========================
class St3215TorqueSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void write_state(bool state) override;

 protected:
  St3215Servo *parent_{nullptr};
};

// =========================== Servo =============================
class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  St3215Servo()
      : PollingComponent(500), uart::UARTDevice(nullptr) {}

  St3215Servo(uart::UARTComponent *parent, uint8_t id)
      : PollingComponent(500), uart::UARTDevice(parent), servo_id_(id) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // YAML settery
  void set_servo_id(uint8_t id) { servo_id_ = id; }

  // turns_full_open použijeme jako defaultní informativní hodnotu,
  // ale finální max_turns_ nastavujeme kalibrací
  void set_turns_full_open(float t) {
    configured_max_turns_ = t;
    // pokud ještě nemáme max z kalibrace, vezmeme tuto hodnotu jako dočasnou
    if (!has_max_ && t > 0.1f) {
      max_turns_ = t;
      has_max_ = true;
    }
  }

  // compatibility dummy:
  void set_max_angle(float) {}

  // Ovládání
  void rotate(bool cw, int speed);
  void stop();
  void set_torque(bool on);

  // Kalibrace – veřejné metody volané z YAML
  void start_calibration();
  void confirm_calibration_step();
  void set_zero();
  void set_max();

  // Sensory
  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }

  // Textový senzor – stav/hlášky
  void set_state_text_sensor(text_sensor::TextSensor *s) { state_text_sensor_ = s; }

  // Switch
  void set_torque_switch(St3215TorqueSwitch *s);

 protected:
  enum CalibState : uint8_t {
    CALIB_IDLE = 0,
    CALIB_WAIT_ZERO,
    CALIB_WAIT_MAX
  };

  uint8_t servo_id_{1};

  bool torque_on_{true};

  uint16_t last_raw_{0};
  bool have_last_{false};
  float turns_unwrapped_{0};

  float zero_offset_{0};
  float max_turns_{0};
  float configured_max_turns_{0};
  bool has_zero_{false};
  bool has_max_{false};

  bool calibration_active_{false};
  bool calibrated_{false};
  CalibState calib_state_{CALIB_IDLE};

  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};

  text_sensor::TextSensor *state_text_sensor_{nullptr};

  St3215TorqueSwitch *torque_switch_{nullptr};

  // Low level comm
  uint8_t checksum_(const uint8_t *data, size_t len);
  void send_packet_(uint8_t id, uint8_t cmd, const std::vector<uint8_t> &params);
  bool read_registers_(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> &out);

  void publish_state_text_(const char *msg);
};

}  // namespace st3215_servo
}  // namespace esphome
