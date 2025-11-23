#include "st3215_servo.h"
#include "esphome/core/log.h"

namespace esphome {
namespace st3215_servo {

static const char *const TAG = "st3215_servo";
static const uint32_t RESPONSE_TIMEOUT_MS = 25;

void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 setup id=%u", servo_id_);

  // Register sensors so Sensor-related symbols get linked
  if (angle_sensor_)   this->register_child(angle_sensor_);
  if (turns_sensor_)   this->register_child(turns_sensor_);
  if (percent_sensor_) this->register_child(percent_sensor_);
}


uint8_t St3215Servo::checksum_(uint8_t id, uint8_t len, uint8_t inst, const std::vector<uint8_t> &params) {
  uint16_t sum = id + len + inst;
  for (auto b : params) sum += b;
  return (~sum) & 0xFF;
}

void St3215Servo::send_packet_(uint8_t inst, const std::vector<uint8_t> &params) {
  uint8_t len = params.size() + 2;
  uint8_t chk = checksum_(servo_id_, len, inst, params);

  std::vector<uint8_t> pkt;
  pkt.reserve(params.size() + 6);
  pkt.push_back(0xFF);
  pkt.push_back(0xFF);
  pkt.push_back(servo_id_);
  pkt.push_back(len);
  pkt.push_back(inst);
  pkt.insert(pkt.end(), params.begin(), params.end());
  pkt.push_back(chk);

  this->write_array(pkt);
  this->flush();
}

bool St3215Servo::read_response_(std::vector<uint8_t> &out) {
  uint32_t start = millis();
  out.clear();
  while (millis() - start < RESPONSE_TIMEOUT_MS) {
    while (this->available()) {
      out.push_back(this->read());
      if (out.size() >= 4) {
        if (out[0] != 0xFF || out[1] != 0xFF) {
          out.erase(out.begin());
          continue;
        }
        uint8_t len = out[3];
        if (out.size() >= len + 4) return true;
      }
    }
  }
  return false;
}

void St3215Servo::send_write_(uint8_t addr, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> params;
  params.reserve(data.size() + 1);
  params.push_back(addr);
  params.insert(params.end(), data.begin(), data.end());
  send_packet_(INST_WRITE, params);
}

bool St3215Servo::send_read_(uint8_t addr, uint8_t len, std::vector<uint8_t> &out) {
  std::vector<uint8_t> params{addr, len};
  send_packet_(INST_READ, params);
  return read_response_(out);
}

void St3215Servo::rotate(bool cw, int speed) {
  speed = clamp(speed, 0, 1000);
  int16_t signed_speed = cw ? speed : -speed;
  uint16_t raw = (uint16_t) signed_speed;
  std::vector<uint8_t> data{(uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF)};
  send_write_(ADDR_GOAL_SPEED_L, data);
}

void St3215Servo::stop() { rotate(true, 0); }

void St3215Servo::set_angle(float degrees, int speed) {
  degrees = clamp(degrees, 0.0f, max_angle_);
  speed = clamp(speed, 0, 1000);
  uint16_t pos = (uint16_t)(degrees / max_angle_ * 4095.0f);
  std::vector<uint8_t> params{
      (uint8_t)(pos & 0xFF),
      (uint8_t)((pos >> 8) & 0xFF),
      0x00, 0x00,
      (uint8_t)(speed & 0xFF),
      (uint8_t)((speed >> 8) & 0xFF),
  };
  send_write_(ADDR_GOAL_POSITION_L, params);
}

void St3215Servo::update_turn_counter_(float new_angle) {
  if (!has_last_angle_) {
    last_angle_ = new_angle;
    has_last_angle_ = true;
    return;
  }
  float delta = new_angle - last_angle_;
  if (delta > max_angle_ / 2.0f) current_turns_ -= 1.0f;
  else if (delta < -max_angle_ / 2.0f) current_turns_ += 1.0f;
  last_angle_ = new_angle;
}

void St3215Servo::move_to_turns(float turns, int speed) {
  float frac = fmodf(turns, 1.0f);
  if (frac < 0) frac += 1.0f;
  set_angle(frac * max_angle_, speed);
}

void St3215Servo::move_to_percent(float percent, int speed) {
  if (turns_full_open_ <= 0.0f) return;
  percent = clamp(percent, 0.0f, 100.0f);
  move_to_turns((percent / 100.0f) * turns_full_open_, speed);
}

void St3215Servo::update() {
  std::vector<uint8_t> resp;
  if (!send_read_(ADDR_PRESENT_POSITION_L, 2, resp)) return;
  if (resp.size() < 8) return;

  uint16_t raw = resp[5] | (resp[6] << 8);
  float angle = (raw / 4095.0f) * max_angle_;
  update_turn_counter_(angle);
  current_angle_ = angle;

  float turns_abs = current_turns_ + current_angle_ / max_angle_;
  if (angle_sensor_) angle_sensor_->publish_state(current_angle_);
  if (turns_sensor_) turns_sensor_->publish_state(turns_abs);
  if (percent_sensor_ && turns_full_open_ > 0.0f) {
    float pct = clamp(turns_abs / turns_full_open_ * 100.0f, 0.0f, 100.0f);
    percent_sensor_->publish_state(pct);
  }
}

}  // namespace st3215_servo
}  // namespace esphome
