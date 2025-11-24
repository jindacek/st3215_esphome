// =====================================================================
// move_relative using STS WritePosEx multiturn-ish (start at 0x29, 7 bytes)
// data: [acc, posL, posH, 0, 0, speedL, speedH]
// NOTE: ST3215 expects absolute 16-bit position at 0x29, NOT split turns.
// =====================================================================
void St3215Servo::move_relative(float turns_delta, int speed) {
  if (speed < 0) speed = 0;
  if (speed > 3400) speed = 3400;
  if (!torque_on_) set_torque(true);

  float target_turns = turns_unwrapped_ + turns_delta;
  int32_t target_raw_total = (int32_t) lroundf(target_turns * RAW_PER_TURN);

  // ST3215 WritePosEx uses uint16 absolute position
  if (target_raw_total < 0) target_raw_total = 0;
  if (target_raw_total > 65535) target_raw_total = 65535;

  uint16_t pos = (uint16_t) target_raw_total;
  uint16_t spd = (uint16_t) speed;

  std::vector<uint8_t> data = {
      (uint8_t)DEFAULT_ACC,
      (uint8_t)(pos & 0xFF),
      (uint8_t)((pos >> 8) & 0xFF),
      0x00,  // required padding (per working V5 packets)
      0x00,  // required padding
      (uint8_t)(spd & 0xFF),
      (uint8_t)((spd >> 8) & 0xFF),
  };

  // IMPORTANT: start address 0x29 for WritePosEx on ST3215
  write_registers_(0x29, data);
}
