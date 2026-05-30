#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace unitree {
namespace IM6014 {

// ==================== Gear ratio & protocol constants ====================
constexpr double GEAR_RATIO = 38.0 / 3.0;

// Scaling factors (rotor-side raw values)
constexpr double FACTOR_TOR = 2560.0;
constexpr double FACTOR_SPD = 64.0 / (2.0 * M_PI);
constexpr double FACTOR_POS = 32768.0 / (2.0 * M_PI);
constexpr double FACTOR_KP = 12800.0;
constexpr double FACTOR_KD = 51200.0;

// Motor ID range
constexpr uint8_t WILDCARD_ID = 15; // broadcast / match-any
constexpr uint8_t MAX_MOTOR_ID = 14;

// Motor status (protocol bits 4-6)
enum MotorStatus : uint8_t {
  DISABLE = 0,
  ENABLE = 1,
};

// ==================== Protocol packet structs ====================
#pragma pack(push, 1)

struct ControlData_t {
  uint8_t head[2];           // 0xFE 0xEE
  uint8_t id_status_timeout; // id(0-3), status(4-6), timeout(7)
  uint8_t res;
  int16_t tor_des; // feed-forward torque  (rotor raw)
  int16_t spd_des; // desired speed       (rotor raw)
  int32_t pos_des; // desired position    (rotor raw)
  int16_t k_pos;   // position stiffness  (rotor raw)
  int16_t k_spd;   // speed damping       (rotor raw)
  uint32_t CRC32;  // over first 16 bytes
};

struct MotorData_t {
  uint8_t head[2]; // 0xFC 0xEE
  uint8_t id_status_timeout;
  int8_t temp1;
  uint8_t temp2;
  uint8_t vol;    // 0.5 V / step
  int16_t torque; // current torque  (rotor raw)
  int16_t speed;  // current speed   (rotor raw)
  int32_t pos;    // current position (rotor raw)
  uint32_t MError;
  uint16_t res1_mwarning; // RES1(0-12), MWarning(13-15)
  uint16_t RES2;
  uint32_t CRC32; // header excluded (bytes 2-21)
};

#pragma pack(pop)

static_assert(sizeof(ControlData_t) == 20,
              "ControlData_t must be exactly 20 bytes");
static_assert(sizeof(MotorData_t) == 26,
              "MotorData_t must be exactly 26 bytes");

// ==================== Bit-field helpers ====================
inline void pack_tx(ControlData_t &tx, uint8_t id, uint8_t status,
                    uint8_t timeout) {
  tx.id_status_timeout =
      (id & 0x0F) | ((status & 0x07) << 4) | ((timeout & 0x01) << 7);
}
inline uint8_t get_rx_id(const MotorData_t &rx) {
  return rx.id_status_timeout & 0x0F;
}
inline uint8_t get_rx_status(const MotorData_t &rx) {
  return (rx.id_status_timeout >> 4) & 0x07;
}
inline uint8_t get_rx_timeout(const MotorData_t &rx) {
  return (rx.id_status_timeout >> 7) & 0x01;
}
inline uint16_t get_rx_mwarning(const MotorData_t &rx) {
  return (rx.res1_mwarning >> 13) & 0x07;
}

// ==================== Encoders: output-side → rotor raw ====================
// All use double intermediates to avoid precision loss.
// Clamping is silent (hot-path, 1 kHz capable) — callers should range-check
// inputs against the motor datasheet limits before entering the control loop.

inline int16_t encode_torque(float tor_out) {
  double tor_rotor = static_cast<double>(tor_out) / GEAR_RATIO;
  long val = std::lround(tor_rotor * FACTOR_TOR);
  return static_cast<int16_t>(std::max(-32768L, std::min(32767L, val)));
}

inline int16_t encode_speed(float spd_out) {
  double spd_rotor = static_cast<double>(spd_out) * GEAR_RATIO;
  long val = std::lround(spd_rotor * FACTOR_SPD);
  return static_cast<int16_t>(std::max(-32768L, std::min(32767L, val)));
}

inline int32_t encode_position(float pos_out) {
  double pos_rotor = static_cast<double>(pos_out) * GEAR_RATIO;
  double raw = std::round(pos_rotor * FACTOR_POS);
  if (raw > static_cast<double>(INT32_MAX))
    return INT32_MAX;
  if (raw < static_cast<double>(INT32_MIN))
    return INT32_MIN;
  return static_cast<int32_t>(raw);
}

inline int16_t encode_kp(float kp_out) {
  double kp_rotor = static_cast<double>(kp_out) / (GEAR_RATIO * GEAR_RATIO);
  long val = std::lround(kp_rotor * FACTOR_KP);
  return static_cast<int16_t>(std::max(0L, std::min(32767L, val)));
}

inline int16_t encode_kd(float kd_out) {
  double kd_rotor = static_cast<double>(kd_out) / (GEAR_RATIO * GEAR_RATIO);
  long val = std::lround(kd_rotor * FACTOR_KD);
  return static_cast<int16_t>(std::max(0L, std::min(32767L, val)));
}

// ==================== Decoders: rotor raw → output-side ====================
inline float decode_torque(int16_t torque_raw) {
  float tor_rotor = static_cast<float>(torque_raw) / FACTOR_TOR;
  return tor_rotor * GEAR_RATIO;
}
inline float decode_speed(int16_t speed_raw) {
  float spd_rotor = static_cast<float>(speed_raw) / FACTOR_SPD;
  return spd_rotor / GEAR_RATIO;
}
inline float decode_position(int32_t pos_raw) {
  float pos_rotor = static_cast<float>(pos_raw) / FACTOR_POS;
  return pos_rotor / GEAR_RATIO;
}

} // namespace IM6014
} // namespace unitree
