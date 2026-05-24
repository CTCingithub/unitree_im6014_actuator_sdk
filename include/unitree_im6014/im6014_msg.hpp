#pragma once
#include <cmath>
#include <cstdint>

namespace unitree {
namespace IM6014 {

// ================= 减速比定义 =================
// IM6014 减速比: 38/3 ≈ 12.6667
// 所有用户接口使用"输出端"物理量，内部自动完成转子端?输出端换算
constexpr double GEAR_RATIO = 38.0 / 3.0;

// ================= 协议缩放系数 (转子端) =================
// 以下系数用于 转子端物理量 ? 协议整数值 的转换
constexpr double FACTOR_TOR = 2560.0;              // 1 Nm (rotor) = 2560
constexpr double FACTOR_SPD = 64.0 / (2.0 * M_PI); // 1 rad/s (rotor) = 64/(2π)
constexpr double FACTOR_POS =
    32768.0 / (2.0 * M_PI);           // 1 rad (rotor) = 32768/(2π)
constexpr double FACTOR_KP = 12800.0; // 1 Nm/rad (rotor) = 12800
constexpr double FACTOR_KD = 51200.0; // 1 Nm/(rad/s) (rotor) = 51200

#pragma pack(push, 1)
// 发送命令包 (20 Bytes) - 协议层仍使用转子端整数值
struct ControlData_t {
  uint8_t head[2];           // 0xFE 0xEE (参与CRC)
  uint8_t id_status_timeout; // 手动打包: id(0-3), status(4-6), timeout(7)
  uint8_t res;
  int16_t tor_des; // 转子端前馈扭矩 (整数值)
  int16_t spd_des; // 转子端目标速度 (整数值)
  int32_t pos_des; // 转子端目标位置 (整数值)
  int16_t k_pos;   // 转子端位置刚度 (整数值)
  int16_t k_spd;   // 转子端速度刚度 (整数值)
  uint32_t CRC32;  // CRC32-MPEG2 (包含包头)
};

// 接收状态包 (26 Bytes) - 协议层返回转子端整数值
struct MotorData_t {
  uint8_t head[2]; // 0xFC 0xEE (不参与CRC)
  uint8_t id_status_timeout;
  int8_t temp1;           // 驱动温度 ℃
  uint8_t temp2;          // 绕组温度 ℃
  uint8_t vol;            // 电压 (0.5V/step)
  int16_t torque;         // 转子端当前扭矩 (整数值)
  int16_t speed;          // 转子端当前速度 (整数值)
  int32_t pos;            // 转子端当前位置 (整数值)
  uint32_t MError;        // 错误码
  uint16_t res1_mwarning; // RES1(0-12), MWarning(13-15)
  uint16_t RES2;
  uint32_t CRC32; // CRC32-MPEG2 (不包含包头)
};
#pragma pack(pop)

// ================= 位域安全操作辅助函数 =================
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

// ================= 输出端 ? 转子端 换算辅助函数 =================
// 发送方向: 用户输出端 → 协议转子端
inline int16_t encode_torque(float tor_out) {
  float tor_rotor = tor_out / GEAR_RATIO;
  int32_t val = static_cast<int32_t>(tor_rotor * FACTOR_TOR);
  return static_cast<int16_t>(std::max(-32768, std::min(32767, val)));
}
inline int16_t encode_speed(float spd_out) {
  float spd_rotor = spd_out * GEAR_RATIO;
  int32_t val = static_cast<int32_t>(spd_rotor * FACTOR_SPD);
  return static_cast<int16_t>(std::max(-32768, std::min(32767, val)));
}
inline int32_t encode_position(float pos_out) {
  float pos_rotor = pos_out * GEAR_RATIO;
  // int32_t 范围足够，无需额外钳位
  return static_cast<int32_t>(pos_rotor * FACTOR_POS);
}
inline int16_t encode_kp(float kp_out) {
  float kp_rotor = kp_out / (GEAR_RATIO * GEAR_RATIO);
  int32_t val = static_cast<int32_t>(kp_rotor * FACTOR_KP);
  return static_cast<int16_t>(std::max(0, std::min(32767, val))); // Kp 无负值
}
inline int16_t encode_kd(float kd_out) {
  float kd_rotor = kd_out / (GEAR_RATIO * GEAR_RATIO);
  int32_t val = static_cast<int32_t>(kd_rotor * FACTOR_KD);
  return static_cast<int16_t>(std::max(0, std::min(32767, val))); // Kd 无负值
}

// 接收方向: 协议转子端 → 用户输出端
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