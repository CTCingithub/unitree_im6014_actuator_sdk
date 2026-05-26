#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace unitree {
namespace IM6014 {

// ================= ���ٱȶ��� =================
// IM6014 ���ٱ�: 38/3 �� 12.6667
// �����û��ӿ�ʹ��"�����"���������ڲ��Զ����ת�Ӷ�?����˻���
constexpr double GEAR_RATIO = 38.0 / 3.0;

// ================= Э������ϵ�� (ת�Ӷ�) =================
// ����ϵ������ ת�Ӷ������� ? Э������ֵ ��ת��
constexpr double FACTOR_TOR = 2560.0;              // 1 Nm (rotor) = 2560
constexpr double FACTOR_SPD = 64.0 / (2.0 * M_PI); // 1 rad/s (rotor) = 64/(2��)
constexpr double FACTOR_POS =
    32768.0 / (2.0 * M_PI);           // 1 rad (rotor) = 32768/(2��)
constexpr double FACTOR_KP = 12800.0; // 1 Nm/rad (rotor) = 12800
constexpr double FACTOR_KD = 51200.0; // 1 Nm/(rad/s) (rotor) = 51200

#pragma pack(push, 1)
// ��������� (20 Bytes) - Э�����ʹ��ת�Ӷ�����ֵ
struct ControlData_t {
  uint8_t head[2];           // 0xFE 0xEE (����CRC)
  uint8_t id_status_timeout; // �ֶ����: id(0-3), status(4-6), timeout(7)
  uint8_t res;
  int16_t tor_des; // ת�Ӷ�ǰ��Ť�� (����ֵ)
  int16_t spd_des; // ת�Ӷ�Ŀ���ٶ� (����ֵ)
  int32_t pos_des; // ת�Ӷ�Ŀ��λ�� (����ֵ)
  int16_t k_pos;   // ת�Ӷ�λ�øն� (����ֵ)
  int16_t k_spd;   // ת�Ӷ��ٶȸն� (����ֵ)
  uint32_t CRC32;  // CRC32-MPEG2 (������ͷ)
};

// ����״̬�� (26 Bytes) - Э��㷵��ת�Ӷ�����ֵ
struct MotorData_t {
  uint8_t head[2]; // 0xFC 0xEE (������CRC)
  uint8_t id_status_timeout;
  int8_t temp1;           // �����¶� ��
  uint8_t temp2;          // �����¶� ��
  uint8_t vol;            // ��ѹ (0.5V/step)
  int16_t torque;         // ת�Ӷ˵�ǰŤ�� (����ֵ)
  int16_t speed;          // ת�Ӷ˵�ǰ�ٶ� (����ֵ)
  int32_t pos;            // ת�Ӷ˵�ǰλ�� (����ֵ)
  uint32_t MError;        // ������
  uint16_t res1_mwarning; // RES1(0-12), MWarning(13-15)
  uint16_t RES2;
  uint32_t CRC32; // CRC32-MPEG2 (��������ͷ)
};
#pragma pack(pop)

// ================= λ��ȫ������������ =================
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

// ================= ����� ? ת�Ӷ� ���㸨������ =================
// ���ͷ���: �û������ �� Э��ת�Ӷ�
inline int16_t encode_torque(float tor_out) {
  float tor_rotor = tor_out / GEAR_RATIO;
  long val = std::lround(tor_rotor * FACTOR_TOR);
  return static_cast<int16_t>(std::max(-32768L, std::min(32767L, val)));
}
inline int16_t encode_speed(float spd_out) {
  float spd_rotor = spd_out * GEAR_RATIO;
  long val = std::lround(spd_rotor * FACTOR_SPD);
  return static_cast<int16_t>(std::max(-32768L, std::min(32767L, val)));
}
inline int32_t encode_position(float pos_out) {
  float pos_rotor = pos_out * GEAR_RATIO;
  // int32_t ��Χ�㹻���������ǯλ
  return static_cast<int32_t>(std::lround(pos_rotor * FACTOR_POS));
}
inline int16_t encode_kp(float kp_out) {
  float kp_rotor = kp_out / (GEAR_RATIO * GEAR_RATIO);
  long val = std::lround(kp_rotor * FACTOR_KP);
  return static_cast<int16_t>(std::max(0L, std::min(32767L, val))); // Kp �޸�ֵ
}
inline int16_t encode_kd(float kd_out) {
  float kd_rotor = kd_out / (GEAR_RATIO * GEAR_RATIO);
  long val = std::lround(kd_rotor * FACTOR_KD);
  return static_cast<int16_t>(std::max(0L, std::min(32767L, val))); // Kd �޸�ֵ
}

// ���շ���: Э��ת�Ӷ� �� �û������
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