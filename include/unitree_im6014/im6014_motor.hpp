#pragma once
#include "im6014_msg.hpp"
#include "serial_port.hpp"
#include <mutex>
#include <string>
namespace unitree {
namespace IM6014 {

// ������������Ϊ����˵�λ (�������ٱȻ���)
struct State {
  int id, status, timeout;
  float temp1, temp2, voltage; // ��, V
  float torque;                // ��������� (Nm)
  float speed;                 // ������ٶ� (rad/s)
  float pos;                   // �����λ�� (rad)
  uint32_t error;
  uint16_t warning;
  bool valid;
};

class Motor {
public:
  Motor();
  ~Motor();
  bool init(const std::string &port, int baudrate = 4000000);
  void close();
  // �������������Ϊ����˵�λ
  bool send_cmd(uint8_t id, float torque, float speed, float position, float kp,
                float kd, uint8_t status = 1, uint8_t timeout = 0);
  bool recv_state(uint8_t id, State &state, int timeout_ms = 50);

private:
  SerialPort serial_;
  std::mutex mutex_;
  bool verify_rx_crc(const MotorData_t &rx);
  // �ڲ�������������˲�������ΪЭ���
  void prepare_tx(ControlData_t &tx, uint8_t id, float tor_out, float spd_out,
                  float pos_out, float kp_out, float kd_out, uint8_t status,
                  uint8_t timeout);
};
} // namespace IM6014
} // namespace unitree