#pragma once
#include "im6014_msg.hpp"
#include "serial_port.hpp"
#include <mutex>
#include <string>
namespace unitree {
namespace IM6014 {

// 所有物理量均为输出端单位 (经过减速比换算)
struct IM6014State {
  int id, status, timeout;
  float temp1, temp2, voltage; // ℃, V
  float torque;                // 输出端力矩 (Nm)
  float speed;                 // 输出端速度 (rad/s)
  float pos;                   // 输出端位置 (rad)
  uint32_t error;
  uint16_t warning;
  bool valid;
};

class IM6014Motor {
public:
  IM6014Motor();
  ~IM6014Motor();
  bool init(const std::string &port, int baudrate = 4000000);
  void close();
  // 所有输入参数均为输出端单位
  bool send_cmd(uint8_t id, float torque, float speed, float position, float kp,
                float kd, uint8_t status = 1, uint8_t timeout = 1);
  bool recv_state(uint8_t id, IM6014State &state, int timeout_ms = 10);

private:
  SerialPort serial_;
  std::mutex mutex_;
  bool verify_rx_crc(const MotorData_t &rx);
  // 内部函数：将输出端参数编码为协议包
  void prepare_tx(ControlData_t &tx, uint8_t id, float tor_out, float spd_out,
                  float pos_out, float kp_out, float kd_out, uint8_t status,
                  uint8_t timeout);
};
} // namespace IM6014
} // namespace unitree