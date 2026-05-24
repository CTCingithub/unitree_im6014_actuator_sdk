#include "unitree_im6014/im6014_motor.hpp"
#include "unitree_im6014/crc32.hpp"
#include <algorithm>
#include <cstring>
#include <iostream>

namespace unitree {
namespace IM6014 {

IM6014Motor::IM6014Motor() {}
IM6014Motor::~IM6014Motor() { close(); }

bool IM6014Motor::init(const std::string &port, int baudrate) {
  return serial_.open(port, baudrate);
}
void IM6014Motor::close() { serial_.close(); }

void IM6014Motor::prepare_tx(ControlData_t &tx, uint8_t id, float torque,
                             float speed, float position, float kp,
                             float kd, uint8_t status, uint8_t timeout) {
  std::memset(&tx, 0, sizeof(tx));
  tx.head[0] = 0xFE;
  tx.head[1] = 0xEE;
  pack_tx(tx, id, status, timeout);

  // 输出端 → 转子端 → 协议整数值
  tx.tor_des = encode_torque(torque);
  tx.spd_des = encode_speed(speed);
  tx.pos_des = encode_position(position);
  tx.k_pos = encode_kp(kp);
  tx.k_spd = encode_kd(kd);

  // CRC: 发送包包含包头，共16字节
  tx.CRC32 = crc32_mpeg2((uint8_t *)&tx, 16);
}

bool IM6014Motor::send_cmd(uint8_t id, float torque, float speed,
                           float position, float kp, float kd,
                           uint8_t status, uint8_t timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  ControlData_t tx;
  prepare_tx(tx, id, torque, speed, position, kp, kd, status,
             timeout);
  return serial_.write((uint8_t *)&tx, sizeof(tx)) == sizeof(tx);
}

bool IM6014Motor::verify_rx_crc(const MotorData_t &rx) {
  // 接收包 CRC: 不包含包头(跳过前2字节)，共20字节
  uint32_t calc_crc = crc32_mpeg2((uint8_t *)&rx + 2, 20);
  return calc_crc == rx.CRC32;
}

// ? 核心修改：将转子端协议值解码为输出端物理量
bool IM6014Motor::recv_state(uint8_t id, IM6014State &state, int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[64];
  int bytes_read = serial_.read(buffer, sizeof(buffer), timeout_ms);
  state.valid = false;
  if (bytes_read < (int)sizeof(MotorData_t))
    return false;

  for (int i = 0; i <= bytes_read - (int)sizeof(MotorData_t); ++i) {
    if (buffer[i] == 0xFC && buffer[i + 1] == 0xEE) {
      MotorData_t rx;
      std::memcpy(&rx, &buffer[i], sizeof(rx));
      if (verify_rx_crc(rx)) {
        uint8_t rx_id = get_rx_id(rx);
        if (rx_id == id || id == 15) {
          state.id = rx_id;
          state.status = get_rx_status(rx);
          state.timeout = get_rx_timeout(rx);
          state.temp1 = rx.temp1;
          state.temp2 = rx.temp2;
          state.voltage = rx.vol * 0.5f; // 电压换算不变

          // ? 转子端 → 输出端 换算
          state.torque = decode_torque(rx.torque);
          state.speed = decode_speed(rx.speed);
          state.pos = decode_position(rx.pos);

          state.error = rx.MError;
          state.warning = get_rx_mwarning(rx);
          state.valid = true;
          return true;
        }
      }
    }
  }
  return false;
}

} // namespace IM6014
} // namespace unitree