#include "unitree_im6014/im6014_motor.hpp"
#include "unitree_im6014/crc32.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>

namespace unitree {
namespace IM6014 {

// ==================== life-cycle ====================
Motor::Motor() = default;
Motor::~Motor() { close(); }

bool Motor::init(const std::string &port, int baudrate) {
  return serial_.open(port, baudrate);
}
void Motor::close() { serial_.close(); }

// ==================== command ====================
void Motor::prepare_tx(ControlData_t &tx, uint8_t id, float torque, float speed,
                       float position, float kp, float kd, uint8_t status,
                       uint8_t timeout) {
  tx.head[0] = 0xFE;
  tx.head[1] = 0xEE;
  tx.res = 0;
  pack_tx(tx, id, status, timeout);

  tx.tor_des = encode_torque(torque);
  tx.spd_des = encode_speed(speed);
  tx.pos_des = encode_position(position);
  tx.k_pos = encode_kp(kp);
  tx.k_spd = encode_kd(kd);

  tx.CRC32 = crc32_mpeg2(reinterpret_cast<const uint8_t *>(&tx), 16);
}

bool Motor::send_cmd(uint8_t id, float torque, float speed, float position,
                     float kp, float kd, uint8_t status, uint8_t timeout) {
  std::lock_guard<std::mutex> lock(mutex_);
  ControlData_t tx;
  prepare_tx(tx, id, torque, speed, position, kp, kd, status, timeout);
  return serial_.write_all(reinterpret_cast<const uint8_t *>(&tx),
                           sizeof(tx)) == static_cast<int>(sizeof(tx));
}

// ==================== frame parsing ====================
bool Motor::verify_rx_crc(const MotorData_t &rx) {
  uint32_t calc = crc32_mpeg2(reinterpret_cast<const uint8_t *>(&rx) + 2, 20);
  return calc == rx.CRC32;
}

State Motor::decode_state(const MotorData_t &rx) {
  State s;
  s.id = get_rx_id(rx);
  s.status = get_rx_status(rx);
  s.timeout = get_rx_timeout(rx);
  s.temp1 = static_cast<float>(rx.temp1);
  s.temp2 = static_cast<float>(rx.temp2);
  s.voltage = rx.vol * 0.5f;
  s.torque = decode_torque(rx.torque);
  s.speed = decode_speed(rx.speed);
  s.pos = decode_position(rx.pos);
  s.error = rx.MError;
  s.warning = get_rx_mwarning(rx);
  s.valid = true;
  s.timestamp_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
  return s;
}

// ==================== polled reception ====================
void Motor::poll_states(int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);

  // 1. Drain serial data into the accumulation buffer
  uint8_t temp[128];
  int n = serial_.read(temp, sizeof(temp), timeout_ms);
  if (n > 0)
    rx_buffer_.insert(rx_buffer_.end(), temp, temp + n);

  // 2. Scan for complete frames, byte-by-byte to tolerate misalignment
  size_t i = 0;
  while (i + sizeof(MotorData_t) <= rx_buffer_.size()) {
    if (rx_buffer_[i] == 0xFC && rx_buffer_[i + 1] == 0xEE) {
      MotorData_t rx;
      std::memcpy(&rx, &rx_buffer_[i], sizeof(rx));
      if (verify_rx_crc(rx)) {
        State s = decode_state(rx);
        {
          std::lock_guard<std::mutex> s_lock(state_mutex_);
          latest_states_[s.id] = s;
        }
        i += sizeof(MotorData_t); // skip this complete frame
        continue;
      }
    }
    ++i; // CRC failed or header not found at this offset — step one byte
  }

  // 3. Discard bytes that have already been examined.
  //    If i == 0 we keep everything (no header was found at all).
  //    If i < buffer size we keep the trailing partial-frame bytes.
  if (i > 0)
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i);
}

bool Motor::get_state(uint8_t id, State &state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto it = latest_states_.find(id);
  if (it == latest_states_.end())
    return false;
  state = it->second;
  return state.valid;
}

// ==================== legacy request-response ====================
bool Motor::recv_state(uint8_t id, State &state, int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint8_t buffer[128];
  int total = 0;
  int remaining_ms = timeout_ms;

  // Read with timeout, accommodating partial arrivals
  while (total < static_cast<int>(sizeof(MotorData_t))) {
    int n = serial_.read(buffer + total, sizeof(buffer) - total, remaining_ms);
    if (n <= 0) {
      state.valid = false;
      return false;
    }
    total += n;
    // Try to find a frame in what we have so far
    for (int i = 0; i <= total - static_cast<int>(sizeof(MotorData_t)); ++i) {
      if (buffer[i] == 0xFC && buffer[i + 1] == 0xEE) {
        MotorData_t rx;
        std::memcpy(&rx, &buffer[i], sizeof(rx));
        if (verify_rx_crc(rx)) {
          uint8_t rx_id = get_rx_id(rx);
          if (rx_id == id || id == WILDCARD_ID) {
            state = decode_state(rx);
            return true;
          }
        }
      }
    }
  }
  state.valid = false;
  return false;
}

} // namespace IM6014
} // namespace unitree
