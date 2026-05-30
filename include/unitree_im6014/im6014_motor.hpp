#pragma once
#include "im6014_msg.hpp"
#include "serial_port.hpp"
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace unitree {
namespace IM6014 {

// Decoded motor state in output-end (load-side) units
struct State {
  int id = 0;
  int status = 0;
  int timeout = 0;
  float temp1 = 0;   // deg C
  float temp2 = 0;   // deg C
  float voltage = 0; // V
  float torque = 0;  // Nm   (output)
  float speed = 0;   // rad/s (output)
  float pos = 0;     // rad   (output)
  uint32_t error = 0;
  uint16_t warning = 0;
  bool valid = false;
  uint64_t timestamp_us = 0; // steady_clock timestamp of reception
};

class Motor {
public:
  Motor();
  ~Motor();

  // ----- life-cycle -----
  bool init(const std::string &port, int baudrate = 4000000);
  void close();

  // ----- command -----
  // Send a single control command (output-end units).
  // Uses write_all to guarantee the full 20-byte packet is sent.
  bool send_cmd(uint8_t id, float torque, float speed, float position, float kp,
                float kd, uint8_t status = MotorStatus::ENABLE,
                uint8_t timeout = 0);

  // ----- polled reception (recommended for multi-motor / high-rate) -----
  // Drain the serial port into the internal buffer, parse all complete
  // frames, and update the state cache. Call once per control cycle.
  void poll_states(int timeout_ms = 2);

  // Retrieve the most recent cached state for a given motor id.
  // Returns false if no valid frame has been received yet.
  bool get_state(uint8_t id, State &state);

  // ----- legacy request-response (single-motor, debugging) -----
  // Block until a single frame matching `id` (or WILDCARD_ID) arrives.
  // Kept for backwards compatibility and simple interactive scripts.
  bool recv_state(uint8_t id, State &state, int timeout_ms = 50);

private:
  SerialPort serial_;
  std::mutex mutex_;

  // Reception buffer & cache
  std::vector<uint8_t> rx_buffer_;
  std::unordered_map<uint8_t, State> latest_states_;
  std::mutex state_mutex_;

  // Helpers
  bool verify_rx_crc(const MotorData_t &rx);
  void prepare_tx(ControlData_t &tx, uint8_t id, float tor_out, float spd_out,
                  float pos_out, float kp_out, float kd_out, uint8_t status,
                  uint8_t timeout);
  State decode_state(const MotorData_t &rx);
};

} // namespace IM6014
} // namespace unitree
