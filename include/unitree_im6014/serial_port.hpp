#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
namespace unitree {
namespace IM6014 {
class SerialPort {
public:
  SerialPort();
  ~SerialPort();
  bool open(const std::string &port, int baudrate);
  void close();
  bool is_open() const;
  int write(const uint8_t *data, size_t length);
  int write_all(const uint8_t *data, size_t length);
  int read(uint8_t *buffer, size_t max_length, int timeout_ms);

private:
  int fd_;
};
} // namespace IM6014
} // namespace unitree
