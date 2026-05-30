#include "unitree_im6014/serial_port.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// termios2 for custom baud rates (not exposed by glibc headers)
struct termios2 {
  tcflag_t c_iflag;
  tcflag_t c_oflag;
  tcflag_t c_cflag;
  tcflag_t c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  speed_t c_ispeed;
  speed_t c_ospeed;
};
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#define BOTHER 0010000

namespace unitree {
namespace IM6014 {

SerialPort::SerialPort() : fd_(-1) {}
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string &port, int baudrate) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    std::cerr << "[SerialPort] open(" << port
              << ") failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  // Clear non-blocking so VMIN / VTIME blocking reads take effect
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags >= 0)
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios2 tio;
  if (ioctl(fd_, TCGETS2, &tio) < 0) {
    std::cerr << "[SerialPort] TCGETS2 failed: " << std::strerror(errno)
              << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_cc[VTIME] = 1;
  tio.c_cc[VMIN] = 1;

  if (ioctl(fd_, TCSETS2, &tio) < 0) {
    std::cerr << "[SerialPort] TCSETS2 failed: " << std::strerror(errno)
              << std::endl;
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  tcflush(fd_, TCIFLUSH);
  return true;
}

void SerialPort::close() {
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const { return fd_ != -1; }

int SerialPort::write(const uint8_t *data, size_t length) {
  return fd_ == -1 ? -1 : static_cast<int>(::write(fd_, data, length));
}

int SerialPort::write_all(const uint8_t *data, size_t length) {
  if (fd_ == -1)
    return -1;
  size_t written = 0;
  while (written < length) {
    ssize_t n = ::write(fd_, data + written, length - written);
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN)
        continue;
      return -1;
    }
    written += static_cast<size_t>(n);
  }
  return static_cast<int>(written);
}

int SerialPort::read(uint8_t *buffer, size_t max_length, int timeout_ms) {
  if (fd_ == -1)
    return -1;
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd_, &set);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  if (select(fd_ + 1, &set, nullptr, nullptr, &tv) <= 0)
    return 0;
  return static_cast<int>(::read(fd_, buffer, max_length));
}

} // namespace IM6014
} // namespace unitree
