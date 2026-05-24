#include "unitree_im6014/crc32.hpp"

namespace unitree {
namespace IM6014 {
uint32_t crc32_mpeg2(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  const uint32_t poly = 0x04C11DB7;
  for (size_t i = 0; i < length; i++) {
    crc ^= ((uint32_t)data[i] << 24);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80000000)
        crc = (crc << 1) ^ poly;
      else
        crc <<= 1;
    }
  }
  return crc;
}
} // namespace IM6014
} // namespace unitree