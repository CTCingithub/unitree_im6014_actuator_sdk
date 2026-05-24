// crc32.hpp (²»±ä)
#pragma once
#include <cstddef>
#include <cstdint>
namespace unitree {
namespace IM6014 {
uint32_t crc32_mpeg2(const uint8_t *data, size_t length);
}
} // namespace unitree
