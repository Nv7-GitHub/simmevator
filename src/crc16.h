#pragma once
//
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
//
// Deliberately free of any Arduino dependency so that the packet formats built
// on top of it can be compiled and unit-tested on the host - see the `native`
// environment in platformio.ini. Shared by ping_packet.h and baro_packet.h so
// there is exactly one definition of the polynomial in the project.
//

#include <stddef.h>
#include <stdint.h>

static inline uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}
