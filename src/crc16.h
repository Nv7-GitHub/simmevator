#pragma once
//
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
//
// Deliberately free of any Arduino dependency so that the packet formats built
// on top of it can be compiled and unit-tested on the host - see the `native`
// environment in platformio.ini. mesh_packet.h is the only consumer here - the
// two LoRa formats in elev_packet.h carry no application CRC, because the
// SX1262's hardware CRC-16 has already rejected corrupt frames by the time
// RadioLib hands one over.
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
