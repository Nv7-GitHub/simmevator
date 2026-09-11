#pragma once
//
// Wire format for what the car broadcasts over LoRa.
//
// Two formats share the air and are told apart by the first byte, the way
// baro_packet.h does it:
//
//   0xE0  STATE   8 bytes  - where the car is, sent while it moves.
//   0xE1  STATS  24 bytes  - battery, odometer and the learned model, once a
//                            minute whether or not anything is happening.
//
// Pure C++ over <stdint.h>: no Arduino.h, no Serial, no printing, no
// allocation. Unpack returns a reason code and the caller decides how to
// report it. That is what lets the format be unit-tested on the host by the
// `native` environment, where an offset or scaling error shows up in a second
// instead of as a display quietly reading the wrong floor.
//
// ---------------------------------------------------------------------------
// Why there is no application CRC on either format
// ---------------------------------------------------------------------------
// The SX1262's hardware CRC-16 is enabled on both ends, and RadioLib reports
// a CRC failure as a receive error rather than handing the frame over. A
// second CRC would spend 2 bytes re-checking what has already been checked,
// and airtime quantises in 8-symbol steps at SF10 - 2 bytes is enough to push
// a packet into the next step. The tag byte stays as a guard against a foreign
// frame that happens to be the right length. The ESP-NOW leg is different and
// does carry a CRC: see mesh_packet.h.
//
// ---------------------------------------------------------------------------
// Why the fields are scaled the way they are
// ---------------------------------------------------------------------------
//   * posQ8 is 1/256 of a floor, signed, relative to floor 1. At a 2.871 m
//     pitch that is 11 mm of resolution, well under what the barometer can
//     resolve, and +/-128 floors of range in an i16. It exists to animate a
//     display sweeping 3 -> 4 -> 5; it is derived from the drift-tracked datum
//     that ALGORITHM.md section 6 shows cannot be trusted to *decide* a floor,
//     so `floor` always overrides it on arrival.
//   * batteryMv is millivolts, not a float: a 4S LiFePO4 pack spans
//     11200..14600 mV and the divider is only good to +/-2-3% anyway.
//   * pitchMm is millimetres. The acceptance gate on the learned pitch is
//     +/-1 mm, so millimetres is exactly the resolution that matters.
//
// Every multi-byte field is little-endian, written out byte by byte.
//

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Format identifiers and sizes
// ---------------------------------------------------------------------------
#define ELEV_TAG_STATE 0xE0
#define ELEV_TAG_STATS 0xE1

// 8 bytes is 297 ms of airtime at SF10/BW125/CR4-8, 24 bytes is 494 ms. Both
// numbers are line items in the power budget, so neither length is free to
// grow without redoing it.
#define ELEV_STATE_BYTES 8
#define ELEV_STATS_BYTES 24

// The largest thing that goes on the air, for sizing caller buffers.
#define ELEV_MAX_PACKET_BYTES ELEV_STATS_BYTES

// tag, txId and seq are common to both formats and are stripped by the mesh
// wrapper, which carries the tag in its own `type` field.
#define ELEV_BODY_OFFSET 3

// ---------------------------------------------------------------------------
// Units
// ---------------------------------------------------------------------------
#define ELEV_POS_UNITS_PER_FLOOR 256
#define ELEV_MV_PER_VOLT         1000
#define ELEV_MM_PER_M            1000

// ---------------------------------------------------------------------------
// STATE byte at offset 6: bit0 moving, bits1-2 direction, bit3 modelReady, bit4 sensorErr
//
// Packed into one byte rather than spread over three because at 8 bytes total
// every byte is an airtime decision. Callers go through the accessors below
// instead of open-coding the masks - the direction field in particular is two
// bits at an offset, which is the kind of thing that gets shifted the wrong
// way in one place out of four.
// ---------------------------------------------------------------------------
#define ELEV_STATE_MOVING      0x01
#define ELEV_STATE_DIR_MASK    0x06
#define ELEV_STATE_DIR_SHIFT   1
#define ELEV_STATE_MODEL_READY 0x08
#define ELEV_STATE_SENSOR_ERR  0x10

enum ElevDirection {
  ELEV_DIR_IDLE = 0,
  ELEV_DIR_UP   = 1,
  ELEV_DIR_DOWN = 2
};

// ---------------------------------------------------------------------------
// STATS flags byte 22
// ---------------------------------------------------------------------------
#define ELEV_FLAG_SENSOR_ERR   0x01
#define ELEV_FLAG_MODEL_READY  0x02
// The model came back from NVS on boot rather than being learned this run.
// A display can tell "restored and trusted" from "learned here".
#define ELEV_FLAG_NVS_RESTORED 0x04
// Below the 12.0 V warn threshold, ~20% remaining on a 4S LiFePO4 pack. Sent
// as a bit as well as a voltage so the threshold lives on the transmitter,
// which is the only node that knows its own calibration.
#define ELEV_FLAG_LOW_BATTERY  0x08

// ---------------------------------------------------------------------------
// Decode results
// ---------------------------------------------------------------------------
enum ElevDecodeStatus {
  ELEV_OK = 0,
  ELEV_ERR_SHORT,   // buffer too small for the format the tag claims
  ELEV_ERR_TAG,     // first byte is not a format we speak
  ELEV_ERR_LENGTH   // right tag, wrong length - a truncated or padded frame
};

static inline const char *elevDecodeStatusName(ElevDecodeStatus s) {
  switch (s) {
    case ELEV_OK:         return "ok";
    case ELEV_ERR_SHORT:  return "too short";
    case ELEV_ERR_TAG:    return "unknown tag";
    case ELEV_ERR_LENGTH: return "length mismatch";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Little-endian primitives
//
// Written out byte by byte rather than memcpy'ing a struct: the ESP32-S3, the
// ESP32-WROOM and the host all happen to be little-endian and all happen to
// pack these structs the same way, but the wire format should not depend on
// three coincidences staying true, and explicit bytes are what the unit tests
// can pin to an offset.
// ---------------------------------------------------------------------------
static inline void elevPutU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
static inline uint16_t elevGetU16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline void elevPutU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline uint32_t elevGetU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ---------------------------------------------------------------------------
// In-memory representations
// ---------------------------------------------------------------------------

struct ElevState {
  uint8_t txId;
  uint8_t seq;         // u8, wraps every 256 packets; loss statistics only
  uint8_t floor;       // confirmed 1-based index, 0 = model not ready
  int16_t posQ8;       // 1/256 floor units, relative to floor 1
  uint8_t state;       // the bit field above
  uint8_t confidence;  // 0-255, from the lattice fit in _arrive()
};

struct ElevStats {
  uint8_t  txId;
  uint8_t  seq;
  uint16_t batteryMv;
  uint16_t dist24hM;    // metres over the last 24 hourly buckets
  uint32_t distTotalM;  // metres, lifetime
  uint16_t pitchMm;     // learned floor pitch
  uint8_t  nFloors;     // learned floor count
  uint16_t trips;
  uint16_t stops;
  uint32_t uptimeS;
  uint8_t  flags;       // the flag bits above
  int8_t   tempC;
};

// ---------------------------------------------------------------------------
// STATE encode / decode
// ---------------------------------------------------------------------------
// Returns the number of bytes written, or 0 if the arguments are unusable.
static inline size_t elevStatePack(uint8_t *out, size_t cap, const ElevState *s) {
  if (out == NULL || s == NULL) return 0;
  if (cap < ELEV_STATE_BYTES) return 0;

  out[0] = ELEV_TAG_STATE;
  out[1] = s->txId;
  out[2] = s->seq;
  out[3] = s->floor;
  elevPutU16(&out[4], (uint16_t)s->posQ8);
  out[6] = s->state;
  out[7] = s->confidence;
  return ELEV_STATE_BYTES;
}

static inline ElevDecodeStatus elevStateUnpack(const uint8_t *in, size_t len,
                                               ElevState *out) {
  if (in == NULL || out == NULL) return ELEV_ERR_SHORT;
  if (len < ELEV_STATE_BYTES) return ELEV_ERR_SHORT;
  if (in[0] != ELEV_TAG_STATE) return ELEV_ERR_TAG;
  if (len != ELEV_STATE_BYTES) return ELEV_ERR_LENGTH;

  memset(out, 0, sizeof(*out));
  out->txId       = in[1];
  out->seq        = in[2];
  out->floor      = in[3];
  out->posQ8      = (int16_t)elevGetU16(&in[4]);
  out->state      = in[6];
  out->confidence = in[7];
  return ELEV_OK;
}

// ---------------------------------------------------------------------------
// STATS encode / decode
// ---------------------------------------------------------------------------
static inline size_t elevStatsPack(uint8_t *out, size_t cap, const ElevStats *s) {
  if (out == NULL || s == NULL) return 0;
  if (cap < ELEV_STATS_BYTES) return 0;

  out[0] = ELEV_TAG_STATS;
  out[1] = s->txId;
  out[2] = s->seq;
  elevPutU16(&out[3],  s->batteryMv);
  elevPutU16(&out[5],  s->dist24hM);
  elevPutU32(&out[7],  s->distTotalM);
  elevPutU16(&out[11], s->pitchMm);
  out[13] = s->nFloors;
  elevPutU16(&out[14], s->trips);
  elevPutU16(&out[16], s->stops);
  elevPutU32(&out[18], s->uptimeS);
  out[22] = s->flags;
  out[23] = (uint8_t)s->tempC;
  return ELEV_STATS_BYTES;
}

static inline ElevDecodeStatus elevStatsUnpack(const uint8_t *in, size_t len,
                                               ElevStats *out) {
  if (in == NULL || out == NULL) return ELEV_ERR_SHORT;
  if (len < ELEV_STATS_BYTES) return ELEV_ERR_SHORT;
  if (in[0] != ELEV_TAG_STATS) return ELEV_ERR_TAG;
  if (len != ELEV_STATS_BYTES) return ELEV_ERR_LENGTH;

  memset(out, 0, sizeof(*out));
  out->txId       = in[1];
  out->seq        = in[2];
  out->batteryMv  = elevGetU16(&in[3]);
  out->dist24hM   = elevGetU16(&in[5]);
  out->distTotalM = elevGetU32(&in[7]);
  out->pitchMm    = elevGetU16(&in[11]);
  out->nFloors    = in[13];
  out->trips      = elevGetU16(&in[14]);
  out->stops      = elevGetU16(&in[16]);
  out->uptimeS    = elevGetU32(&in[18]);
  out->flags      = in[22];
  out->tempC      = (int8_t)in[23];
  return ELEV_OK;
}

// ---------------------------------------------------------------------------
// Accessors - the only place the scaling is undone
//
// Nothing outside this file divides by 256 or by 1000. A display that wants
// volts asks for volts; if the scaling ever changes it changes here.
// ---------------------------------------------------------------------------

// Signed offset from floor 1, in floors. Negative below floor 1.
static inline float elevStatePosFloors(const ElevState *s) {
  return (float)s->posQ8 / (float)ELEV_POS_UNITS_PER_FLOOR;
}

// The same position on the 1-based scale `floor` uses, so a display can
// interpolate its big digits between two labels without doing the +1 itself.
static inline float elevStatePosFloorIndex(const ElevState *s) {
  return 1.0f + elevStatePosFloors(s);
}

static inline bool elevStateMoving(const ElevState *s) {
  return (s->state & ELEV_STATE_MOVING) != 0;
}
static inline ElevDirection elevStateDirection(const ElevState *s) {
  return (ElevDirection)((s->state & ELEV_STATE_DIR_MASK) >> ELEV_STATE_DIR_SHIFT);
}
static inline bool elevStateModelReady(const ElevState *s) {
  return (s->state & ELEV_STATE_MODEL_READY) != 0;
}
static inline bool elevStateSensorErr(const ElevState *s) {
  return (s->state & ELEV_STATE_SENSOR_ERR) != 0;
}

// The one place the state byte is assembled. Direction values above DOWN are
// not representable in two bits and become idle rather than corrupting the
// neighbouring flags.
static inline uint8_t elevStateByte(bool moving, ElevDirection dir,
                                    bool modelReady, bool sensorErr) {
  uint8_t b = 0;
  if (moving) b |= ELEV_STATE_MOVING;
  b |= (uint8_t)(((uint8_t)dir << ELEV_STATE_DIR_SHIFT) & ELEV_STATE_DIR_MASK);
  if (modelReady) b |= ELEV_STATE_MODEL_READY;
  if (sensorErr)  b |= ELEV_STATE_SENSOR_ERR;
  return b;
}

static inline float elevStatsBatteryVolts(const ElevStats *s) {
  return (float)s->batteryMv / (float)ELEV_MV_PER_VOLT;
}
static inline float elevStatsPitchM(const ElevStats *s) {
  return (float)s->pitchMm / (float)ELEV_MM_PER_M;
}

static inline bool elevStatsSensorErr(const ElevStats *s) {
  return (s->flags & ELEV_FLAG_SENSOR_ERR) != 0;
}
static inline bool elevStatsModelReady(const ElevStats *s) {
  return (s->flags & ELEV_FLAG_MODEL_READY) != 0;
}
static inline bool elevStatsNvsRestored(const ElevStats *s) {
  return (s->flags & ELEV_FLAG_NVS_RESTORED) != 0;
}
static inline bool elevStatsLowBattery(const ElevStats *s) {
  return (s->flags & ELEV_FLAG_LOW_BATTERY) != 0;
}

static inline uint8_t elevStatsFlagsByte(bool sensorErr, bool modelReady,
                                         bool nvsRestored, bool lowBattery) {
  uint8_t b = 0;
  if (sensorErr)   b |= ELEV_FLAG_SENSOR_ERR;
  if (modelReady)  b |= ELEV_FLAG_MODEL_READY;
  if (nvsRestored) b |= ELEV_FLAG_NVS_RESTORED;
  if (lowBattery)  b |= ELEV_FLAG_LOW_BATTERY;
  return b;
}

// On-air length of whichever format this tag names, or 0 for a tag we do not
// speak. The mesh wrapper needs this to size a payload without decoding it.
static inline size_t elevPacketLength(uint8_t tag) {
  if (tag == ELEV_TAG_STATE) return ELEV_STATE_BYTES;
  if (tag == ELEV_TAG_STATS) return ELEV_STATS_BYTES;
  return 0;
}
