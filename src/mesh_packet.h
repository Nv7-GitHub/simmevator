#pragma once
//
// The ESP-NOW flood envelope: bridge -> displays -> displays.
//
// The bridge hears a LoRa packet, strips the three bytes the LoRa leg needed
// for itself (tag, txId, seq), and re-sends the body inside this wrapper. Every
// node - the bridge and all ten displays - runs the identical relay rule, so
// there is no routing table and no node that is special apart from the one
// that mints origSeq.
//
//   off  size  field
//   0    u16   magic     0x5E1E
//   2    u8    version   1
//   3    u8    hop       remaining hops, dropped at 0
//   4    u16   origSeq   monotonic, minted by the bridge
//   6    u8    type      0xE0 or 0xE1, the LoRa tag
//   7    u8    len       payload length
//   8..  ...   payload   the LoRa packet body from offset 3 onward
//   +2   u16   crc16     CCITT over everything before it
//
// Pure C++ over <stdint.h>, no Arduino.h and no allocation, so the whole flood
// rule can be exercised on the host.
//
// ---------------------------------------------------------------------------
// Why this format carries a CRC when the LoRa formats do not
// ---------------------------------------------------------------------------
// elev_packet.h leans on the SX1262's hardware CRC. Here the frame is rewritten
// at every hop - `hop` is decremented in place - so a corrupted frame can be
// re-transmitted with a freshly computed FCS by a node that never noticed, and
// the 802.11 FCS only ever covers one link. The CRC-16 below travels with the
// payload end to end. It is also what catches a frame from an unrelated ESP-NOW
// device that happens to open with the right two bytes.
//
// ---------------------------------------------------------------------------
// Why dedup is by set membership and never by comparison
// ---------------------------------------------------------------------------
// origSeq is u16 and *will* wrap: at one STATE every 2 s the bridge reaches
// 65535 in about a day and a half. The obvious rule - relay only if origSeq is
// greater than the highest seen - deadlocks the entire mesh for a full cycle at
// that moment, because every subsequent frame looks old. The ring below stores
// the last MESH_DEDUP_DEPTH values and tests equality, so the wrap is not an
// event it can observe. Nothing in this file compares two origSeq values with
// < or >, and nothing added to it should.
//

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc16.h"

// ---------------------------------------------------------------------------
// Envelope constants
// ---------------------------------------------------------------------------
#define MESH_MAGIC   0x5E1E
#define MESH_VERSION 1

#define MESH_HEADER_BYTES 8
#define MESH_CRC_BYTES    2

#define MESH_OFF_MAGIC    0
#define MESH_OFF_VERSION  2
#define MESH_OFF_HOP      3
#define MESH_OFF_ORIGSEQ  4
#define MESH_OFF_TYPE     6
#define MESH_OFF_LEN      7
#define MESH_OFF_PAYLOAD  8

// The largest body today is STATS: 24 bytes on the air less the 3 the wrapper
// replaces, so 21. 32 leaves room for a field to be added to a LoRa format
// without this one becoming the reason it cannot be, and 42 bytes total is far
// inside the 250 an ESP-NOW frame carries.
#define MESH_MAX_PAYLOAD 32

#define MESH_MAX_FRAME_BYTES (MESH_HEADER_BYTES + MESH_MAX_PAYLOAD + MESH_CRC_BYTES)

// Five floors is the furthest any display sits from the bridge, so 8 leaves
// slack for a path that has to detour around a dead node.
#define MESH_HOP_LIMIT 8

// 32 is roughly a minute of STATE traffic at the 2 s cadence - long enough
// that a frame arriving late by a few relay jitters is still recognised, short
// enough to stay a linear scan of one cache line's worth of u16.
#define MESH_DEDUP_DEPTH 32

// ---------------------------------------------------------------------------
// Decode results
// ---------------------------------------------------------------------------
enum MeshDecodeStatus {
  MESH_OK = 0,
  MESH_ERR_SHORT,    // buffer too small to hold even an empty frame
  MESH_ERR_MAGIC,    // not one of ours
  MESH_ERR_VERSION,  // a node running a different build of this format
  MESH_ERR_LENGTH,   // len disagrees with the frame, or overruns the payload
  MESH_ERR_CRC       // corrupted in flight or relayed without a CRC rewrite
};

static inline const char *meshDecodeStatusName(MeshDecodeStatus s) {
  switch (s) {
    case MESH_OK:          return "ok";
    case MESH_ERR_SHORT:   return "too short";
    case MESH_ERR_MAGIC:   return "bad magic";
    case MESH_ERR_VERSION: return "bad version";
    case MESH_ERR_LENGTH:  return "length mismatch";
    case MESH_ERR_CRC:     return "CRC mismatch";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Little-endian primitives - see the same note in elev_packet.h
// ---------------------------------------------------------------------------
static inline void meshPutU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}
static inline uint16_t meshGetU16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// Total wire length of a frame carrying n payload bytes.
static inline size_t meshFrameLength(uint8_t payloadLen) {
  return (size_t)MESH_HEADER_BYTES + (size_t)payloadLen + MESH_CRC_BYTES;
}

// ---------------------------------------------------------------------------
// In-memory representation
// ---------------------------------------------------------------------------
struct MeshFrame {
  uint8_t  version;
  uint8_t  hop;
  uint16_t origSeq;
  uint8_t  type;
  uint8_t  len;
  uint8_t  payload[MESH_MAX_PAYLOAD];
};

// ---------------------------------------------------------------------------
// Encode
// ---------------------------------------------------------------------------
// Returns the number of bytes written, or 0 if the arguments are unusable.
static inline size_t meshPack(uint8_t *out, size_t cap, uint8_t hop,
                              uint16_t origSeq, uint8_t type,
                              const uint8_t *payload, uint8_t payloadLen) {
  if (out == NULL) return 0;
  if (payload == NULL && payloadLen > 0) return 0;
  if (payloadLen > MESH_MAX_PAYLOAD) return 0;
  if (cap < meshFrameLength(payloadLen)) return 0;

  meshPutU16(&out[MESH_OFF_MAGIC], MESH_MAGIC);
  out[MESH_OFF_VERSION] = MESH_VERSION;
  out[MESH_OFF_HOP]     = hop;
  meshPutU16(&out[MESH_OFF_ORIGSEQ], origSeq);
  out[MESH_OFF_TYPE]    = type;
  out[MESH_OFF_LEN]     = payloadLen;
  for (uint8_t i = 0; i < payloadLen; i++) out[MESH_OFF_PAYLOAD + i] = payload[i];

  const size_t crcOffset = MESH_HEADER_BYTES + (size_t)payloadLen;
  meshPutU16(&out[crcOffset], crc16Ccitt(out, crcOffset));
  return meshFrameLength(payloadLen);
}

// Wraps a LoRa packet as the bridge receives it: the tag becomes `type` and
// the txId and seq are dropped, because the mesh has its own identity in
// origSeq and only one transmitter exists. Returns 0 if the LoRa frame is too
// short to have a body at all.
static inline size_t meshWrapLora(uint8_t *out, size_t cap, uint8_t hop,
                                  uint16_t origSeq, const uint8_t *lora,
                                  size_t loraLen) {
  if (lora == NULL) return 0;
  if (loraLen <= 3 || loraLen - 3 > MESH_MAX_PAYLOAD) return 0;
  return meshPack(out, cap, hop, origSeq, lora[0], &lora[3], (uint8_t)(loraLen - 3));
}

// ---------------------------------------------------------------------------
// Decode
// ---------------------------------------------------------------------------
static inline MeshDecodeStatus meshUnpack(const uint8_t *in, size_t len,
                                          MeshFrame *out) {
  if (in == NULL || out == NULL) return MESH_ERR_SHORT;
  if (len < MESH_HEADER_BYTES + MESH_CRC_BYTES) return MESH_ERR_SHORT;
  if (meshGetU16(&in[MESH_OFF_MAGIC]) != MESH_MAGIC) return MESH_ERR_MAGIC;
  if (in[MESH_OFF_VERSION] != MESH_VERSION) return MESH_ERR_VERSION;

  const uint8_t payloadLen = in[MESH_OFF_LEN];
  if (payloadLen > MESH_MAX_PAYLOAD) return MESH_ERR_LENGTH;
  if (len != meshFrameLength(payloadLen)) return MESH_ERR_LENGTH;

  const size_t crcOffset = MESH_HEADER_BYTES + (size_t)payloadLen;
  if (meshGetU16(&in[crcOffset]) != crc16Ccitt(in, crcOffset)) return MESH_ERR_CRC;

  memset(out, 0, sizeof(*out));
  out->version = in[MESH_OFF_VERSION];
  out->hop     = in[MESH_OFF_HOP];
  out->origSeq = meshGetU16(&in[MESH_OFF_ORIGSEQ]);
  out->type    = in[MESH_OFF_TYPE];
  out->len     = payloadLen;
  for (uint8_t i = 0; i < payloadLen; i++) out->payload[i] = in[MESH_OFF_PAYLOAD + i];
  return MESH_OK;
}

// ---------------------------------------------------------------------------
// Relay
// ---------------------------------------------------------------------------
// Decrements the hop count of a frame in place and rewrites its CRC, which is
// the part that is easy to forget: the CRC covers the header, and the header
// contains the field just changed, so a relay that skips the rewrite sends a
// frame every downstream node discards as corrupt - a mesh that works one hop
// from the bridge and nowhere else.
//
// Returns true if the frame still has hops left and should go out. A frame
// arriving with hop already 0 is dropped rather than wrapping to 255.
static inline bool meshDecrementHop(uint8_t *frame, size_t len) {
  if (frame == NULL) return false;
  if (len < MESH_HEADER_BYTES + MESH_CRC_BYTES) return false;

  const uint8_t payloadLen = frame[MESH_OFF_LEN];
  if (payloadLen > MESH_MAX_PAYLOAD) return false;
  if (len != meshFrameLength(payloadLen)) return false;
  if (frame[MESH_OFF_HOP] == 0) return false;

  frame[MESH_OFF_HOP]--;

  const size_t crcOffset = MESH_HEADER_BYTES + (size_t)payloadLen;
  meshPutU16(&frame[crcOffset], crc16Ccitt(frame, crcOffset));
  return frame[MESH_OFF_HOP] > 0;
}

// ---------------------------------------------------------------------------
// Dedup ring
//
// A fixed MESH_DEDUP_DEPTH-entry ring of origSeq values, oldest evicted first.
// Membership is equality against every live entry, never an ordering test -
// see the wraparound note at the top of this file.
//
// `count` exists so that a freshly initialised ring does not treat its 32
// zeroed slots as having already seen origSeq 0, which would silently drop the
// first frame after every display reboot.
// ---------------------------------------------------------------------------
struct MeshDedup {
  uint16_t seq[MESH_DEDUP_DEPTH];
  uint8_t  count;  // live entries, saturating at MESH_DEDUP_DEPTH
  uint8_t  next;   // where the next insert lands
};

static inline void meshDedupInit(MeshDedup *d) {
  memset(d, 0, sizeof(*d));
}

static inline bool meshDedupSeen(const MeshDedup *d, uint16_t origSeq) {
  for (uint8_t i = 0; i < d->count; i++) {
    if (d->seq[i] == origSeq) return true;
  }
  return false;
}

static inline void meshDedupInsert(MeshDedup *d, uint16_t origSeq) {
  d->seq[d->next] = origSeq;
  d->next = (uint8_t)((d->next + 1) % MESH_DEDUP_DEPTH);
  if (d->count < MESH_DEDUP_DEPTH) d->count++;
}

// The relay decision in one call: true means this frame is new and has now
// been recorded, so the caller should jitter and rebroadcast it exactly once.
static inline bool meshDedupSeenOrInsert(MeshDedup *d, uint16_t origSeq) {
  if (meshDedupSeen(d, origSeq)) return true;
  meshDedupInsert(d, origSeq);
  return false;
}
