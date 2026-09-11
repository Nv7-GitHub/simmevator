//
// Host-side unit tests for the ESP-NOW flood envelope.
//
// Two of these are worth more than the rest. The hop test checks that the CRC
// is rewritten when hop is decremented, because the failure mode there is a
// mesh that works exactly one hop from the bridge. The wraparound tests check
// that the dedup ring survives origSeq rolling 65535 -> 0, because an ordering
// comparison there deadlocks every node for a full 65536-frame cycle and the
// bug would take a day and a half of running to show up on the wall.
//
// Raw bytes are asserted at named indices as well as through round trips: pack
// and unpack agreeing with each other is not the same as either agreeing with
// the format, and the bridge and the displays are separate builds.
//
//   pio test -e native
//
#include <unity.h>

#include <string.h>

#include "elev_packet.h"
#include "mesh_packet.h"

// A STATE packet as the bridge would have it off the radio.
static size_t buildStatePacket(uint8_t *out) {
  ElevState s;
  s.txId = 1; s.seq = 0x2A; s.floor = 7;
  s.posQ8 = 1600;
  s.state = elevStateByte(true, ELEV_DIR_UP, true, false);
  s.confidence = 212;
  return elevStatePack(out, ELEV_STATE_BYTES, &s);
}

static void fillPayload(uint8_t *p, uint8_t n, uint8_t start) {
  for (uint8_t i = 0; i < n; i++) p[i] = (uint8_t)(start + i);
}

// ---------------------------------------------------------------------------
// Byte layout
// ---------------------------------------------------------------------------
void test_frame_byte_layout(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0xA0);

  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 0x1234, ELEV_TAG_STATE, payload, 5);
  TEST_ASSERT_EQUAL_UINT32(15u, (uint32_t)n);  // 8 header + 5 payload + 2 CRC

  TEST_ASSERT_EQUAL_HEX8(0x1E, f[0]);   // magic 0x5E1E, little-endian
  TEST_ASSERT_EQUAL_HEX8(0x5E, f[1]);
  TEST_ASSERT_EQUAL_HEX8(0x01, f[2]);   // version
  TEST_ASSERT_EQUAL_HEX8(0x08, f[3]);   // hop
  TEST_ASSERT_EQUAL_HEX8(0x34, f[4]);   // origSeq low
  TEST_ASSERT_EQUAL_HEX8(0x12, f[5]);   // origSeq high
  TEST_ASSERT_EQUAL_HEX8(0xE0, f[6]);   // type
  TEST_ASSERT_EQUAL_HEX8(0x05, f[7]);   // len
  TEST_ASSERT_EQUAL_HEX8(0xA0, f[8]);   // payload
  TEST_ASSERT_EQUAL_HEX8(0xA4, f[12]);

  // The CRC covers everything before it, which is bytes 0..12 here.
  const uint16_t expect = crc16Ccitt(f, 13);
  TEST_ASSERT_EQUAL_HEX8((uint8_t)(expect & 0xFF), f[13]);
  TEST_ASSERT_EQUAL_HEX8((uint8_t)(expect >> 8), f[14]);
}

void test_frame_length_formula(void) {
  for (uint8_t n = 0; n <= MESH_MAX_PAYLOAD; n++) {
    TEST_ASSERT_EQUAL_UINT32(10u + n, (uint32_t)meshFrameLength(n));
  }
}

// ---------------------------------------------------------------------------
// Round trips
// ---------------------------------------------------------------------------
static void roundTrip(uint8_t hop, uint16_t origSeq, uint8_t type,
                      const uint8_t *payload, uint8_t len) {
  uint8_t f[MESH_MAX_FRAME_BYTES];
  MeshFrame r;
  size_t n = meshPack(f, sizeof(f), hop, origSeq, type, payload, len);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)meshFrameLength(len), (uint32_t)n);
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
  TEST_ASSERT_EQUAL_UINT8(MESH_VERSION, r.version);
  TEST_ASSERT_EQUAL_UINT8(hop, r.hop);
  TEST_ASSERT_EQUAL_UINT16(origSeq, r.origSeq);
  TEST_ASSERT_EQUAL_UINT8(type, r.type);
  TEST_ASSERT_EQUAL_UINT8(len, r.len);
  if (len > 0) TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, r.payload, len);
}

void test_roundtrip_empty_payload(void) {
  roundTrip(0, 0, 0, NULL, 0);
}

void test_roundtrip_max_payload_and_max_fields(void) {
  uint8_t payload[MESH_MAX_PAYLOAD];
  fillPayload(payload, MESH_MAX_PAYLOAD, 1);
  roundTrip(255, 65535, 0xFF, payload, MESH_MAX_PAYLOAD);
}

void test_roundtrip_typical_state_and_stats_bodies(void) {
  uint8_t payload[MESH_MAX_PAYLOAD];
  fillPayload(payload, 5, 0x10);
  roundTrip(MESH_HOP_LIMIT, 4242, ELEV_TAG_STATE, payload, 5);
  fillPayload(payload, 21, 0x20);
  roundTrip(MESH_HOP_LIMIT, 4243, ELEV_TAG_STATS, payload, 21);
}

// ---------------------------------------------------------------------------
// Wrapping a LoRa packet
// ---------------------------------------------------------------------------
// The wrapper carries the tag in `type` and drops txId and seq, so a STATE
// packet's 8 bytes become a 5 byte payload - and the bytes must be the ones
// from offset 3 onward, not from offset 0.
void test_wrap_lora_state_drops_the_first_three_bytes(void) {
  uint8_t lora[ELEV_STATE_BYTES];
  size_t loraLen = buildStatePacket(lora);

  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshWrapLora(f, sizeof(f), MESH_HOP_LIMIT, 99, lora, loraLen);
  TEST_ASSERT_EQUAL_UINT32(15u, (uint32_t)n);  // 8 header + 5 body + 2 CRC

  MeshFrame r;
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
  TEST_ASSERT_EQUAL_HEX8(ELEV_TAG_STATE, r.type);
  TEST_ASSERT_EQUAL_UINT8(5, r.len);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&lora[3], r.payload, 5);
  TEST_ASSERT_EQUAL_HEX8(lora[3], r.payload[0]);  // floor
}

void test_wrap_lora_rejects_a_body_less_frame(void) {
  uint8_t lora[3] = {ELEV_TAG_STATE, 1, 2};
  uint8_t f[MESH_MAX_FRAME_BYTES];
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshWrapLora(f, sizeof(f), 8, 1, lora, 3));
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshWrapLora(f, sizeof(f), 8, 1, lora, 0));
}

void test_pack_rejects_impossible_arguments(void) {
  uint8_t payload[MESH_MAX_PAYLOAD + 1];
  fillPayload(payload, MESH_MAX_PAYLOAD, 1);
  uint8_t f[MESH_MAX_FRAME_BYTES];

  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshPack(NULL, sizeof(f), 8, 1, 0xE0, payload, 4));
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshPack(f, sizeof(f), 8, 1, 0xE0, NULL, 4));
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshPack(f, sizeof(f), 8, 1, 0xE0, payload,
                                                  MESH_MAX_PAYLOAD + 1));
  // One byte short of what the frame needs.
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)meshPack(f, meshFrameLength(4) - 1, 8, 1,
                                                  0xE0, payload, 4));
}

// ---------------------------------------------------------------------------
// Rejections
// ---------------------------------------------------------------------------
void test_rejects_truncated_buffer(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x30);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 7, ELEV_TAG_STATE, payload, 5);

  MeshFrame r;
  // Below a minimum frame there is nothing to look at.
  for (size_t len = 0; len < MESH_HEADER_BYTES + MESH_CRC_BYTES; len++) {
    TEST_ASSERT_EQUAL(MESH_ERR_SHORT, meshUnpack(f, len, &r));
  }
  // Above it, the declared len is what disagrees.
  for (size_t len = MESH_HEADER_BYTES + MESH_CRC_BYTES; len < n; len++) {
    TEST_ASSERT_EQUAL(MESH_ERR_LENGTH, meshUnpack(f, len, &r));
  }
  TEST_ASSERT_EQUAL(MESH_ERR_LENGTH, meshUnpack(f, n + 1, &r));
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
}

void test_rejects_bad_magic(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x40);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 7, ELEV_TAG_STATE, payload, 5);

  MeshFrame r;
  f[0] ^= 0x01;
  TEST_ASSERT_EQUAL(MESH_ERR_MAGIC, meshUnpack(f, n, &r));
  f[0] ^= 0x01;
  f[1] ^= 0x01;
  TEST_ASSERT_EQUAL(MESH_ERR_MAGIC, meshUnpack(f, n, &r));
}

void test_rejects_bad_version(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x50);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 7, ELEV_TAG_STATE, payload, 5);

  MeshFrame r;
  f[2] = MESH_VERSION + 1;
  TEST_ASSERT_EQUAL(MESH_ERR_VERSION, meshUnpack(f, n, &r));
}

// A len field larger than the struct can hold must be rejected before anything
// is copied, or a hostile or garbled frame overruns MeshFrame::payload.
void test_rejects_oversize_declared_length(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x60);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 7, ELEV_TAG_STATE, payload, 5);

  MeshFrame r;
  f[MESH_OFF_LEN] = 255;
  TEST_ASSERT_EQUAL(MESH_ERR_LENGTH, meshUnpack(f, n, &r));
}

// Every single-bit flip anywhere in the frame must be caught. This is the
// property the CRC is there for, so it is worth testing exhaustively rather
// than on one hand-picked corruption.
void test_every_single_bit_corruption_is_caught(void) {
  uint8_t payload[21];
  fillPayload(payload, 21, 0x70);
  uint8_t good[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(good, sizeof(good), 8, 0x8001, ELEV_TAG_STATS, payload, 21);

  for (size_t byte = 0; byte < n; byte++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      uint8_t f[MESH_MAX_FRAME_BYTES];
      memcpy(f, good, n);
      f[byte] ^= (uint8_t)(1u << bit);

      MeshFrame r;
      TEST_ASSERT_NOT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
    }
  }
}

// ---------------------------------------------------------------------------
// Hop handling
// ---------------------------------------------------------------------------
// The decrement rewrites the CRC. Without that the frame still parses on the
// node that decremented it - it never re-reads its own buffer - and is
// discarded by every node downstream, which looks like a range problem.
void test_hop_decrement_rewrites_the_crc(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x80);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), MESH_HOP_LIMIT, 1234, ELEV_TAG_STATE, payload, 5);

  TEST_ASSERT_TRUE(meshDecrementHop(f, n));

  MeshFrame r;
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
  TEST_ASSERT_EQUAL_UINT8(MESH_HOP_LIMIT - 1, r.hop);
  TEST_ASSERT_EQUAL_UINT16(1234, r.origSeq);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, r.payload, 5);
}

// Eight hops means eight relays and then the frame stops. It must stop by
// reaching zero, never by wrapping a u8 round to 255 and flooding forever.
void test_hop_reaches_zero_and_stops(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0x90);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), MESH_HOP_LIMIT, 5, ELEV_TAG_STATE, payload, 5);

  MeshFrame r;
  for (int i = 1; i < MESH_HOP_LIMIT; i++) {
    TEST_ASSERT_TRUE(meshDecrementHop(f, n));
    TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
    TEST_ASSERT_EQUAL_UINT8(MESH_HOP_LIMIT - i, r.hop);
  }
  // The last decrement takes it to zero: still a valid frame, but not relayed.
  TEST_ASSERT_FALSE(meshDecrementHop(f, n));
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
  TEST_ASSERT_EQUAL_UINT8(0, r.hop);

  // A frame that arrives at zero is dropped, and the field does not wrap.
  TEST_ASSERT_FALSE(meshDecrementHop(f, n));
  TEST_ASSERT_EQUAL(MESH_OK, meshUnpack(f, n, &r));
  TEST_ASSERT_EQUAL_UINT8(0, r.hop);
}

void test_hop_decrement_rejects_a_malformed_frame(void) {
  uint8_t payload[5];
  fillPayload(payload, 5, 0xA0);
  uint8_t f[MESH_MAX_FRAME_BYTES];
  size_t n = meshPack(f, sizeof(f), 8, 1, ELEV_TAG_STATE, payload, 5);

  TEST_ASSERT_FALSE(meshDecrementHop(NULL, n));
  TEST_ASSERT_FALSE(meshDecrementHop(f, MESH_HEADER_BYTES));
  TEST_ASSERT_FALSE(meshDecrementHop(f, n - 1));
  TEST_ASSERT_EQUAL_UINT8(8, f[MESH_OFF_HOP]);  // untouched by any of those
}

// ---------------------------------------------------------------------------
// Dedup ring
// ---------------------------------------------------------------------------
// A freshly initialised ring is all zeroes. If membership were tested against
// all 32 slots rather than the live ones, origSeq 0 would look seen and every
// display would silently drop the first frame after a reboot.
void test_fresh_ring_has_seen_nothing_including_zero(void) {
  MeshDedup d;
  meshDedupInit(&d);
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 0));
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 1));
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 65535));

  TEST_ASSERT_FALSE(meshDedupSeenOrInsert(&d, 0));
  TEST_ASSERT_TRUE(meshDedupSeenOrInsert(&d, 0));
}

void test_insert_and_seen(void) {
  MeshDedup d;
  meshDedupInit(&d);
  for (uint16_t i = 100; i < 110; i++) meshDedupInsert(&d, i);
  for (uint16_t i = 100; i < 110; i++) TEST_ASSERT_TRUE(meshDedupSeen(&d, i));
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 99));
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 110));
}

// Exactly 32 deep: filling it keeps all 32, and the 33rd evicts the oldest.
void test_ring_evicts_oldest_after_32(void) {
  MeshDedup d;
  meshDedupInit(&d);
  for (uint16_t i = 0; i < MESH_DEDUP_DEPTH; i++) meshDedupInsert(&d, (uint16_t)(1000 + i));
  for (uint16_t i = 0; i < MESH_DEDUP_DEPTH; i++) {
    TEST_ASSERT_TRUE(meshDedupSeen(&d, (uint16_t)(1000 + i)));
  }

  meshDedupInsert(&d, 2000);
  TEST_ASSERT_FALSE(meshDedupSeen(&d, 1000));  // the oldest is gone
  TEST_ASSERT_TRUE(meshDedupSeen(&d, 1001));   // the next oldest is not
  TEST_ASSERT_TRUE(meshDedupSeen(&d, 2000));

  // A frame older than the window is relayed again. That is the price of a
  // fixed ring and it is preferable to the alternative, which is dropping new
  // frames forever after a wrap.
  meshDedupInsert(&d, 1000);
  TEST_ASSERT_TRUE(meshDedupSeen(&d, 1000));
}

// The whole point. origSeq rolls 65535 -> 0 after about a day and a half at
// the 2 s STATE cadence. A "greater than the highest seen" rule would reject
// every frame for the next 65536, which is the whole mesh dead for a day.
void test_origseq_wraparound_keeps_flooding(void) {
  MeshDedup d;
  meshDedupInit(&d);

  uint16_t seq = 65530;
  for (int i = 0; i < 12; i++) {
    TEST_ASSERT_FALSE(meshDedupSeenOrInsert(&d, seq));  // every one is new
    TEST_ASSERT_TRUE(meshDedupSeen(&d, seq));
    seq++;  // wraps to 0 after 65535
  }

  // The values from both sides of the wrap are in the same window, and both
  // are still recognised as duplicates.
  TEST_ASSERT_TRUE(meshDedupSeen(&d, 65535));
  TEST_ASSERT_TRUE(meshDedupSeen(&d, 0));
  TEST_ASSERT_TRUE(meshDedupSeenOrInsert(&d, 65535));
  TEST_ASSERT_TRUE(meshDedupSeenOrInsert(&d, 0));
  TEST_ASSERT_FALSE(meshDedupSeenOrInsert(&d, 6));  // and the next one is new
}

// The same thing said as a full cycle: 70000 consecutive origSeq values, which
// crosses the wrap, must every one of them be relayed exactly once.
void test_full_cycle_relays_every_frame_exactly_once(void) {
  MeshDedup d;
  meshDedupInit(&d);

  uint32_t relayed = 0;
  uint16_t seq = 60000;
  for (uint32_t i = 0; i < 70000; i++) {
    if (!meshDedupSeenOrInsert(&d, seq)) relayed++;
    seq++;
  }
  TEST_ASSERT_EQUAL_UINT32(70000u, relayed);
}

// A frame heard from several neighbours at once is relayed once, no matter how
// many copies arrive - which is what stops a flood turning into a broadcast
// storm across ten displays.
void test_duplicate_copies_are_relayed_once(void) {
  MeshDedup d;
  meshDedupInit(&d);

  uint32_t relayed = 0;
  for (int copy = 0; copy < 6; copy++) {
    if (!meshDedupSeenOrInsert(&d, 4242)) relayed++;
  }
  TEST_ASSERT_EQUAL_UINT32(1u, relayed);
}

void test_decode_status_names_exist(void) {
  TEST_ASSERT_EQUAL_STRING("ok", meshDecodeStatusName(MESH_OK));
  TEST_ASSERT_EQUAL_STRING("too short", meshDecodeStatusName(MESH_ERR_SHORT));
  TEST_ASSERT_EQUAL_STRING("bad magic", meshDecodeStatusName(MESH_ERR_MAGIC));
  TEST_ASSERT_EQUAL_STRING("bad version", meshDecodeStatusName(MESH_ERR_VERSION));
  TEST_ASSERT_EQUAL_STRING("length mismatch", meshDecodeStatusName(MESH_ERR_LENGTH));
  TEST_ASSERT_EQUAL_STRING("CRC mismatch", meshDecodeStatusName(MESH_ERR_CRC));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_frame_byte_layout);
  RUN_TEST(test_frame_length_formula);
  RUN_TEST(test_roundtrip_empty_payload);
  RUN_TEST(test_roundtrip_max_payload_and_max_fields);
  RUN_TEST(test_roundtrip_typical_state_and_stats_bodies);
  RUN_TEST(test_wrap_lora_state_drops_the_first_three_bytes);
  RUN_TEST(test_wrap_lora_rejects_a_body_less_frame);
  RUN_TEST(test_pack_rejects_impossible_arguments);
  RUN_TEST(test_rejects_truncated_buffer);
  RUN_TEST(test_rejects_bad_magic);
  RUN_TEST(test_rejects_bad_version);
  RUN_TEST(test_rejects_oversize_declared_length);
  RUN_TEST(test_every_single_bit_corruption_is_caught);
  RUN_TEST(test_hop_decrement_rewrites_the_crc);
  RUN_TEST(test_hop_reaches_zero_and_stops);
  RUN_TEST(test_hop_decrement_rejects_a_malformed_frame);
  RUN_TEST(test_fresh_ring_has_seen_nothing_including_zero);
  RUN_TEST(test_insert_and_seen);
  RUN_TEST(test_ring_evicts_oldest_after_32);
  RUN_TEST(test_origseq_wraparound_keeps_flooding);
  RUN_TEST(test_full_cycle_relays_every_frame_exactly_once);
  RUN_TEST(test_duplicate_copies_are_relayed_once);
  RUN_TEST(test_decode_status_names_exist);
  return UNITY_END();
}
