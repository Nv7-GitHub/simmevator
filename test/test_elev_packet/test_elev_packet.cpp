//
// Host-side unit tests for the LoRa wire format.
//
// The failure this file exists to catch is an offset that is wrong in pack and
// unpack in the same way: a round-trip test passes happily while the bridge and
// the displays disagree with each other about where batteryMv lives. So the
// tests below assert on raw bytes at named indices as well as on round trips,
// and the indices are written out as literals rather than derived from the same
// constants the implementation uses.
//
//   pio test -e native
//
#include <unity.h>

#include <string.h>

#include "elev_packet.h"

// ---------------------------------------------------------------------------
// STATE - byte layout
// ---------------------------------------------------------------------------
void test_state_byte_layout(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  s.txId       = 0x11;
  s.seq        = 0x22;
  s.floor      = 7;
  s.posQ8      = 1600;  // floor 7.25 relative to floor 1 -> 0x0640
  s.state      = elevStateByte(true, ELEV_DIR_UP, true, false);
  s.confidence = 0xC8;

  uint8_t buf[ELEV_STATE_BYTES];
  TEST_ASSERT_EQUAL_UINT32(8u, (uint32_t)elevStatePack(buf, sizeof(buf), &s));

  TEST_ASSERT_EQUAL_HEX8(0xE0, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x11, buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x22, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0x07, buf[3]);
  TEST_ASSERT_EQUAL_HEX8(0x40, buf[4]);  // posQ8 low byte
  TEST_ASSERT_EQUAL_HEX8(0x06, buf[5]);  // posQ8 high byte
  TEST_ASSERT_EQUAL_HEX8(0x0B, buf[6]);  // moving | up | modelReady
  TEST_ASSERT_EQUAL_HEX8(0xC8, buf[7]);
}

// A negative posQ8 is the case where a byte-order or sign mistake is invisible
// to a round trip: the same wrong bytes come back the same wrong way.
void test_state_negative_pos_is_little_endian_twos_complement(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  s.posQ8 = -1234;  // 0xFB2E

  uint8_t buf[ELEV_STATE_BYTES];
  elevStatePack(buf, sizeof(buf), &s);
  TEST_ASSERT_EQUAL_HEX8(0x2E, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0xFB, buf[5]);

  ElevState r;
  TEST_ASSERT_EQUAL(ELEV_OK, elevStateUnpack(buf, ELEV_STATE_BYTES, &r));
  TEST_ASSERT_EQUAL_INT16(-1234, r.posQ8);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -4.8203f, elevStatePosFloors(&r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -3.8203f, elevStatePosFloorIndex(&r));
}

// ---------------------------------------------------------------------------
// STATE - round trips at the limits
// ---------------------------------------------------------------------------
static void stateRoundTrip(const ElevState *s) {
  uint8_t buf[ELEV_STATE_BYTES];
  ElevState r;
  TEST_ASSERT_EQUAL_UINT32(ELEV_STATE_BYTES, (uint32_t)elevStatePack(buf, sizeof(buf), s));
  TEST_ASSERT_EQUAL(ELEV_OK, elevStateUnpack(buf, ELEV_STATE_BYTES, &r));
  TEST_ASSERT_EQUAL_UINT8(s->txId, r.txId);
  TEST_ASSERT_EQUAL_UINT8(s->seq, r.seq);
  TEST_ASSERT_EQUAL_UINT8(s->floor, r.floor);
  TEST_ASSERT_EQUAL_INT16(s->posQ8, r.posQ8);
  TEST_ASSERT_EQUAL_HEX8(s->state, r.state);
  TEST_ASSERT_EQUAL_UINT8(s->confidence, r.confidence);
}

void test_state_roundtrip_min(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  s.posQ8 = -32768;
  stateRoundTrip(&s);
}

void test_state_roundtrip_max(void) {
  ElevState s;
  s.txId = 255; s.seq = 255; s.floor = 255;
  s.posQ8 = 32767;
  s.state = 0xFF; s.confidence = 255;
  stateRoundTrip(&s);
}

// The car at floor 4, halfway to 5, going up with the model settled.
void test_state_roundtrip_typical(void) {
  ElevState s;
  s.txId = 1; s.seq = 130; s.floor = 4;
  s.posQ8 = 3 * 256 + 128;
  s.state = elevStateByte(true, ELEV_DIR_UP, true, false);
  s.confidence = 212;
  stateRoundTrip(&s);

  uint8_t buf[ELEV_STATE_BYTES];
  ElevState r;
  elevStatePack(buf, sizeof(buf), &s);
  elevStateUnpack(buf, ELEV_STATE_BYTES, &r);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, elevStatePosFloors(&r));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.5f, elevStatePosFloorIndex(&r));
}

// floor 0 means the model is not ready, not "the ground floor".
void test_state_model_not_ready_is_floor_zero(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  s.state = elevStateByte(false, ELEV_DIR_IDLE, false, false);
  stateRoundTrip(&s);

  uint8_t buf[ELEV_STATE_BYTES];
  ElevState r;
  elevStatePack(buf, sizeof(buf), &s);
  elevStateUnpack(buf, ELEV_STATE_BYTES, &r);
  TEST_ASSERT_EQUAL_UINT8(0, r.floor);
  TEST_ASSERT_FALSE(elevStateModelReady(&r));
}

// ---------------------------------------------------------------------------
// STATE - the bit field
// ---------------------------------------------------------------------------
void test_state_bits_are_at_the_documented_positions(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));

  s.state = elevStateByte(true, ELEV_DIR_IDLE, false, false);
  TEST_ASSERT_EQUAL_HEX8(0x01, s.state);
  s.state = elevStateByte(false, ELEV_DIR_UP, false, false);
  TEST_ASSERT_EQUAL_HEX8(0x02, s.state);
  s.state = elevStateByte(false, ELEV_DIR_DOWN, false, false);
  TEST_ASSERT_EQUAL_HEX8(0x04, s.state);
  s.state = elevStateByte(false, ELEV_DIR_IDLE, true, false);
  TEST_ASSERT_EQUAL_HEX8(0x08, s.state);
  s.state = elevStateByte(false, ELEV_DIR_IDLE, false, true);
  TEST_ASSERT_EQUAL_HEX8(0x10, s.state);
}

void test_state_accessors_agree_with_the_builder(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  const ElevDirection dirs[3] = {ELEV_DIR_IDLE, ELEV_DIR_UP, ELEV_DIR_DOWN};

  for (int d = 0; d < 3; d++) {
    for (int m = 0; m < 2; m++) {
      for (int k = 0; k < 2; k++) {
        for (int e = 0; e < 2; e++) {
          s.state = elevStateByte(m != 0, dirs[d], k != 0, e != 0);
          TEST_ASSERT_EQUAL(dirs[d], elevStateDirection(&s));
          TEST_ASSERT_EQUAL(m != 0, elevStateMoving(&s));
          TEST_ASSERT_EQUAL(k != 0, elevStateModelReady(&s));
          TEST_ASSERT_EQUAL(e != 0, elevStateSensorErr(&s));
        }
      }
    }
  }
}

// The direction field is two bits at an offset, so it is the one that can
// bleed into its neighbours if the shift and the mask disagree.
void test_direction_never_disturbs_the_neighbouring_flags(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  s.state = elevStateByte(true, ELEV_DIR_DOWN, true, true);
  TEST_ASSERT_EQUAL_HEX8(0x1D, s.state);
  TEST_ASSERT_TRUE(elevStateMoving(&s));
  TEST_ASSERT_TRUE(elevStateModelReady(&s));
  TEST_ASSERT_TRUE(elevStateSensorErr(&s));
  TEST_ASSERT_EQUAL(ELEV_DIR_DOWN, elevStateDirection(&s));
}

// ---------------------------------------------------------------------------
// STATE - rejections
// ---------------------------------------------------------------------------
void test_state_rejects_truncated_and_overlong(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  uint8_t buf[ELEV_STATE_BYTES + 1];
  elevStatePack(buf, ELEV_STATE_BYTES, &s);
  buf[ELEV_STATE_BYTES] = 0;

  ElevState r;
  for (size_t len = 0; len < ELEV_STATE_BYTES; len++) {
    TEST_ASSERT_EQUAL(ELEV_ERR_SHORT, elevStateUnpack(buf, len, &r));
  }
  TEST_ASSERT_EQUAL(ELEV_ERR_LENGTH, elevStateUnpack(buf, ELEV_STATE_BYTES + 1, &r));
}

void test_state_rejects_wrong_tag(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  uint8_t buf[ELEV_STATE_BYTES];
  elevStatePack(buf, sizeof(buf), &s);
  buf[0] = ELEV_TAG_STATS;
  ElevState r;
  TEST_ASSERT_EQUAL(ELEV_ERR_TAG, elevStateUnpack(buf, ELEV_STATE_BYTES, &r));
}

void test_state_pack_rejects_a_short_buffer(void) {
  ElevState s;
  memset(&s, 0, sizeof(s));
  uint8_t buf[ELEV_STATE_BYTES];
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)elevStatePack(buf, ELEV_STATE_BYTES - 1, &s));
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)elevStatePack(NULL, sizeof(buf), &s));
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)elevStatePack(buf, sizeof(buf), NULL));
}

// ---------------------------------------------------------------------------
// STATS - byte layout
// ---------------------------------------------------------------------------
static void fillTypicalStats(ElevStats *s) {
  s->txId       = 0x11;
  s->seq        = 0x22;
  s->batteryMv  = 12750;    // 0x31CE
  s->dist24hM   = 3409;     // 0x0D51, the reference capture's 3 h distance
  s->distTotalM = 1234567;  // 0x0012D687
  s->pitchMm    = 2871;     // 0x0B37, the learned pitch
  s->nFloors    = 10;
  s->trips      = 356;      // 0x0164
  s->stops      = 256;      // 0x0100
  s->uptimeS    = 10943;    // 0x00002ABF
  s->flags      = elevStatsFlagsByte(false, true, true, false);
  s->tempC      = -5;       // 0xFB
}

void test_stats_byte_layout(void) {
  ElevStats s;
  fillTypicalStats(&s);

  uint8_t buf[ELEV_STATS_BYTES];
  TEST_ASSERT_EQUAL_UINT32(24u, (uint32_t)elevStatsPack(buf, sizeof(buf), &s));

  TEST_ASSERT_EQUAL_HEX8(0xE1, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0x11, buf[1]);
  TEST_ASSERT_EQUAL_HEX8(0x22, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(0xCE, buf[3]);   // batteryMv
  TEST_ASSERT_EQUAL_HEX8(0x31, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x51, buf[5]);   // dist24hM
  TEST_ASSERT_EQUAL_HEX8(0x0D, buf[6]);
  TEST_ASSERT_EQUAL_HEX8(0x87, buf[7]);   // distTotalM
  TEST_ASSERT_EQUAL_HEX8(0xD6, buf[8]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buf[9]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[10]);
  TEST_ASSERT_EQUAL_HEX8(0x37, buf[11]);  // pitchMm
  TEST_ASSERT_EQUAL_HEX8(0x0B, buf[12]);
  TEST_ASSERT_EQUAL_HEX8(0x0A, buf[13]);  // nFloors
  TEST_ASSERT_EQUAL_HEX8(0x64, buf[14]);  // trips
  TEST_ASSERT_EQUAL_HEX8(0x01, buf[15]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[16]);  // stops
  TEST_ASSERT_EQUAL_HEX8(0x01, buf[17]);
  TEST_ASSERT_EQUAL_HEX8(0xBF, buf[18]);  // uptimeS
  TEST_ASSERT_EQUAL_HEX8(0x2A, buf[19]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[20]);
  TEST_ASSERT_EQUAL_HEX8(0x00, buf[21]);
  TEST_ASSERT_EQUAL_HEX8(0x06, buf[22]);  // modelReady | nvsRestored
  TEST_ASSERT_EQUAL_HEX8(0xFB, buf[23]);  // tempC = -5
}

// ---------------------------------------------------------------------------
// STATS - round trips at the limits
// ---------------------------------------------------------------------------
static void statsRoundTrip(const ElevStats *s) {
  uint8_t buf[ELEV_STATS_BYTES];
  ElevStats r;
  TEST_ASSERT_EQUAL_UINT32(ELEV_STATS_BYTES, (uint32_t)elevStatsPack(buf, sizeof(buf), s));
  TEST_ASSERT_EQUAL(ELEV_OK, elevStatsUnpack(buf, ELEV_STATS_BYTES, &r));
  TEST_ASSERT_EQUAL_UINT8(s->txId, r.txId);
  TEST_ASSERT_EQUAL_UINT8(s->seq, r.seq);
  TEST_ASSERT_EQUAL_UINT16(s->batteryMv, r.batteryMv);
  TEST_ASSERT_EQUAL_UINT16(s->dist24hM, r.dist24hM);
  TEST_ASSERT_EQUAL_UINT32(s->distTotalM, r.distTotalM);
  TEST_ASSERT_EQUAL_UINT16(s->pitchMm, r.pitchMm);
  TEST_ASSERT_EQUAL_UINT8(s->nFloors, r.nFloors);
  TEST_ASSERT_EQUAL_UINT16(s->trips, r.trips);
  TEST_ASSERT_EQUAL_UINT16(s->stops, r.stops);
  TEST_ASSERT_EQUAL_UINT32(s->uptimeS, r.uptimeS);
  TEST_ASSERT_EQUAL_HEX8(s->flags, r.flags);
  TEST_ASSERT_EQUAL_INT8(s->tempC, r.tempC);
}

void test_stats_roundtrip_min(void) {
  ElevStats s;
  memset(&s, 0, sizeof(s));
  s.tempC = -128;
  statsRoundTrip(&s);
}

void test_stats_roundtrip_max(void) {
  ElevStats s;
  s.txId = 255; s.seq = 255;
  s.batteryMv = 65535; s.dist24hM = 65535; s.distTotalM = 0xFFFFFFFFu;
  s.pitchMm = 65535; s.nFloors = 255; s.trips = 65535; s.stops = 65535;
  s.uptimeS = 0xFFFFFFFFu; s.flags = 0xFF; s.tempC = 127;
  statsRoundTrip(&s);
}

void test_stats_roundtrip_typical(void) {
  ElevStats s;
  fillTypicalStats(&s);
  statsRoundTrip(&s);
}

// distTotalM is the field a u16 would have overflowed: the reference capture
// alone is 3409 m in three hours, so a year is well past 65535.
void test_stats_lifetime_odometer_survives_a_year(void) {
  ElevStats s;
  fillTypicalStats(&s);
  s.distTotalM = 10000000u;
  statsRoundTrip(&s);
}

// ---------------------------------------------------------------------------
// STATS - scaling accessors
// ---------------------------------------------------------------------------
void test_stats_accessors_undo_the_scaling(void) {
  ElevStats s;
  fillTypicalStats(&s);
  uint8_t buf[ELEV_STATS_BYTES];
  ElevStats r;
  elevStatsPack(buf, sizeof(buf), &s);
  elevStatsUnpack(buf, ELEV_STATS_BYTES, &r);

  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 12.750f, elevStatsBatteryVolts(&r));
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 2.871f, elevStatsPitchM(&r));
}

// The 12.0 V warn threshold from the spec, either side of it.
void test_stats_battery_thresholds(void) {
  ElevStats s;
  fillTypicalStats(&s);

  s.batteryMv = 13300; s.flags = elevStatsFlagsByte(false, true, false, false);
  statsRoundTrip(&s);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 13.300f, elevStatsBatteryVolts(&s));
  TEST_ASSERT_FALSE(elevStatsLowBattery(&s));

  s.batteryMv = 11200; s.flags = elevStatsFlagsByte(false, true, false, true);
  statsRoundTrip(&s);
  TEST_ASSERT_FLOAT_WITHIN(0.0005f, 11.200f, elevStatsBatteryVolts(&s));
  TEST_ASSERT_TRUE(elevStatsLowBattery(&s));
}

void test_stats_flag_bits_are_at_the_documented_positions(void) {
  TEST_ASSERT_EQUAL_HEX8(0x01, elevStatsFlagsByte(true, false, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x02, elevStatsFlagsByte(false, true, false, false));
  TEST_ASSERT_EQUAL_HEX8(0x04, elevStatsFlagsByte(false, false, true, false));
  TEST_ASSERT_EQUAL_HEX8(0x08, elevStatsFlagsByte(false, false, false, true));

  ElevStats s;
  memset(&s, 0, sizeof(s));
  s.flags = elevStatsFlagsByte(true, true, true, true);
  TEST_ASSERT_TRUE(elevStatsSensorErr(&s));
  TEST_ASSERT_TRUE(elevStatsModelReady(&s));
  TEST_ASSERT_TRUE(elevStatsNvsRestored(&s));
  TEST_ASSERT_TRUE(elevStatsLowBattery(&s));
}

// ---------------------------------------------------------------------------
// STATS - rejections
// ---------------------------------------------------------------------------
void test_stats_rejects_truncated_and_overlong(void) {
  ElevStats s;
  fillTypicalStats(&s);
  uint8_t buf[ELEV_STATS_BYTES + 1];
  elevStatsPack(buf, ELEV_STATS_BYTES, &s);
  buf[ELEV_STATS_BYTES] = 0;

  ElevStats r;
  for (size_t len = 0; len < ELEV_STATS_BYTES; len++) {
    TEST_ASSERT_EQUAL(ELEV_ERR_SHORT, elevStatsUnpack(buf, len, &r));
  }
  TEST_ASSERT_EQUAL(ELEV_ERR_LENGTH, elevStatsUnpack(buf, ELEV_STATS_BYTES + 1, &r));
}

void test_stats_rejects_wrong_tag(void) {
  ElevStats s;
  fillTypicalStats(&s);
  uint8_t buf[ELEV_STATS_BYTES];
  elevStatsPack(buf, sizeof(buf), &s);
  buf[0] = 0x00;
  ElevStats r;
  TEST_ASSERT_EQUAL(ELEV_ERR_TAG, elevStatsUnpack(buf, ELEV_STATS_BYTES, &r));
}

void test_stats_pack_rejects_a_short_buffer(void) {
  ElevStats s;
  fillTypicalStats(&s);
  uint8_t buf[ELEV_STATS_BYTES];
  TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)elevStatsPack(buf, ELEV_STATS_BYTES - 1, &s));
}

// ---------------------------------------------------------------------------
// The two formats must never be mistaken for one another
// ---------------------------------------------------------------------------
void test_formats_do_not_decode_as_each_other(void) {
  ElevState st;
  ElevStats sa;
  memset(&st, 0, sizeof(st));
  fillTypicalStats(&sa);

  uint8_t a[ELEV_STATE_BYTES];
  uint8_t b[ELEV_STATS_BYTES];
  elevStatePack(a, sizeof(a), &st);
  elevStatsPack(b, sizeof(b), &sa);

  ElevState rs;
  ElevStats rt;
  TEST_ASSERT_NOT_EQUAL(ELEV_OK, elevStatsUnpack(a, ELEV_STATE_BYTES, &rt));
  TEST_ASSERT_NOT_EQUAL(ELEV_OK, elevStateUnpack(b, ELEV_STATS_BYTES, &rs));
}

void test_packet_length_from_tag(void) {
  TEST_ASSERT_EQUAL_UINT32(8u,  (uint32_t)elevPacketLength(ELEV_TAG_STATE));
  TEST_ASSERT_EQUAL_UINT32(24u, (uint32_t)elevPacketLength(ELEV_TAG_STATS));
  TEST_ASSERT_EQUAL_UINT32(0u,  (uint32_t)elevPacketLength(0xB0));
}

void test_decode_status_names_exist(void) {
  TEST_ASSERT_EQUAL_STRING("ok", elevDecodeStatusName(ELEV_OK));
  TEST_ASSERT_EQUAL_STRING("too short", elevDecodeStatusName(ELEV_ERR_SHORT));
  TEST_ASSERT_EQUAL_STRING("unknown tag", elevDecodeStatusName(ELEV_ERR_TAG));
  TEST_ASSERT_EQUAL_STRING("length mismatch", elevDecodeStatusName(ELEV_ERR_LENGTH));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_state_byte_layout);
  RUN_TEST(test_state_negative_pos_is_little_endian_twos_complement);
  RUN_TEST(test_state_roundtrip_min);
  RUN_TEST(test_state_roundtrip_max);
  RUN_TEST(test_state_roundtrip_typical);
  RUN_TEST(test_state_model_not_ready_is_floor_zero);
  RUN_TEST(test_state_bits_are_at_the_documented_positions);
  RUN_TEST(test_state_accessors_agree_with_the_builder);
  RUN_TEST(test_direction_never_disturbs_the_neighbouring_flags);
  RUN_TEST(test_state_rejects_truncated_and_overlong);
  RUN_TEST(test_state_rejects_wrong_tag);
  RUN_TEST(test_state_pack_rejects_a_short_buffer);
  RUN_TEST(test_stats_byte_layout);
  RUN_TEST(test_stats_roundtrip_min);
  RUN_TEST(test_stats_roundtrip_max);
  RUN_TEST(test_stats_roundtrip_typical);
  RUN_TEST(test_stats_lifetime_odometer_survives_a_year);
  RUN_TEST(test_stats_accessors_undo_the_scaling);
  RUN_TEST(test_stats_battery_thresholds);
  RUN_TEST(test_stats_flag_bits_are_at_the_documented_positions);
  RUN_TEST(test_stats_rejects_truncated_and_overlong);
  RUN_TEST(test_stats_rejects_wrong_tag);
  RUN_TEST(test_stats_pack_rejects_a_short_buffer);
  RUN_TEST(test_formats_do_not_decode_as_each_other);
  RUN_TEST(test_packet_length_from_tag);
  RUN_TEST(test_decode_status_names_exist);
  return UNITY_END();
}
