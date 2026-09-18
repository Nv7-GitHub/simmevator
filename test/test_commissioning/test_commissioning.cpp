//
// Host-side unit tests for the three-elevator change (spec 5.6).
//
// Two things are covered here, and they fail in opposite ways.
//
// The commissioning verdict is the one piece of this change that can be wrong
// without anybody noticing. Spec 4.1's trap - a car commissioned over a period
// in which nobody pressed B learns a 10-landing span in an 11-landing shaft -
// produces a model that is internally consistent, ready, confident, and one
// floor low on all eleven screens indefinitely. displayCommissionState() is the
// only thing in the system that can see it, so every row of spec 4.3's table
// gets a test, and so does every combination the table does not name: an
// unnamed combination that silently resolves to OK is the same failure with an
// extra step. The exhaustive sweep at the end states the safety property
// directly rather than re-deriving the table, so a future rewrite of the
// function is free to change shape but not to start returning OK while the
// model and the building disagree.
//
// The bridge's txId filter fails loudly instead - a wrong offset drops every
// packet and the shaft goes dark in one heartbeat - so what is worth pinning is
// the offset itself, which bridge_rx.cpp reads from the raw buffer before
// either unpacker runs and therefore cannot get from a shared constant. The
// tests below pack real packets and assert on the byte at that offset, so the
// literal 1 in bridge_rx.cpp is checked against what elev_packet.h actually
// writes rather than against a restatement of it.
//
//   pio test -e native
//
#include <unity.h>

#include <string.h>

#include "display_ui.h"
#include "elev_packet.h"

// The three shafts as platformio.ini builds them (spec 2). Written out here
// rather than taken from the build flags: this test runs in env:native, which
// defines no elevator, and the point of the txId tests is to check one shaft's
// id against another's the way two boards on one table would.
static const uint8_t kTxIdA = 0x41;
static const uint8_t kTxIdB = 0x42;
static const uint8_t kTxIdC = 0x43;

// The offset bridge_rx.cpp reads txId from, as a literal. It is deliberately
// not ELEV_BODY_OFFSET or any other constant from elev_packet.h - a test that
// derived it the same way the implementation does would pass while both were
// wrong together, which is the mistake test_elev_packet.cpp's header comment
// describes.
static const size_t kTxIdOffset = 1;

// A and C have a basement, B does not (spec 1).
static const uint8_t kLabelsWithBasement = 11;
static const uint8_t kLabelsNoBasement   = 10;

// ===========================================================================
// The commissioning verdict - spec 4.3
// ===========================================================================

// Everything the verdict reads, and nothing else. A test that built a whole
// plausible DisplayUiState would hide which field it was actually varying.
static DisplayCommission verdict(uint8_t labelCount, uint8_t modelFloors,
                                 bool modelFloorsValid, bool modelReady,
                                 bool nvsRestored) {
  DisplayUiState s      = displayUiStateInit();
  s.modelFloors         = modelFloors;
  s.modelFloorsValid    = modelFloorsValid;
  s.modelReady          = modelReady;
  s.nvsRestored         = nvsRestored;
  return displayCommissionState(s, labelCount);
}

// ---------------------------------------------------------------------------
// Row 1: nFloors < labelCount, ready 0, restored 0 -> LEARNING
// ---------------------------------------------------------------------------
void test_a_fresh_model_short_of_the_shaft_is_learning(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_LEARNING,
                    verdict(kLabelsWithBasement, 3, true, false, false));
}

// Spec 4.4: nFloors is the span of learned indices, not a tally of landings
// visited, so an express run from the basement to floor 10 jumps the readout
// from 2 to 11. Both ends of that jump are still LEARNING - the verdict must
// not treat a span of 1 as a fault, because that is what a car that has only
// confirmed one stop legitimately reports.
void test_learning_covers_the_whole_span_below_the_label_count(void) {
  for (uint8_t n = 0; n < kLabelsWithBasement; n++) {
    TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_LEARNING,
                      verdict(kLabelsWithBasement, n, true, false, false));
  }
}

// ---------------------------------------------------------------------------
// Row 2: nFloors == labelCount, ready 0, restored 1 -> ANCHORING
// ---------------------------------------------------------------------------
void test_a_restored_model_spanning_the_shaft_is_anchoring(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_ANCHORING,
                    verdict(kLabelsWithBasement, kLabelsWithBasement, true, false, true));
}

// ---------------------------------------------------------------------------
// Row 3: nFloors == labelCount, ready 1 -> normal operation
// ---------------------------------------------------------------------------
void test_a_ready_model_spanning_the_shaft_is_normal_operation(void) {
  // The table leaves NVS_RESTORED unspecified on this row, and it has to stay
  // that way: a transmitter that restored its model out of flash and then
  // anchored is in exactly the same condition as one that learned the shaft
  // from nothing, and keeps the flag set for the rest of its uptime.
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_OK,
                    verdict(kLabelsWithBasement, kLabelsWithBasement, true, true, false));
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_OK,
                    verdict(kLabelsWithBasement, kLabelsWithBasement, true, true, true));
}

// Elevator B must behave exactly as the single-elevator system does today,
// which for this function means: ten landings, ten labels, nothing on screen.
void test_elevator_b_with_ten_landings_shows_nothing_new(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_OK,
                    verdict(kLabelsNoBasement, kLabelsNoBasement, true, true, false));
}

// ---------------------------------------------------------------------------
// Row 4: nFloors != labelCount, ready 1 -> CHECK SHAFT
// ---------------------------------------------------------------------------

// Spec 4.1, the reason this whole mechanism exists: elevator A commissioned
// without anybody pressing B. Ten learned landings, eleven real ones, a model
// that is ready and internally consistent, and every label one floor too low.
void test_a_ready_model_one_landing_short_is_the_off_by_one_trap(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                    verdict(kLabelsWithBasement, kLabelsNoBasement, true, true, false));
}

// The same fault seen from the other side: elevator B's display flashed with
// floor_display_a, so an eleven-landing label table meets B's ten-landing car.
// Spec 2 says identity is a build flag and this is the state that catches a
// board flashed from the wrong environment.
void test_a_ready_model_one_landing_long_is_also_check_shaft(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                    verdict(kLabelsNoBasement, kLabelsWithBasement, true, true, false));
}

// ---------------------------------------------------------------------------
// The combinations spec 4.3's table does not name
//
// Each of these is a decision display_ui.h documents at the function, pinned
// here so that changing one is a deliberate act rather than a side effect.
// ---------------------------------------------------------------------------

// No STATS heartbeat yet is not a fault, and this is the one unnamed case that
// resolves towards OK rather than away from it. It lasts up to 60 s after every
// boot and after every mesh outage longer than the heartbeat, so a CHECK SHAFT
// here would fire on every power cycle and clear itself - and an alarm that
// cries wolf daily is one nobody reads. The screen is not confidently wrong in
// that window either: with no model the floor renders as "--" (spec 4.6).
void test_no_heartbeat_yet_is_not_a_fault_whatever_else_is_set(void) {
  for (int ready = 0; ready <= 1; ready++) {
    for (int restored = 0; restored <= 1; restored++) {
      for (uint8_t n = 0; n <= 13; n++) {
        TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_OK,
                          verdict(kLabelsWithBasement, n, false, ready != 0, restored != 0));
      }
    }
  }
}

// An unparsable FLOOR_LABELS leaves nothing to compare against and no label to
// draw, so there is no state in which a zero-length table is acceptable.
void test_an_empty_label_table_is_always_check_shaft(void) {
  for (int valid = 0; valid <= 1; valid++) {
    for (int ready = 0; ready <= 1; ready++) {
      for (int restored = 0; restored <= 1; restored++) {
        TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                          verdict(0, 0, valid != 0, ready != 0, restored != 0));
      }
    }
  }
}

// A span already wider than the building cannot be fixed by learning more of
// it, so waiting is the wrong instruction even though the model is not ready.
void test_a_span_wider_than_the_shaft_is_check_shaft_before_ready(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                    verdict(kLabelsWithBasement, 12, true, false, false));
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                    verdict(kLabelsWithBasement, 12, true, false, true));
}

// A model that came back out of flash already short of the building is spec
// 4.1's trap persisted, so it reads as CHECK SHAFT rather than LEARNING. The
// two states ask for different things - LEARNING says wait, CHECK SHAFT says
// ride the whole shaft - and riding the whole shaft is what actually clears it.
void test_a_restored_model_short_of_the_shaft_asks_for_a_ride(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_MISMATCH,
                    verdict(kLabelsWithBasement, kLabelsNoBasement, true, false, true));
}

// Full span, not ready, and learned here rather than restored: something is
// being waited on, and "ride to both ends, in either order" (spec 4.5) is both
// harmless and the right move whatever that something turns out to be.
void test_a_freshly_learned_full_span_that_is_not_ready_anchors(void) {
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_ANCHORING,
                    verdict(kLabelsWithBasement, kLabelsWithBasement, true, false, false));
}

// ---------------------------------------------------------------------------
// The safety property, over every combination
//
// Stated as a property rather than as the table again. The table says what each
// input produces; this says what no input may produce, which is the part that
// has to survive a rewrite of the function. A verdict that answered OK while
// the model and the building disagree is spec 4.1 restored in full: eleven
// screens, confidently wrong, with nothing anywhere that can tell.
// ---------------------------------------------------------------------------
void test_ok_is_unreachable_whenever_the_model_and_the_building_disagree(void) {
  for (uint8_t labelCount = 0; labelCount <= 13; labelCount++) {
    for (uint8_t n = 0; n <= 13; n++) {
      for (int ready = 0; ready <= 1; ready++) {
        for (int restored = 0; restored <= 1; restored++) {
          const DisplayCommission c =
              verdict(labelCount, n, true, ready != 0, restored != 0);

          // Totality: every combination lands on one of the four states, so
          // the renderer never has to guess at a default.
          TEST_ASSERT_TRUE(c == DISPLAY_COMMISSION_OK ||
                           c == DISPLAY_COMMISSION_LEARNING ||
                           c == DISPLAY_COMMISSION_ANCHORING ||
                           c == DISPLAY_COMMISSION_MISMATCH);

          if (labelCount == 0 || n != labelCount) {
            TEST_ASSERT_NOT_EQUAL(DISPLAY_COMMISSION_OK, c);
          }
          // Not ready means the floor on screen is "--" and the only honest
          // answers are the three commissioning states.
          if (!ready) {
            TEST_ASSERT_NOT_EQUAL(DISPLAY_COMMISSION_OK, c);
          }
        }
      }
    }
  }
}

// The verdict is about commissioning, not about the link. Staleness, battery
// and the floor itself already have their own rendering, and coupling them to
// this would mean a mesh outage eventually reading as a mis-commissioned shaft.
void test_the_verdict_ignores_everything_that_is_not_the_model(void) {
  DisplayUiState quiet = displayUiStateInit();
  quiet.modelFloors      = kLabelsNoBasement;
  quiet.modelFloorsValid = true;
  quiet.modelReady       = true;

  DisplayUiState noisy   = quiet;
  noisy.floorIndex       = 7;
  noisy.position         = 7.5f;
  noisy.positionValid    = true;
  noisy.moving           = true;
  noisy.direction        = 1;
  noisy.batteryVolts     = 11.4f;
  noisy.batteryValid     = true;
  noisy.batteryLevel     = DISPLAY_BATTERY_CRITICAL;
  noisy.dist24hMiles     = 2.1f;
  noisy.distValid        = true;
  noisy.stateStale       = true;
  noisy.statsStale       = true;

  TEST_ASSERT_EQUAL(displayCommissionState(quiet, kLabelsNoBasement),
                    displayCommissionState(noisy, kLabelsNoBasement));
  TEST_ASSERT_EQUAL(displayCommissionState(quiet, kLabelsWithBasement),
                    displayCommissionState(noisy, kLabelsWithBasement));
}

// displayUiStateInit() is what floor_display.cpp starts every frame from, so
// what it leaves behind decides what a display shows in the seconds before the
// first packet arrives. That has to be the quiet state, not a CHECK SHAFT.
void test_a_freshly_initialised_state_is_quiet(void) {
  DisplayUiState s = displayUiStateInit();
  TEST_ASSERT_FALSE(s.modelFloorsValid);
  TEST_ASSERT_FALSE(s.modelReady);
  TEST_ASSERT_FALSE(s.nvsRestored);
  TEST_ASSERT_EQUAL_UINT8(0, s.modelFloors);
  TEST_ASSERT_EQUAL_UINT8(0, s.floorIndex);  // draws as "--", spec 4.6
  TEST_ASSERT_EQUAL(DISPLAY_COMMISSION_OK,
                    displayCommissionState(s, kLabelsWithBasement));
}

// ===========================================================================
// An eleven-entry label table - spec 4.7
//
// What is checked here is only the headroom spec 4.7 claims, because the parser
// itself is out of reach: parseFloorLabels(), displayFloorLabel() and
// segmentsFor() are file-static in display_ui.cpp, which constructs a TFT_eSPI
// at file scope and so cannot be compiled by env:native. Making the eleven
// labels and the 'B' glyph testable on the host would mean moving those three
// out of display_ui.cpp and into display_ui.h, outside DISPLAY_UI_HAVE_PANEL -
// a src/ change this file is not permitted to make. Until then they are covered
// by the nine firmware builds and by the commissioning run in spec 7.2, and
// this test covers the one claim that can be checked from the header alone.
// ===========================================================================
void test_the_label_table_has_room_for_a_basement(void) {
  // Spec 4.7 justifies adding a basement by saying the ceiling has room to
  // spare. If a future edit trims DISPLAY_MAX_LABELS towards B's ten, A and C
  // silently lose their top landing and every display in those shafts draws
  // "--" for floor 11.
  TEST_ASSERT_TRUE(DISPLAY_MAX_LABELS >= kLabelsWithBasement);

  // "10" is the longest label in any of the three tables. Labels are truncated
  // rather than shrunk, so a ceiling of 1 would render floor 10 as "1".
  TEST_ASSERT_TRUE(DISPLAY_MAX_LABEL_CHARS >= 2);
}

// ===========================================================================
// The bridge's foreign-packet filter - spec 3
//
// bridge_rx.cpp reads txId straight out of the received buffer at a file-local
// ELEV_OFF_TXID, before either unpacker runs, because the filter has to work on
// a frame it has not yet decided the length of. That is a second, independent
// statement of where txId lives, and these tests are what stops it from
// drifting away from the one in elev_packet.h.
//
// The drop itself is not reachable from the host: handleFrame() is static in a
// translation unit that calls millis(), digitalWrite() and Serial. It is
// covered by the nine firmware builds and by the bench coexistence test in
// spec 7.2, where dropForeignTxId staying at 0 with all three systems on one
// table is the pass condition.
// ===========================================================================
void test_txid_is_at_the_same_offset_in_both_lora_formats(void) {
  ElevState st;
  memset(&st, 0, sizeof(st));
  st.txId = kTxIdB;
  uint8_t state[ELEV_STATE_BYTES];
  TEST_ASSERT_EQUAL_UINT32(ELEV_STATE_BYTES,
                           (uint32_t)elevStatePack(state, sizeof(state), &st));
  TEST_ASSERT_EQUAL_HEX8(kTxIdB, state[kTxIdOffset]);

  ElevStats sa;
  memset(&sa, 0, sizeof(sa));
  sa.txId = kTxIdB;
  uint8_t stats[ELEV_STATS_BYTES];
  TEST_ASSERT_EQUAL_UINT32(ELEV_STATS_BYTES,
                           (uint32_t)elevStatsPack(stats, sizeof(stats), &sa));
  TEST_ASSERT_EQUAL_HEX8(kTxIdB, stats[kTxIdOffset]);
}

// The filter compares one byte, so the three ids have to differ in that byte
// and nowhere else matters. ASCII 'A'/'B'/'C' is spec 3's choice so that a hex
// dump reads "E0 42 ..." and names the shaft without a lookup table.
void test_the_three_elevator_ids_are_distinct_ascii_letters(void) {
  TEST_ASSERT_EQUAL_HEX8('A', kTxIdA);
  TEST_ASSERT_EQUAL_HEX8('B', kTxIdB);
  TEST_ASSERT_EQUAL_HEX8('C', kTxIdC);
  TEST_ASSERT_NOT_EQUAL(kTxIdA, kTxIdB);
  TEST_ASSERT_NOT_EQUAL(kTxIdB, kTxIdC);
  TEST_ASSERT_NOT_EQUAL(kTxIdA, kTxIdC);
}

// What bridge B does at the byte it reads: its own car's packets match, the
// other two shafts' do not. Built from real packed frames rather than from
// hand-written bytes, so a change to either pack function that moved txId would
// show up here as well as in test_elev_packet.cpp.
void test_bridge_b_sees_its_own_car_and_not_the_other_two(void) {
  const uint8_t others[] = { kTxIdA, kTxIdC };

  ElevState st;
  memset(&st, 0, sizeof(st));
  uint8_t buf[ELEV_STATE_BYTES];

  st.txId = kTxIdB;
  elevStatePack(buf, sizeof(buf), &st);
  TEST_ASSERT_EQUAL_HEX8(kTxIdB, buf[kTxIdOffset]);  // accepted, forwarded

  for (size_t i = 0; i < sizeof(others); i++) {
    st.txId = others[i];
    elevStatePack(buf, sizeof(buf), &st);
    TEST_ASSERT_NOT_EQUAL(kTxIdB, buf[kTxIdOffset]);  // dropForeignTxId++

    // The tag is still ours, so the filter is the only thing standing between
    // another shaft's car and bridge B's mesh. Nothing earlier in handleFrame()
    // would have rejected this frame.
    TEST_ASSERT_EQUAL_HEX8(ELEV_TAG_STATE, buf[0]);
    TEST_ASSERT_EQUAL_UINT32(ELEV_STATE_BYTES,
                             (uint32_t)elevPacketLength(buf[0]));
  }
}

// handleFrame() guards the read with `len > ELEV_OFF_TXID`, and that guard is
// the difference between counting a truncated frame as a length error and
// reading one byte off the end of the buffer. Both real formats are long enough
// that the guard never fires on a whole packet; a length-1 frame carries a
// valid tag and no txId at all.
void test_the_txid_offset_is_inside_every_real_packet(void) {
  TEST_ASSERT_TRUE(kTxIdOffset < ELEV_STATE_BYTES);
  TEST_ASSERT_TRUE(kTxIdOffset < ELEV_STATS_BYTES);
  TEST_ASSERT_TRUE(kTxIdOffset >= 1);  // byte 0 is the tag, never a txId
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_a_fresh_model_short_of_the_shaft_is_learning);
  RUN_TEST(test_learning_covers_the_whole_span_below_the_label_count);
  RUN_TEST(test_a_restored_model_spanning_the_shaft_is_anchoring);
  RUN_TEST(test_a_ready_model_spanning_the_shaft_is_normal_operation);
  RUN_TEST(test_elevator_b_with_ten_landings_shows_nothing_new);
  RUN_TEST(test_a_ready_model_one_landing_short_is_the_off_by_one_trap);
  RUN_TEST(test_a_ready_model_one_landing_long_is_also_check_shaft);
  RUN_TEST(test_no_heartbeat_yet_is_not_a_fault_whatever_else_is_set);
  RUN_TEST(test_an_empty_label_table_is_always_check_shaft);
  RUN_TEST(test_a_span_wider_than_the_shaft_is_check_shaft_before_ready);
  RUN_TEST(test_a_restored_model_short_of_the_shaft_asks_for_a_ride);
  RUN_TEST(test_a_freshly_learned_full_span_that_is_not_ready_anchors);
  RUN_TEST(test_ok_is_unreachable_whenever_the_model_and_the_building_disagree);
  RUN_TEST(test_the_verdict_ignores_everything_that_is_not_the_model);
  RUN_TEST(test_a_freshly_initialised_state_is_quiet);
  RUN_TEST(test_the_label_table_has_room_for_a_basement);
  RUN_TEST(test_txid_is_at_the_same_offset_in_both_lora_formats);
  RUN_TEST(test_the_three_elevator_ids_are_distinct_ascii_letters);
  RUN_TEST(test_bridge_b_sees_its_own_car_and_not_the_other_two);
  RUN_TEST(test_the_txid_offset_is_inside_every_real_packet);
  return UNITY_END();
}
