//
// The per-floor screen. Ten identical units, one on every landing.
//
// This node is a renderer and a relay, and deliberately nothing else. It never
// runs the floor algorithm and it never infers a floor from anything but a
// received packet. Spec section 1: the decision belongs in the car, where the
// sensor is, so radio loss can only age what a screen shows - it can never
// corrupt it, and one received packet resyncs everything. A display that
// second-guessed the packet would turn every dropout into a correctness
// problem instead of a latency one.
//
// ---------------------------------------------------------------------------
// Why the relay is never made to wait for a repaint
// ---------------------------------------------------------------------------
// meshService() is where a frame whose jitter has expired actually goes back on
// the air. A TFT repaint is tens of milliseconds; the top floor is five hops
// from the bridge, so a display that redrew before servicing the mesh would add
// that delay once per hop, and a floor number arriving a fifth of a second late
// is visible to somebody standing in front of it.
//
// So the frame handler does packet arithmetic and nothing else - it records
// what arrived and sets a flag - and every pixel is written from loop(), after
// meshService() has returned. The dedup, the hop decrement and the rebroadcast
// decision all live in espnow_mesh.cpp and have already happened by the time
// the handler is called; what this file owes the mesh is simply to get out of
// its way quickly.
//
// ---------------------------------------------------------------------------
// What this node has to decide for itself
// ---------------------------------------------------------------------------
// Two things, both of which are about time rather than about the elevator:
// how stale the link is, and what the odometer reads between heartbeats.
// Everything else on the screen is a field out of a packet.
//
#include "display_ui.h"
#include "elev_packet.h"
#include "espnow_mesh.h"

// ---------------------------------------------------------------------------
// Cadences
// ---------------------------------------------------------------------------

// How often the screen is rebuilt when no packet has arrived. Nothing in the
// UI state moves on its own except the two staleness flags, and a quarter
// second of lag on a flag that has already waited ten seconds is not something
// anyone can see. A packet sets gDirty and repaints immediately, so this is not
// what governs how fast the floor number responds.
#ifndef UI_REFRESH_MS
#define UI_REFRESH_MS 250
#endif

// Dedup is reported as a delta rather than per frame. With ten displays each
// sending MESH_REPEATS copies, one STATE can produce twenty-odd duplicates at a
// well-connected node, and a line each would bury the frames that matter.
#ifndef DEDUP_REPORT_MIN_MS
#define DEDUP_REPORT_MIN_MS 1000
#endif

// The periodic "what does this node think is going on" summary.
#ifndef SUMMARY_INTERVAL_MS
#define SUMMARY_INTERVAL_MS 30000
#endif

// Statute miles. The corridor reads miles; everything on the wire is metres,
// and this is the only place the two meet.
#define METRES_PER_MILE 1609.344f

// ---------------------------------------------------------------------------
// What the last packets said
//
// Touched only from meshService()'s frame handler and from loop(), both of
// which run on the Arduino task - nothing here is reachable from the WiFi
// callback, so none of it needs to be volatile or guarded.
// ---------------------------------------------------------------------------
static bool     gSeenAnyFrame = false;
static uint32_t gFirstFrameMs = 0;

static bool     gHaveState   = false;
static uint32_t gLastStateMs = 0;
static bool     gHaveStats   = false;
static uint32_t gLastStatsMs = 0;

static uint8_t  gFloor      = 0;  // confirmed 1-based index, 0 = model not ready
static float    gPosition   = 0.0f;
static bool     gModelReady = false;
static bool     gMoving     = false;
static uint8_t  gDirection  = ELEV_DIR_IDLE;

static float    gBatteryVolts = 0.0f;
static float    gPitchM       = 0.0f;

// The odometer, as a fixed baseline from the last heartbeat plus whatever this
// node has watched happen since. See accrueDistance().
static float    gDist24hBaseM     = 0.0f;
static uint32_t gFloorsSinceStats = 0;
static uint8_t  gLastFloorSeen    = 0;

static bool     gDirty         = false;
static uint32_t gLastDrawMs    = 0;
static uint32_t gDedupReported = 0;
static uint32_t gDedupReportMs = 0;
static uint32_t gSummaryMs     = 0;
static uint32_t gBadFrames     = 0;  // right envelope, unusable contents

// ===========================================================================
// Packet handling
// ===========================================================================

// The mesh wrapper strips tag, txId and seq and carries the tag in its own
// `type` field (mesh_packet.h). Putting those three bytes back and handing the
// result to the elev_packet.h unpackers keeps every field offset in the one
// file the host tests exercise - reading the payload at open-coded offsets here
// would be a second copy of the wire format to keep in step, and the failure
// mode of the two drifting apart is a screen that is confidently wrong.
//
// txId and seq are restored as zero. There is one transmitter, and neither
// field means anything to a display.
static bool rebuildLoraPacket(const MeshFrame &f, uint8_t *out, size_t cap,
                              size_t *outLen) {
  const size_t want = elevPacketLength(f.type);
  if (want == 0 || want > cap) return false;
  if ((size_t)f.len + ELEV_BODY_OFFSET != want) return false;

  out[0] = f.type;
  out[1] = 0;
  out[2] = 0;
  memcpy(&out[ELEV_BODY_OFFSET], f.payload, f.len);
  *outLen = want;
  return true;
}

// The odometer between heartbeats.
//
// STATS carries the transmitter's own 24 h figure once a minute and that is the
// truth; this only fills the sixty seconds in between, so the number ticks
// while the car is actually running instead of sitting still and then jumping.
// It is never a second opinion - applyStats() overwrites the baseline and zeroes
// the counter, so all ten screens are back in exact agreement every minute
// (spec section 6).
//
// Counting confirmed floor changes rather than integrating posQ8 is the same
// rule as everywhere else here: posQ8 is animation, `floor` is the decision.
static void accrueDistance(uint8_t floor) {
  if (floor == 0) return;  // model not ready, so there is no lattice to count on
  if (gLastFloorSeen != 0 && floor != gLastFloorSeen) {
    const int delta = (int)floor - (int)gLastFloorSeen;
    gFloorsSinceStats += (uint32_t)(delta < 0 ? -delta : delta);
  }
  gLastFloorSeen = floor;
}

static void applyState(const uint8_t *pkt, size_t len) {
  ElevState st;
  if (elevStateUnpack(pkt, len, &st) != ELEV_OK) {
    gBadFrames++;
    return;
  }

  gLastStateMs = millis();
  gHaveState   = true;

  gFloor     = st.floor;
  gMoving    = elevStateMoving(&st);
  gDirection = (uint8_t)elevStateDirection(&st);

  // posQ8 is animation only, and gating it on modelReady is what keeps it that
  // way: it is derived from the drift-tracked datum that ALGORITHM.md section 6
  // shows cannot be trusted to decide a floor. display_ui.cpp uses `position`
  // only while the car is moving and drops it the instant `floor` disagrees
  // (spec section 4.1).
  gModelReady = elevStateModelReady(&st);
  gPosition   = elevStatePosFloorIndex(&st);

  accrueDistance(st.floor);
  gDirty = true;
}

static void applyStats(const uint8_t *pkt, size_t len) {
  ElevStats s;
  if (elevStatsUnpack(pkt, len, &s) != ELEV_OK) {
    gBadFrames++;
    return;
  }

  gLastStatsMs = millis();
  gHaveStats   = true;

  gBatteryVolts = elevStatsBatteryVolts(&s);
  gPitchM       = elevStatsPitchM(&s);

  // The transmitter's figure lands as-is and whatever this node extrapolated
  // since the last heartbeat is discarded, not added to it.
  gDist24hBaseM     = (float)s.dist24hM;
  gFloorsSinceStats = 0;

  Serial.printf("[stats] batt %.2f V  24h %u m  total %lu m  pitch %u mm  "
                "floors %u  trips %u  stops %u  up %lu s  flags 0x%02X  %d C\n",
                (double)gBatteryVolts, (unsigned)s.dist24hM,
                (unsigned long)s.distTotalM, (unsigned)s.pitchMm,
                (unsigned)s.nFloors, (unsigned)s.trips, (unsigned)s.stops,
                (unsigned long)s.uptimeS, (unsigned)s.flags, (int)s.tempC);

  gDirty = true;
}

// Called from meshService(), on the Arduino task. Fast on purpose - see the
// note at the top of this file.
static void onMeshFrame(const MeshFrame &frame, const uint8_t *srcMac,
                        int8_t rssi) {
  if (!gSeenAnyFrame) {
    gSeenAnyFrame = true;
    gFirstFrameMs = millis();
  }

  // frame.hop is the count as it arrived; meshDecrementHop() takes one off and
  // re-sends only what still has a hop left, so a frame arriving at 1 dies
  // here. Worth printing rather than inferring afterwards: from a floor below,
  // "the top of the building stopped relaying" and "the top of the building
  // stopped hearing" look identical.
  const bool willRelay = frame.hop > 1;

  char mac[18];
  Serial.printf("[mesh] seq %u  hop %u  type 0x%02X  len %u  rssi %d  "
                "from %s  %s\n",
                (unsigned)frame.origSeq, (unsigned)frame.hop,
                (unsigned)frame.type, (unsigned)frame.len, (int)rssi,
                srcMac ? meshFormatMac(srcMac, mac) : "??",
                willRelay ? "relaying" : "hop exhausted");

  uint8_t pkt[ELEV_MAX_PACKET_BYTES];
  size_t  len = 0;
  if (!rebuildLoraPacket(frame, pkt, sizeof(pkt), &len)) {
    gBadFrames++;
    Serial.printf("[mesh] seq %u: type 0x%02X with %u payload bytes is not a "
                  "format this build speaks\n",
                  (unsigned)frame.origSeq, (unsigned)frame.type,
                  (unsigned)frame.len);
    return;
  }

  if (frame.type == ELEV_TAG_STATE) {
    applyState(pkt, len);
  } else {
    applyStats(pkt, len);
  }
}

// ===========================================================================
// Staleness and the UI state
// ===========================================================================

// Before the first packet of a given kind arrives there is nothing to age, so
// the clock runs from the moment this node first heard anything at all. That is
// what makes a display which hears STATE but never a STATS heartbeat grey out
// after 180 s rather than sit there trusting a battery reading it was never
// sent.
static bool streamStale(bool have, uint32_t lastMs, uint32_t limitMs,
                        uint32_t now) {
  const uint32_t since = have ? (now - lastMs) : (now - gFirstFrameMs);
  return since > limitMs;
}

static DisplayUiState buildUiState(uint32_t now) {
  DisplayUiState s = displayUiStateInit();

  s.floorIndex    = gFloor;
  s.position      = gPosition;
  s.positionValid = gModelReady;
  s.moving        = gMoving;
  s.direction     = gDirection;

  // Battery and the odometer both come from STATS, so they are legitimately
  // absent for the first minute after boot and draw as "--" until then.
  s.batteryValid = gHaveStats;
  s.batteryVolts = gBatteryVolts;
  s.distValid    = gHaveStats;
  s.dist24hMiles =
      (gDist24hBaseM + (float)gFloorsSinceStats * gPitchM) / METRES_PER_MILE;

  // Both from the STATS clock: STATE stops whenever the car parks, so timing the
  // dim against it would dim every screen through every idle period.
  s.stateStale = streamStale(gHaveStats, gLastStatsMs, DISPLAY_DIM_STALE_MS, now);
  s.statsStale = streamStale(gHaveStats, gLastStatsMs, DISPLAY_STATS_STALE_MS, now);
  return s;
}

// ===========================================================================
// Console reporting
//
// The audience for all of this is somebody standing in a stairwell with a USB
// cable, asking whether this particular screen is hearing the bridge directly,
// hearing it through a neighbour, or not hearing it at all.
// ===========================================================================

static void reportDedup(uint32_t now) {
  const MeshCounters &c = meshCounters();
  if (c.deduped == gDedupReported) return;
  if ((uint32_t)(now - gDedupReportMs) < DEDUP_REPORT_MIN_MS) return;

  Serial.printf("[mesh] +%lu duplicate frames absorbed (%lu total)\n",
                (unsigned long)(c.deduped - gDedupReported),
                (unsigned long)c.deduped);
  gDedupReported = c.deduped;
  gDedupReportMs = now;
}

static void reportSummary(uint32_t now) {
  if ((uint32_t)(now - gSummaryMs) < SUMMARY_INTERVAL_MS) return;
  gSummaryMs = now;

  meshPrintCounters();

  const DisplayUiState s = buildUiState(now);
  Serial.printf("[ui] floor %s  pos %.2f%s  %s  %.2f mi (%lu floors since the "
                "last heartbeat)  %s%s  bad frames %lu  backlight %u\n",
                displayFloorLabel(s.floorIndex), (double)s.position,
                s.positionValid ? "" : " (stale model)",
                s.moving ? (s.direction == ELEV_DIR_UP ? "moving up"
                                                       : "moving down")
                         : "stopped",
                (double)s.dist24hMiles, (unsigned long)gFloorsSinceStats,
                s.stateStale ? "STATE stale" : "STATE fresh",
                s.statsStale ? ", STATS stale" : "",
                (unsigned long)gBadFrames, (unsigned)displayBacklight());
}

// ===========================================================================
// Lifecycle
// ===========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("=== simmevator floor display ==="));

  // The panel comes up before the mesh on purpose: meshBringUp() halts on
  // failure, and a board stuck on the splash screen is a diagnosis where a dark
  // one is a mystery.
  if (!displayBegin()) {
    Serial.println(F("[ui] no number sprite - drawing direct, expect flicker"));
  }
  displaySplash("Simmevator", "waiting for the mesh");

  // A label table that is one entry short shows up in the field as a display
  // that is silently wrong at the top of the building and right everywhere
  // else, so print the ends of it where somebody will see them.
  Serial.printf("[ui] %u floor labels, \"%s\" through \"%s\"\n",
                (unsigned)displayFloorLabelCount(), displayFloorLabel(1),
                displayFloorLabel(displayFloorLabelCount()));

  // MESH_ROLE_RELAY: this node floods like every other, but may not mint an
  // origSeq. Two nodes minting into the same u16 space would collide, and the
  // dedup ring would read the collisions as duplicates and swallow them.
  meshBringUp(MESH_ROLE_RELAY, onMeshFrame, "floor display");

  const uint32_t now = millis();
  gLastDrawMs    = now;
  gDedupReportMs = now;
  gSummaryMs     = now;
}

void loop() {
  const uint32_t now = millis();

  // Relay first. See the note at the top of this file.
  meshService();

  displayTick();

  if (gDirty || (uint32_t)(now - gLastDrawMs) >= UI_REFRESH_MS) {
    gDirty      = false;
    gLastDrawMs = now;
    // The splash stays up until something has actually been heard. "--" on a
    // fully drawn screen reads as a known floor that happens to be unknown;
    // "waiting for the mesh" reads as what it is.
    if (gSeenAnyFrame) displayUpdate(buildUiState(now));
  }

  // The repaint above may have run long. Drain again rather than leave a frame
  // whose jitter expired mid-draw waiting out another pass through loop().
  meshService();

  reportDedup(now);
  reportSummary(now);
}
