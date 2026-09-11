//
// The floor-5 bridge: LoRa in, ESP-NOW out.
//
// Wall powered, so nothing here sleeps and the SX1262 stays in continuous
// receive - the car transmits on its own schedule and a bridge that was asleep
// would simply miss it.
//
// This node holds no state worth losing. It does not run FloorMonitor, does
// not touch NVS, and above all does not decide a floor: spec section 1 puts
// that decision in the car, where an unbroken 1 Hz barometer stream lives, so
// that radio loss can only age what a display shows and never corrupt it.
// Everything below is unwrap, validate, mint, flood.
//
// The one piece of judgement the bridge does exercise is refusing to forward a
// frame it could not decode. A foreign 915 MHz packet that happens to survive
// the SX1262's CRC would otherwise be wrapped and flooded to ten screens, and
// a wrong floor painted confidently is worse than a stale one.
//
// The other ordering rule is that publishing happens before printing. A STATE
// packet's whole job is to be on a display before the car finishes its next
// two seconds of travel, and Serial.printf at 115200 is milliseconds of that
// budget spent on a console nobody is watching most of the time.
//

#include <Arduino.h>

#include "elev_packet.h"
#include "espnow_mesh.h"
#include "lora_link.h"

// ---------------------------------------------------------------------------
// Receive buffer
//
// Deliberately larger than ELEV_MAX_PACKET_BYTES. loraPoll() truncates to the
// capacity it is given, so a 24-byte buffer would shorten an over-long foreign
// frame to exactly the length a STATS claims and hand it to the decoder with
// the length check already satisfied. Reading more than any real packet can be
// means an over-long frame stays over-long and is rejected on length.
// ---------------------------------------------------------------------------
#define RX_BUF_BYTES 64

#define REPORT_INTERVAL_MS 30000

// ---------------------------------------------------------------------------
// Sequence accounting
//
// Both formats carry `seq` as a u8, which repeats every 256 packets. At the
// car's 2 s STATE cadence that is 8.5 minutes and at the 60 s STATS cadence
// 4.3 hours, so a naive (now - last) difference produces a fictitious loss
// figure several times an hour rather than once in a blue moon.
//
// Two defences, because a u8 gap is only information under two conditions:
//
//   * The stream must not have been silent long enough for the counter to have
//     lapped. Past that horizon the difference is genuinely unknowable, so the
//     stream resyncs and counts nothing. This also covers the ordinary case of
//     a parked car, which emits no STATE at all for hours - not loss.
//   * The transmitter must not have restarted, since a reboot puts seq back
//     near zero and the unsigned difference reads as a near-full lap of loss.
//     STATS carries uptimeS, so a reboot is visible the moment the first STATS
//     after it arrives; uptime only ever counts up.
//
// The two formats are tracked as separate streams, which assumes the car
// advances a counter per format rather than one shared across both. A shared
// counter would show up here as a steady ~3% phantom STATE loss - one gap of
// two every sixty seconds - so the symptom is recognisable if that assumption
// is ever wrong.
// ---------------------------------------------------------------------------
#define SEQ_SPAN 256u

// The cadences elevator_tx is built with. They are not shared build flags:
// this environment does not compile the transmitter, and the horizon only has
// to be the right order of magnitude to tell "lapped" from "lost".
#define STATE_CADENCE_MS 2000u
#define STATS_CADENCE_MS 60000u

struct SeqTrack {
  const char *name;
  uint32_t    horizonMs;  // silence past which a gap carries no information
  bool        have;
  uint8_t     last;
  uint32_t    lastMs;
  uint32_t    good;
  uint32_t    lost;
  uint32_t    dups;
  uint32_t    resyncs;
};

static SeqTrack stateSeq = {"STATE", STATE_CADENCE_MS * SEQ_SPAN, false, 0, 0, 0, 0, 0, 0};
static SeqTrack statsSeq = {"STATS", STATS_CADENCE_MS * SEQ_SPAN, false, 0, 0, 0, 0, 0, 0};

// --- radio and decode counters ---
static uint32_t heardCount     = 0;  // frames the SX1262 handed over, good or not
static uint32_t radioCrcCount  = 0;  // hardware CRC rejected it
static uint32_t radioErrCount  = 0;  // readData failed for some other reason
static uint32_t dropTagCount   = 0;  // first byte is not a format we speak
static uint32_t dropLenCount   = 0;  // our tag, wrong length
static uint32_t publishCount   = 0;
static uint32_t publishFails   = 0;

// Frames arriving from the mesh with an origSeq this node never minted. The
// dedup ring swallows echoes of our own, so anything reaching the handler is
// a second origin on the channel - which would collide in the origSeq space
// and be read as duplicates by every display.
static uint32_t foreignOrigins = 0;

static uint32_t lastRxMs     = 0;
static uint32_t lastReportMs = 0;

// Last uptime a STATS reported, for the reboot test above.
static bool     haveUptime = false;
static uint32_t lastUptimeS = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void fmtUptime(uint32_t seconds, char *out, size_t cap) {
  uint32_t h = seconds / 3600u;
  uint32_t m = (seconds / 60u) % 60u;
  uint32_t s = seconds % 60u;
  snprintf(out, cap, "%luh%02lum%02lus", (unsigned long)h, (unsigned long)m,
           (unsigned long)s);
}

static const char *directionName(ElevDirection d) {
  switch (d) {
    case ELEV_DIR_UP:   return "up";
    case ELEV_DIR_DOWN: return "down";
    case ELEV_DIR_IDLE: return "idle";
  }
  return "?";
}

static float streamLossPercent(const SeqTrack &t) {
  uint32_t expected = t.good + t.lost;
  return expected ? (100.0f * (float)t.lost / (float)expected) : 0.0f;
}

// Resets both streams. A restarted transmitter invalidates every counter we
// hold, not just the one whose packet revealed the restart.
static void resyncAll(const char *why) {
  stateSeq.have = false;
  statsSeq.have = false;
  Serial.printf("[bridge] resync: %s\n", why);
}

static void trackSeq(SeqTrack *t, uint8_t seq, uint32_t nowMs) {
  if (t->have && (nowMs - t->lastMs) >= t->horizonMs) {
    t->resyncs++;
    t->have = false;
  }

  if (!t->have) {
    t->have = true;
  } else {
    uint8_t gap = (uint8_t)(seq - t->last);
    if (gap == 0) {
      // Either a genuine repeat or a full 256-packet lap, and inside the
      // horizon a lap is impossible. Count it, do not treat it as 255 losses.
      t->dups++;
    } else if (gap > 1) {
      t->lost += (uint32_t)(gap - 1);
      Serial.printf("[bridge] missed %u %s packet(s) before seq=%u\n",
                    (unsigned)(gap - 1), t->name, (unsigned)seq);
    }
  }

  t->last   = seq;
  t->lastMs = nowMs;
  t->good++;
}

// ---------------------------------------------------------------------------
// Packet handling
//
// Publish first, report second - see the note at the top of the file.
// ---------------------------------------------------------------------------

static void publish(const uint8_t *buf, size_t len) {
  if (meshPublishLora(buf, len)) {
    publishCount++;
  } else {
    publishFails++;
    Serial.printf("[bridge] mesh publish refused (%u bytes)\n", (unsigned)len);
  }
}

static void handleState(const uint8_t *buf, size_t len, uint32_t nowMs) {
  ElevState s;
  ElevDecodeStatus st = elevStateUnpack(buf, len, &s);
  if (st != ELEV_OK) {
    dropLenCount++;
    Serial.printf("[bridge] STATE rejected (%u bytes, %s) - not forwarded\n",
                  (unsigned)len, elevDecodeStatusName(st));
    return;
  }

  publish(buf, len);
  trackSeq(&stateSeq, s.seq, nowMs);

  // floor 0 means the car has not established its model yet, which is a
  // different thing from "ground floor" and is worth saying in words.
  char floorText[8];
  if (s.floor == 0) {
    snprintf(floorText, sizeof(floorText), "--");
  } else {
    snprintf(floorText, sizeof(floorText), "%u", (unsigned)s.floor);
  }

  Serial.printf("[bridge] STATE seq=%u floor=%s pos=%.2f %s %s conf=%.2f%s%s "
                "| RSSI=%.1f dBm SNR=%.1f dB | loss %.1f%% (%lu/%lu)\n",
                (unsigned)s.seq, floorText,
                (double)elevStatePosFloorIndex(&s),
                elevStateMoving(&s) ? "moving" : "stopped",
                directionName(elevStateDirection(&s)),
                (double)s.confidence / 255.0,
                elevStateModelReady(&s) ? "" : " [model not ready]",
                elevStateSensorErr(&s) ? " [SENSOR ERR]" : "",
                (double)loraLastRssi(), (double)loraLastSnr(),
                (double)streamLossPercent(stateSeq),
                (unsigned long)stateSeq.lost,
                (unsigned long)(stateSeq.good + stateSeq.lost));
}

static void handleStats(const uint8_t *buf, size_t len, uint32_t nowMs) {
  ElevStats s;
  ElevDecodeStatus st = elevStatsUnpack(buf, len, &s);
  if (st != ELEV_OK) {
    dropLenCount++;
    Serial.printf("[bridge] STATS rejected (%u bytes, %s) - not forwarded\n",
                  (unsigned)len, elevDecodeStatusName(st));
    return;
  }

  publish(buf, len);

  if (haveUptime && s.uptimeS < lastUptimeS) {
    resyncAll("transmitter restarted");
  }
  haveUptime  = true;
  lastUptimeS = s.uptimeS;

  trackSeq(&statsSeq, s.seq, nowMs);

  char uptimeText[24];
  fmtUptime(s.uptimeS, uptimeText, sizeof(uptimeText));

  Serial.printf("[bridge] STATS seq=%u vbat=%.2f V%s 24h=%.2f km total=%.1f km "
                "pitch=%.3f m n=%u trips=%u stops=%u up=%s temp=%d C%s%s "
                "| RSSI=%.1f dBm SNR=%.1f dB | loss %.1f%% (%lu/%lu)\n",
                (unsigned)s.seq, (double)elevStatsBatteryVolts(&s),
                elevStatsLowBattery(&s) ? " [LOW]" : "",
                (double)s.dist24hM / 1000.0, (double)s.distTotalM / 1000.0,
                (double)elevStatsPitchM(&s), (unsigned)s.nFloors,
                (unsigned)s.trips, (unsigned)s.stops, uptimeText, (int)s.tempC,
                elevStatsNvsRestored(&s) ? " [nvs]" : "",
                elevStatsSensorErr(&s) ? " [SENSOR ERR]" : "",
                (double)loraLastRssi(), (double)loraLastSnr(),
                (double)streamLossPercent(statsSeq),
                (unsigned long)statsSeq.lost,
                (unsigned long)(statsSeq.good + statsSeq.lost));
}

static void handleFrame(const uint8_t *buf, size_t len) {
  uint32_t nowMs = millis();
  lastRxMs = nowMs;

  if (len == 0) {
    dropLenCount++;
    return;
  }

  // elevPacketLength() is the one place that knows which tags exist, so an
  // unknown tag is rejected without guessing at a length for it.
  if (elevPacketLength(buf[0]) == 0) {
    dropTagCount++;
    Serial.printf("[bridge] unknown tag 0x%02X (%u bytes, RSSI=%.1f dBm) - not forwarded\n",
                  (unsigned)buf[0], (unsigned)len, (double)loraLastRssi());
    return;
  }

  digitalWrite(LED_BUILTIN, LOW);   // XIAO LED is active low
  if (buf[0] == ELEV_TAG_STATE) {
    handleState(buf, len, nowMs);
  } else {
    handleStats(buf, len, nowMs);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

// ---------------------------------------------------------------------------
// Mesh receive - see foreignOrigins above
// ---------------------------------------------------------------------------
static void onMeshFrame(const MeshFrame &frame, const uint8_t *srcMac, int8_t rssi) {
  foreignOrigins++;
  char mac[18];
  Serial.printf("[bridge] WARNING: mesh frame origSeq=%u type=0x%02X from %s "
                "(RSSI=%d dBm) that this bridge did not mint - a second origin "
                "on channel %d would collide in the origSeq space\n",
                (unsigned)frame.origSeq, (unsigned)frame.type,
                meshFormatMac(srcMac, mac), (int)rssi, (int)MESH_CHANNEL);
}

// ---------------------------------------------------------------------------
// Periodic summary
// ---------------------------------------------------------------------------
static void printSummary() {
  char uptimeText[24];
  fmtUptime(millis() / 1000u, uptimeText, sizeof(uptimeText));

  Serial.printf("[bridge] --- heard=%lu | STATE good=%lu lost=%lu (%.1f%%) "
                "| STATS good=%lu lost=%lu (%.1f%%) "
                "| dropped: crc=%lu radio=%lu tag=%lu len=%lu "
                "| dup=%lu resync=%lu | published=%lu failed=%lu | up %s",
                (unsigned long)heardCount,
                (unsigned long)stateSeq.good, (unsigned long)stateSeq.lost,
                (double)streamLossPercent(stateSeq),
                (unsigned long)statsSeq.good, (unsigned long)statsSeq.lost,
                (double)streamLossPercent(statsSeq),
                (unsigned long)radioCrcCount, (unsigned long)radioErrCount,
                (unsigned long)dropTagCount, (unsigned long)dropLenCount,
                (unsigned long)(stateSeq.dups + statsSeq.dups),
                (unsigned long)(stateSeq.resyncs + statsSeq.resyncs),
                (unsigned long)publishCount, (unsigned long)publishFails,
                uptimeText);
  if (foreignOrigins) {
    Serial.printf(" | foreignOrigins=%lu", (unsigned long)foreignOrigins);
  }
  // None of the counters above move when nothing arrives, so an unchanged
  // summary reads as healthy. Say otherwise explicitly.
  uint32_t silentMs = millis() - lastRxMs;
  if (silentMs > STATS_CADENCE_MS * 3u) {
    Serial.printf(" | SILENT for %lu s", (unsigned long)(silentMs / 1000u));
  }
  Serial.println(" ---");

  meshPrintCounters();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  loraBringUp("BRIDGE RX");

  int16_t state = loraStartReceive();
  if (state != RADIOLIB_ERR_NONE) {
    // Nothing this node does matters if it cannot hear the car, and every
    // later call would fail identically, so stop where the reason is visible.
    Serial.printf("[bridge] startReceive failed: %d (%s) - the bridge is deaf\n",
                  (int)state, loraStatusName(state));
    while (true) {
      delay(1000);
    }
  }

  // ESP-NOW comes up after the radio: meshBringUp starts WiFi, and doing that
  // while the SX1262 bring-up is still driving SPI is a needless overlap of
  // two peripherals' power-on transients on one 3V3 rail.
  meshBringUp(MESH_ROLE_ORIGIN, onMeshFrame, "BRIDGE");

  Serial.printf("[bridge] listening: SF%d BW%.1f kHz CR4/%d @ %.1f MHz "
                "-> ESP-NOW ch%d hop%d\n",
                (int)LORA_SPREADING_FACTOR, (double)LORA_BANDWIDTH,
                (int)LORA_CODING_RATE, (double)LORA_FREQUENCY,
                (int)MESH_CHANNEL, (int)MESH_HOP_LIMIT);

  lastRxMs     = millis();
  lastReportMs = millis();
}

void loop() {
  // First, unconditionally: relays whose jitter has expired are waiting in
  // here, and a frame held back while the loop does something else is latency
  // on every screen downstream of this node.
  meshService();

  uint8_t buf[RX_BUF_BYTES];
  size_t  len   = 0;
  int16_t state = RADIOLIB_ERR_NONE;

  if (loraPoll(buf, sizeof(buf), &len, &state)) {
    heardCount++;
    if (state == RADIOLIB_ERR_NONE) {
      handleFrame(buf, len);
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      radioCrcCount++;
      Serial.printf("[bridge] radio CRC mismatch (RSSI=%.1f dBm, SNR=%.1f dB)\n",
                    (double)loraLastRssi(), (double)loraLastSnr());
    } else {
      radioErrCount++;
      Serial.printf("[bridge] readData failed: %d (%s)\n",
                    (int)state, loraStatusName(state));
    }
  }

  if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = millis();
    printSummary();
  }
}
