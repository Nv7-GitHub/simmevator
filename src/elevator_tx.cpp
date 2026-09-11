//
// The car node: barometer in, floor out, LoRa on a battery.
//
// Everything the system knows about where the elevator is is decided here, on
// the one board that holds the sensor and therefore sees an unbroken sample
// stream (spec 1). The radio only carries the answer; losing a packet ages a
// display, it cannot corrupt a floor.
//
// ---------------------------------------------------------------------------
// Three clocks, and why they are not one
// ---------------------------------------------------------------------------
// 4 Hz  the barometer is read, and the reading is tested for stillness. This
//       clock exists only to notice that the car has started moving, so the
//       first STATE packet goes out about half a second after departure
//       instead of up to two seconds after it.
// 1 Hz  one of those readings - the freshest - is handed to FloorMonitor.
//       Every window length and rate threshold in ALGORITHM.md section 7 is
//       expressed in samples per second and was measured at 1 Hz, so feeding
//       the monitor at 4 Hz would quietly re-tune the whole algorithm. The
//       BMP390 free-runs at 25 Hz internally, so reading it four times a
//       second does not change what the once-a-second read returns.
// event STATE goes out while the car moves and for STATE_HOLD_AFTER_STOP_MS
//       after; STATS goes out on its own 60 s heartbeat regardless.
//
// ---------------------------------------------------------------------------
// Why the loop sleeps
// ---------------------------------------------------------------------------
// Spec 2.2 budgets ~3 mA light-sleeping against ~25 mA awake, and the whole
// 30-day figure rests on the node being asleep for almost all of every second.
// Deep sleep is not available: the algorithm needs the stream unbroken, and a
// deep-sleep wake is a reboot. So the loop does its due work and then sleeps
// to the next deadline with the SX1262 sleeping too - the radio idles at
// several mA in standby, which alone would be a third of the budget.
//
// The sleep is measured rather than assumed. esp_light_sleep_start() returns
// near the requested time, not at it, and the difference is the difference
// between the budget above and a battery that lasts three weeks - so the
// STATS line reports what was asked for and what was actually spent.
//
// ---------------------------------------------------------------------------
// Clocks and wrapping
// ---------------------------------------------------------------------------
// millis() wraps every 49.7 days and this node is meant to run for months, so
// the schedule runs on esp_timer_get_time(): 64-bit microseconds, monotonic
// across light sleep, and 584,000 years from wrapping. Every deadline is still
// compared as an unsigned difference, which is the form that survives a wrap
// wherever one can happen - nvsModelMaybeSave() still takes millis() and
// handles its own.
//

#include <Arduino.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <math.h>

#include "battery.h"
#include "bmp390_sensor.h"
#include "elev_packet.h"
#include "floor_monitor.h"
#include "lora_link.h"
#include "nvs_model.h"

// ---------------------------------------------------------------------------
// Schedule - the real values come from [env:elevator_tx] in platformio.ini,
// where the power-budget arithmetic behind each one is written down. These
// fallbacks only keep the file compilable on its own.
// ---------------------------------------------------------------------------
#ifndef FLOOR_SAMPLE_INTERVAL_MS
#define FLOOR_SAMPLE_INTERVAL_MS 1000
#endif
#ifndef STILLNESS_PROBE_INTERVAL_MS
#define STILLNESS_PROBE_INTERVAL_MS 250
#endif
#ifndef STATE_INTERVAL_MS
#define STATE_INTERVAL_MS 2000
#endif
#ifndef STATE_HOLD_AFTER_STOP_MS
#define STATE_HOLD_AFTER_STOP_MS 10000
#endif
#ifndef STATS_INTERVAL_MS
#define STATS_INTERVAL_MS 60000
#endif

// Which transmitter this is. One car, so one identity; the field exists so a
// second shaft does not need a second protocol.
#ifndef ELEV_TX_ID
#define ELEV_TX_ID 1
#endif

// Re-attempt sensor bring-up this often while it is missing, matching
// elevatormons' baro_tx. A node whose BMP390 comes up late still works.
#ifndef SENSOR_RETRY_INTERVAL_MS
#define SENSOR_RETRY_INTERVAL_MS 5000
#endif

// Core clock. The spec 2.2 transmit figure of 140 mA is the SX1262 at +22 dBm
// plus an ~22 mA core, which is 80 MHz - not the 240 MHz the board default
// leaves it at. RadioLib busy-polls DIO1 for the whole 297 ms of every packet,
// so the clock the core happens to be at is spent for the entire airtime. 80 is
// the floor: the USB-Serial-JTAG PHY stops working below it.
#ifndef TX_CPU_MHZ
#define TX_CPU_MHZ 80
#endif

// Backoff for an NVS that is failing every write. The first retry waits a
// second, then it doubles to a minute - long enough that a dead partition costs
// nothing, short enough that a transient recovers on its own.
#ifndef NVS_FAIL_BACKOFF_MIN_MS
#define NVS_FAIL_BACKOFF_MIN_MS 1000
#endif
#ifndef NVS_FAIL_BACKOFF_MAX_MS
#define NVS_FAIL_BACKOFF_MAX_MS 60000
#endif
#ifndef NVS_FAIL_PRINT_LIMIT
#define NVS_FAIL_PRINT_LIMIT 3
#endif

// Below this the entry and exit cost more than the sleep saves, so the loop
// just falls through and arrives at the deadline a moment early.
#define LIGHT_SLEEP_MIN_US 3000

#define US_PER_MS 1000ULL
#define US_PER_S  1000000ULL

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static FloorMonitor   monitor;
static FloorBroadcast bc;            // last broadcast, held across a dropped sample

static bool           sensorUp     = false;
static Bmp390Reading  probeSample;  // freshest 4 Hz reading
static bool           sampleFresh  = false;   // cleared by the 1 Hz step, so a
                                              // stale reading is never re-fed
static bool           sensorErr    = false;   // STATE bit 4 / STATS bit 0
static uint32_t       sensorSkips  = 0;

// Trailing window for the motion-onset probe. Same threshold and depth as the
// algorithm's own stillness test, but running at 4 Hz it spans 1.25 s rather
// than 5 s. It is not a second opinion on where the car is - FloorMonitor's
// `moving` stays authoritative - it only decides when to start transmitting.
static double   stillRing[FLOOR_STILL_WIN];
static uint8_t  stillCount = 0;

// The STATE stream runs until this deadline, which every sign of motion pushes
// out. holdActive distinguishes "the hold has expired" from "the node has only
// just booted", which an all-zero deadline cannot.
static uint64_t holdUntilUs = 0;
static bool     holdActive  = false;

static uint64_t lastProbeUs = 0;
static uint64_t lastFloorUs = 0;
static uint64_t lastStateUs = 0;
static uint64_t lastStatsUs = 0;
static uint32_t lastRetryMs = 0;
static uint32_t nvsFailBackoffMs = 0;   // 0 = healthy, no backoff in effect
static uint32_t nvsLastFailMs    = 0;
static uint32_t nvsFailCount     = 0;

static uint8_t  stateSeq = 0;   // one counter per format: the two streams run at
static uint8_t  statsSeq = 0;   // different cadences, so a shared seq would make
                                // per-stream loss impossible to measure
static uint32_t stateSent = 0;
static uint32_t statsSent = 0;
static uint32_t txFails   = 0;

// Sleep accounting for the window since the last STATS line.
static uint64_t winSleptUs = 0;
static uint64_t winReqUs   = 0;
static uint32_t winNaps    = 0;
static uint32_t sleepRejects = 0;
static uint64_t winStartUs = 0;

// Changes worth persisting, watched rather than hooked: FloorMonitor has no
// callback, and a confirmed stop is exactly when these move.
static uint32_t seenTrips  = 0;
static uint32_t seenStops  = 0;
static bool     seenReady  = false;

static uint8_t packetBuf[ELEV_MAX_PACKET_BYTES];

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline uint64_t nowUs() {
  return (uint64_t)esp_timer_get_time();
}

// Advances the deadline rather than restarting it from now, so a cadence does
// not drift by however long the rest of the loop took. If something held the
// loop up for more than a whole interval - a 494 ms STATS transmit against the
// 250 ms probe - the schedule skips ahead instead of firing a burst of
// catch-up work with bunched timestamps.
static bool due(uint64_t now, uint64_t *last, uint64_t interval) {
  if (now - *last < interval) return false;
  *last += interval;
  if (now - *last >= interval) *last = now;
  return true;
}

static uint8_t clampU8(long v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

static uint16_t clampU16(double v) {
  if (!(v > 0.0)) return 0;
  if (v > 65535.0) return 65535;
  return (uint16_t)lround(v);
}

static uint32_t clampU32(double v) {
  if (!(v > 0.0)) return 0;
  if (v > 4294967295.0) return 4294967295u;
  return (uint32_t)v;
}

static int16_t clampI16(double v) {
  if (!isfinite(v)) return 0;
  if (v < -32768.0) return -32768;
  if (v > 32767.0) return 32767;
  return (int16_t)lround(v);
}

// FloorDirection and ElevDirection happen to share their numbering. Mapped
// rather than cast so that the wire format and the algorithm stay free to
// diverge without a silent off-by-one between them.
static ElevDirection wireDirection(FloorDirection d) {
  switch (d) {
    case FLOOR_DIR_UP:   return ELEV_DIR_UP;
    case FLOOR_DIR_DOWN: return ELEV_DIR_DOWN;
    case FLOOR_DIR_IDLE: break;
  }
  return ELEV_DIR_IDLE;
}

static const char *directionName(FloorDirection d) {
  switch (d) {
    case FLOOR_DIR_UP:   return "up";
    case FLOOR_DIR_DOWN: return "down";
    case FLOOR_DIR_IDLE: break;
  }
  return "idle";
}

// ---------------------------------------------------------------------------
// Transmit
// ---------------------------------------------------------------------------
static bool transmit(size_t len, const char *what) {
  uint64_t start = nowUs();
  int16_t state = loraTransmit(packetBuf, len);
  uint32_t airMs = (uint32_t)((nowUs() - start) / US_PER_MS);

  // Back to sleep immediately: standby is several mA and the next packet is at
  // least two seconds away. loraTransmit() wakes the radio itself.
  loraSleep();

  if (state != RADIOLIB_ERR_NONE) {
    txFails++;
    Serial.printf("[tx] %s FAILED %d (%s) after %lu ms\n", what, state,
                  loraStatusName(state), (unsigned long)airMs);
    return false;
  }
  return true;
}

static void sendState(uint64_t now, const char *why) {
  ElevState s;
  s.txId  = ELEV_TX_ID;
  s.seq   = stateSeq++;
  // 0 is the wire's "model not ready". A fresh node withholds the floor rather
  // than broadcasting one it has not earned - ALGORITHM.md section 8.
  s.floor = bc.modelReady ? clampU8(bc.floor) : 0;
  s.posQ8 = bc.modelReady
                ? clampI16(bc.posFloors * (double)ELEV_POS_UNITS_PER_FLOOR)
                : 0;
  s.state = elevStateByte(bc.moving, wireDirection(bc.direction), bc.modelReady,
                          sensorErr);
  s.confidence = clampU8(lround(bc.confidence * 255.0));

  size_t len = elevStatePack(packetBuf, sizeof(packetBuf), &s);
  if (len == 0) return;   // only reachable through a programming error

  lastStateUs = now;
  if (!transmit(len, "STATE")) return;
  stateSent++;

  Serial.printf("[tx] STATE seq=%u floor=%u pos=%+.2f %s%s conf=%u (%s)\n",
                s.seq, s.floor, (double)s.posQ8 / ELEV_POS_UNITS_PER_FLOOR,
                bc.moving ? "moving " : "stopped ", directionName(bc.direction),
                s.confidence, why);
}

static void sendStats(uint64_t now) {
  // Once per STATS packet, not per loop: the divider is always connected and
  // the pack moves on a timescale of hours, so a minute-old reading is as good
  // as a fresh one and costs nothing.
  uint16_t mv = batteryReadMv();

  ElevStats s;
  s.txId       = ELEV_TX_ID;
  s.seq        = statsSeq++;
  s.batteryMv  = mv;
  s.dist24hM   = clampU16(bc.dist24hM);
  s.distTotalM = clampU32(bc.distanceM);
  s.pitchMm    = clampU16(bc.pitch * (double)ELEV_MM_PER_M);
  s.nFloors    = clampU8(bc.nFloors);
  s.trips      = clampU16((double)bc.trips);
  s.stops      = clampU16((double)bc.stops);
  s.uptimeS    = (uint32_t)(now / US_PER_S);
  s.flags      = elevStatsFlagsByte(sensorErr, bc.modelReady, nvsModelRestored(),
                                    batteryIsLow(mv));
  s.tempC      = (int8_t)constrain(lround(probeSample.temperatureC), -128L, 127L);

  size_t len = elevStatsPack(packetBuf, sizeof(packetBuf), &s);
  if (len == 0) return;

  lastStatsUs = now;
  if (transmit(len, "STATS")) {
    statsSent++;
    Serial.printf("[tx] STATS seq=%u %u mV%s %.1f C | pitch=%u mm floors=%u "
                  "trips=%u stops=%u | %lu m total, %u m/24h | up %lu s "
                  "flags=0x%02X\n",
                  s.seq, s.batteryMv, batteryIsLow(mv) ? " LOW" : "",
                  probeSample.temperatureC, s.pitchMm, s.nFloors, s.trips,
                  s.stops, (unsigned long)s.distTotalM, s.dist24hM,
                  (unsigned long)s.uptimeS, s.flags);
  }

  // The number the 30-day figure rests on. Requested against measured is the
  // part that cannot be assumed: the wake path costs what it costs.
  uint64_t windowUs = now - winStartUs;
  double pct = windowUs ? (100.0 * (double)winSleptUs / (double)windowUs) : 0.0;
  Serial.printf("[tx] sleep %.2f/%.2f s of %.2f s (%.1f%%) in %lu naps, "
                "%+.0f ms vs request%s | tx ok=%lu/%lu fail=%lu skips=%lu\n",
                winSleptUs / 1e6, winReqUs / 1e6, windowUs / 1e6, pct,
                (unsigned long)winNaps,
                ((double)winSleptUs - (double)winReqUs) / 1000.0,
                sleepRejects ? " (rejects seen)" : "",
                (unsigned long)stateSent, (unsigned long)statsSent,
                (unsigned long)txFails, (unsigned long)sensorSkips);

  winSleptUs = 0;
  winReqUs   = 0;
  winNaps    = 0;
  winStartUs = now;
}

// ---------------------------------------------------------------------------
// The 4 Hz probe
// ---------------------------------------------------------------------------
// How often the barometer is read. 4 Hz exists solely to catch motion ONSET;
// once the STATE stream is already running that job is done, and three of every
// four wakeups would do nothing but read a sample the 1 Hz step never looks at.
// At evening-peak traffic the hold is active ~90% of the time, so this is most
// of the probe's wakeups. It returns to 4 Hz the moment the hold expires, which
// is what keeps the next onset fast.
static uint64_t probeIntervalUs() {
  return (uint64_t)(holdActive ? FLOOR_SAMPLE_INTERVAL_MS
                               : STILLNESS_PROBE_INTERVAL_MS) * US_PER_MS;
}

static void pushStill(double alt) {
  // bmp390Read() calls a reading valid once the conversion returns, but the
  // altitude on top of it is a powf over the pressure ratio and comes back
  // non-finite if that ratio ever goes bad. One of those in the ring makes
  // stdPopulation() non-finite for the next 1.25 s, which fails the stillness
  // comparison and fires an onset packet for a car that never moved.
  if (!isfinite(alt)) return;
  if (stillCount < FLOOR_STILL_WIN) {
    stillRing[stillCount++] = alt;
    return;
  }
  for (uint8_t i = 1; i < FLOOR_STILL_WIN; i++) stillRing[i - 1] = stillRing[i];
  stillRing[FLOOR_STILL_WIN - 1] = alt;
}

static void noteMotion(uint64_t now) {
  holdUntilUs = now + (uint64_t)STATE_HOLD_AFTER_STOP_MS * US_PER_MS;
  holdActive  = true;
}

static void probe(uint64_t now) {
  Bmp390Reading r = bmp390Read();
  if (!r.valid) {
    // Not stored and not pushed into the ring. A failed conversion has no
    // altitude in it, and feeding the last good one forward would read as the
    // car holding perfectly still - which is exactly the wrong conclusion
    // while the sensor is out.
    return;
  }
  probeSample = r;
  sampleFresh = true;
  pushStill(r.altitudeM);

  if (holdActive) return;   // already transmitting; the 1 Hz monitor has it now
  if (stillCount < FLOOR_STILL_WIN) return;

  if (floor_detail::stdPopulation(stillRing, FLOOR_STILL_WIN) <= FLOOR_STILL_STD_M) {
    return;
  }

  // The whole reason this clock exists. Deferring this packet to the next 2 s
  // slot boundary would throw away the latency the 4 Hz probe was added to buy.
  noteMotion(now);
  Serial.printf("[tx] motion onset at %.2f s\n", now / 1e6);
  sendState(now, "onset");
}

// ---------------------------------------------------------------------------
// The 1 Hz algorithm step
// ---------------------------------------------------------------------------
static void stepAlgorithm(uint64_t now) {
  if (!sampleFresh) {
    sensorSkips++;
    if (!sensorErr) {
      Serial.printf("[tx] sensor error - holding floor %d (%lu skipped)\n",
                    bc.floor, (unsigned long)sensorSkips);
    }
    sensorErr = true;
    return;   // bc is deliberately left as it was: a held floor is what a
              // display should show while the sensor is out, not a zero
  }
  sampleFresh = false;

  bool wasErr    = sensorErr;
  int  wasFloor  = bc.floor;
  bool wasMoving = bc.moving;

  bc = monitor.update(now / (double)US_PER_S, probeSample.altitudeM);
  // The sample can reach here finite-looking and still be dropped inside
  // update(), which holds the previous broadcast and raises its own flag.
  // Reading that flag back is what puts a NaN altitude on the wire as
  // sensorErr instead of reporting a healthy sensor over a frozen floor.
  sensorErr = monitor.sensorError();

  if (wasErr && !sensorErr) {
    Serial.printf("[tx] sensor recovered at %.2f s\n", now / 1e6);
  }
  if (bc.moving) {
    noteMotion(now);
  }
  if (bc.moving != wasMoving || bc.floor != wasFloor) {
    Serial.printf("[tx] floor=%d %s %s pos=%+.2f conf=%.2f | pitch=%.3f m "
                  "floors=%d rise=%.2f m %s\n",
                  bc.floor, bc.moving ? "moving" : "stopped",
                  directionName(bc.direction), bc.posFloors, bc.confidence,
                  bc.pitch, bc.nFloors, bc.rise,
                  bc.modelReady ? "" : "(model not ready)");
  }

  // A confirmed stop is the only thing that changes the ladder, the stop counts
  // or the odometer in a way worth carrying across a reboot.
  if (bc.trips != seenTrips || bc.stops != seenStops || bc.modelReady != seenReady) {
    seenTrips = bc.trips;
    seenStops = bc.stops;
    seenReady = bc.modelReady;
    nvsModelMarkDirty();
  }

  // Offered once a second; the throttle inside decides whether this is the one
  // in three hundred that actually writes.
  //
  // An NVS that is permanently unhealthy - a missing partition, a corrupt
  // namespace - fails every single offer. Without the backoff below that is a
  // full record write and a console line once a second for the life of the
  // deployment: the write is wasted current, and the console becomes unreadable
  // exactly when someone is trying to read it to find out what is wrong. So a
  // failure doubles the retry gap up to a ceiling, and only the first few are
  // printed. A success resets both.
  if (nvsFailBackoffMs == 0 || millis() - nvsLastFailMs >= nvsFailBackoffMs) {
    NvsModelStatus ns = nvsModelMaybeSave(millis(), &monitor);
    if (ns == NVS_MODEL_SKIPPED) {
      // Not an outcome, just the throttle declining. Leaves the backoff alone.
    } else if (ns == NVS_MODEL_OK) {
      if (nvsFailBackoffMs) {
        Serial.printf("[tx] nvs recovered after %lu failures\n",
                      (unsigned long)nvsFailCount);
      }
      nvsFailBackoffMs = 0;
      nvsFailCount = 0;
      Serial.printf("[tx] nvs save: %s (write %lu)\n", nvsModelStatusName(ns),
                    (unsigned long)nvsModelWriteCount());
    } else {
      nvsLastFailMs = millis();
      nvsFailBackoffMs = nvsFailBackoffMs ? nvsFailBackoffMs * 2
                                          : NVS_FAIL_BACKOFF_MIN_MS;
      if (nvsFailBackoffMs > NVS_FAIL_BACKOFF_MAX_MS) {
        nvsFailBackoffMs = NVS_FAIL_BACKOFF_MAX_MS;
      }
      nvsFailCount++;
      if (nvsFailCount <= NVS_FAIL_PRINT_LIMIT) {
        Serial.printf("[tx] nvs save failed: %s (%lu, next retry in %lu ms)%s\n",
                      nvsModelStatusName(ns), (unsigned long)nvsFailCount,
                      (unsigned long)nvsFailBackoffMs,
                      nvsFailCount == NVS_FAIL_PRINT_LIMIT ? " - silencing" : "");
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Sleep
// ---------------------------------------------------------------------------
static uint64_t nextDeadline(uint64_t now) {
  uint64_t d = lastProbeUs + probeIntervalUs();

  uint64_t f = lastFloorUs + (uint64_t)FLOOR_SAMPLE_INTERVAL_MS * US_PER_MS;
  if (f < d) d = f;

  uint64_t st = lastStatsUs + (uint64_t)STATS_INTERVAL_MS * US_PER_MS;
  if (st < d) d = st;

  if (holdActive) {
    uint64_t s = lastStateUs + (uint64_t)STATE_INTERVAL_MS * US_PER_MS;
    if (s < d) d = s;
    // The hold expiring is itself an event - it is what stops the stream and
    // prints the line saying so.
    if (holdUntilUs < d) d = holdUntilUs;
  }

  return (d > now) ? d : now;
}

static void sleepUntil(uint64_t deadline) {
  uint64_t now = nowUs();
  if (deadline <= now) return;
  uint64_t want = deadline - now;
  if (want < LIGHT_SLEEP_MIN_US) return;

  // Standby is several mA - a third of the whole budget - so the radio goes
  // down before the CPU does, every time.
  if (!loraIsAsleep()) loraSleep();

  // The USB CDC peripheral loses its clock across a light sleep, so anything
  // still in the FIFO would come out truncated after the wake.
  Serial.flush();

  esp_sleep_enable_timer_wakeup(want);
  uint64_t before = nowUs();
  esp_err_t err = esp_light_sleep_start();
  uint64_t after = nowUs();

  if (err != ESP_OK) {
    sleepRejects++;
    return;
  }
  winSleptUs += after - before;
  winReqUs   += want;
  winNaps++;
}

// ---------------------------------------------------------------------------
void setup() {
  // Before anything else: every current figure in spec 2.2 assumes this clock.
  setCpuFrequencyMhz(TX_CPU_MHZ);

  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }

  loraBringUp("ELEVATOR TX");
  loraSleep();

  // Not halting on a sensor failure, for the reason elevatormons' baro_tx does
  // not: setup() runs seconds after reset, long before a monitor can attach, so
  // a node that reports a fault here and stops looks exactly like a dead board.
  // loop() retries and re-reports instead.
  sensorUp = bmp390BringUp("ELEVATOR TX SENSOR");

  batteryBegin();

  if (!nvsModelBegin()) {
    Serial.println("[tx] nvs unavailable - the model will not survive a reboot");
  } else {
    NvsModelStatus ns = nvsModelLoad(&monitor);
    if (ns == NVS_MODEL_OK) {
      // Restored, so modelReady is true from the first sample and a battery
      // swap does not restart the bootstrap - spec 5.1.
      Serial.printf("[tx] model restored: pitch=%.3f m floors=%d trips=%lu "
                    "stops=%lu %.0f m total\n",
                    monitor.pitch(), monitor.nFloors(),
                    (unsigned long)monitor.trips(), (unsigned long)monitor.stops(),
                    monitor.distanceM());
    } else {
      Serial.printf("[tx] cold boot: %s%s\n", nvsModelStatusName(ns),
                    nvsModelStatusIsFault(ns) ? " - stored record discarded" : "");
    }
  }

  seenTrips = monitor.trips();
  seenStops = monitor.stops();
  seenReady = monitor.modelReady();

  uint16_t mv = batteryReadMv();
  Serial.printf("[tx] txId=%d  probe %d ms / algorithm %d ms / STATE %d ms "
                "(+%d ms hold) / STATS %d ms  battery %u mV%s\n",
                ELEV_TX_ID, STILLNESS_PROBE_INTERVAL_MS, FLOOR_SAMPLE_INTERVAL_MS,
                STATE_INTERVAL_MS, STATE_HOLD_AFTER_STOP_MS, STATS_INTERVAL_MS,
                mv, batteryIsLow(mv) ? " LOW" : "");

  uint64_t now = nowUs();
  lastProbeUs = now;
  lastFloorUs = now;
  lastStateUs = now;
  winStartUs  = now;
  lastRetryMs = millis();
  // Deliberately back-dated so the first STATS goes out immediately: a node
  // that has just booted should say so rather than staying silent for a minute.
  lastStatsUs = now - (uint64_t)STATS_INTERVAL_MS * US_PER_MS;
}

void loop() {
  uint64_t now = nowUs();

  if (!sensorUp) {
    // Nothing probes while the sensor is down, but the probe deadline still
    // has to move: nextDeadline() takes the earliest of them, and one stuck in
    // the past makes it return `now` every pass. sleepUntil() then declines
    // every nap and the node spins at full current between bring-up retries -
    // the one state where it can least afford to.
    (void)due(now, &lastProbeUs, probeIntervalUs());
    uint32_t nowMs = millis();
    if (nowMs - lastRetryMs >= SENSOR_RETRY_INTERVAL_MS) {
      lastRetryMs = nowMs;
      sensorUp = bmp390BringUp("ELEVATOR TX SENSOR RETRY");
    }
  } else if (due(now, &lastProbeUs, probeIntervalUs())) {
    probe(now);
  }

  if (due(now, &lastFloorUs, (uint64_t)FLOOR_SAMPLE_INTERVAL_MS * US_PER_MS)) {
    stepAlgorithm(now);
  }

  if (holdActive) {
    if (now >= holdUntilUs) {
      holdActive = false;
      Serial.printf("[tx] STATE stream off at %.2f s\n", now / 1e6);
    } else if (now - lastStateUs >= (uint64_t)STATE_INTERVAL_MS * US_PER_MS) {
      sendState(now, "cadence");
    }
  }

  if (now - lastStatsUs >= (uint64_t)STATS_INTERVAL_MS * US_PER_MS) {
    sendStats(now);
  }

  sleepUntil(nextDeadline(nowUs()));
}
