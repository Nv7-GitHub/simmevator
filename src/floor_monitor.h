#pragma once
//
// On-device elevator floor detection from barometric altitude.
//
// A line-for-line port of elevatormons/tools/floor_algorithm.py, documented in
// elevatormons/docs/ALGORITHM.md. The two are meant to stay diffable by eye:
// every function here has the same name and the same body order as the Python,
// and every tuning constant in section 7 of ALGORITHM.md appears below with the
// value it was measured at. A divergence from the reference is a bug in this
// file, not an improvement - the constants are tuned to one building and one
// sensor and re-tuning them invalidates the measured performance.
//
// The two ideas the algorithm rests on:
//
//   Floors come from jumps. A trip takes 5-60 s, over which the weather moves
//   the reference by under 0.05 m, so round(jump / pitch) is safe. Absolute
//   altitude never decides a floor - over the 3 h reference capture the
//   sea-level reference wandered 9.9 m, more than three floor heights.
//
//   The reference follows the weather, slowly. While parked it is nudged toward
//   the measurement at a hard cap of 0.02 m/s, about 30x slower than the slowest
//   real elevator and 25x faster than the fastest storm ramp, so drift is
//   absorbed and motion never is.
//
// ---------------------------------------------------------------------------
// What this file is allowed to depend on
// ---------------------------------------------------------------------------
// Nothing but <stdint.h>, <string.h> and <math.h>. No Arduino.h, no allocation,
// no exceptions, no STL. The `native` environment therefore compiles the exact
// code the ESP32 runs, which is the whole point of the replay harness in sim/:
// a host-only reimplementation would only prove that the reimplementation works.
//
// ---------------------------------------------------------------------------
// Four places a port silently diverges from the Python
// ---------------------------------------------------------------------------
//   * double, never float. trustedRate() computes med^2/(med^2 + (k*mad)^2) on
//     numbers around 1e-4; squaring those costs 8 decimal digits and float32
//     does not have them to spend.
//   * np.std is the population standard deviation (ddof = 0). The sample std is
//     12% larger over a 5-window, which would move the stillness threshold off
//     the gap it was placed in.
//   * np.median averages the two middle values on even-length input. The rate
//     ring holds an even count while it fills, so this case is reached.
//   * numpy's arange accumulates rather than computing lo + i*step, and both
//     np.round and Python's round() break ties to even. estimatePitch()
//     reproduces both; see the notes there.
//
// ---------------------------------------------------------------------------
// The dicts
// ---------------------------------------------------------------------------
// `heights` and `stop_counts` are Python dicts keyed by a signed floor index
// that starts at 0 and may go negative before the ladder is re-based. Here they
// are 64-entry arrays at origin +32 plus an occupancy bitmap - enough for 32
// floors below the boot landing and 31 above it, against a building of 10. An
// index outside that range clamps and raises tableClamped rather than growing:
// on a microcontroller the alternative to a bound is a crash, and an index that
// far out is a runaway model rather than a real floor. Reads do not clamp: a
// lookup outside the range falls back to the nominal index x pitch, because
// handing back a neighbouring floor's learned height is a wrong answer that
// looks like a right one.
//

#include <math.h>
#include <stdint.h>
#include <string.h>

// ===========================================================================
// Tuning constants - ALGORITHM.md section 7. Do not re-tune.
// ===========================================================================

// Sits in the gap between ~0.05 m rms stop noise and a car running at up to
// 1.78 m/s. A 5-sample std separates those by more than an order of magnitude.
#define FLOOR_STILL_STD_M 0.08

// Trailing window, so it has to fit inside the shortest dwell. Dwells here run
// as short as 5 s, which is why it is 5 and not more.
#define FLOOR_STILL_WIN 5

// Below this an apparent trip is a door cycle on the same floor.
#define FLOOR_MIN_JUMP_M 1.2

// Parked-reference follow cap. 30x slower than the slowest real elevator and
// 25x faster than the fastest storm ramp, so it absorbs drift and never motion.
// It only corrects while parked - 28.8% of the time here - so its effective
// tracking rate is 2.46 hPa/h, which is exactly where the uncompensated version
// started failing. The feed-forward rate is what removes that duty-cycle ceiling.
#define FLOOR_MAX_DRIFT_MPS 0.02

// Let the post-stop shaft transient decay before the dwell rate measurement
// takes its first reading.
#define FLOOR_DWELL_SKIP_S 4.0

// Long baselines only: a shorter dwell carries about 1 hPa/h of the relaxation
// transient, undiluted.
#define FLOOR_MIN_DWELL_RATE_S 30.0

// Median depth for the weather-rate ring. One bad observation cannot steer it.
#define FLOOR_RATE_HISTORY 7

// Shortest useful gap between two visits to the same floor. Raising it to 60 s
// pushed the first usable rate estimate from 6.7 min out to 13.4 min.
#define FLOOR_REVISIT_MIN_S 30.0

// Beyond this the weather rate has probably changed, so the pair says nothing.
#define FLOOR_REVISIT_MAX_S 900.0

// How much scatter discredits the rate estimate. Raising it shrinks harder
// toward zero; 0 would trust the median outright, which costs more than it
// gains as soon as there is any gust at all (54% -> 9% in one ungated test).
#define FLOOR_RATE_TRUST_K 1.0

// Discard the last samples of a dwell: the trailing window starts to see the
// departure before the stillness test trips, and departures are directional
// (every lobby exit is upward), so keeping them biases the rate.
#define FLOOR_DWELL_TAIL_N 3

// ~21 hPa/h. Beyond this it is not weather.
#define FLOOR_DRIFT_RATE_CAP 0.05

// Trips observed before the pitch is trusted. Reached about 5 min from boot.
#define FLOOR_BOOTSTRAP_JUMPS 12

// A "stop" this far off the floor grid in units of pitch is the car coasting
// through, not a landing. Without this the odometer over-counts badly: a car
// decelerating through a floor sits briefly at a near-constant height and a
// plain stillness test calls that an arrival.
#define FLOOR_OFF_LATTICE 0.30

// ...unless the still run persists this many samples, at which point it was a
// landing after all and the tracker resyncs to it.
#define FLOOR_RESYNC_S 25.0

// Still samples a dwell must hold before the landing counts on the odometer and
// teaches the building model. 356 floor-to-floor moves in the reference capture,
// of which 244 were served stops and 100 were coasted through.
#define FLOOR_CONFIRM_N 3

// Running-mean gain for learned landing heights.
#define FLOOR_HEIGHT_ALPHA 0.25

// Gain at which each trip's implied spacing blends into the running pitch.
#define FLOOR_PITCH_GAIN 0.05

// estimatePitch sweep bounds. 2.2-4.0 m covers every plausible floor spacing;
// 0.5 mm steps are an order finer than the +/-1 mm the pitch is checked to.
#define FLOOR_PITCH_LO   2.2
#define FLOOR_PITCH_HI   4.0
#define FLOOR_PITCH_STEP 5e-4

// ===========================================================================
// Fixed-size stand-ins for the Python containers
// ===========================================================================

// 64 slots at origin +32 - see the dicts note in the file header.
#define FLOOR_TABLE_N      64
#define FLOOR_TABLE_ORIGIN 32

// Room for the stillness ring if STILL_WIN is ever raised at construction.
#define FLOOR_STILL_WIN_MAX 16

// 24 rolling one-hour buckets. There is no RTC, so they are indexed by
// uptime/3600 mod 24 and advancing into a bucket clears it: "last 24 h" means
// "the last 24 hourly buckets", which is what the display claims. A reboot
// starts the window empty - restore() does not load them, because uptime
// restarts with it and a bucket from before the reboot cannot be aged.
#define FLOOR_ODO_BUCKETS 24

// Bumped whenever the layout of FloorModelState changes. nvs_model.cpp stores
// this alongside a CRC and discards a record that disagrees.
// 2: added tableClamped, so a runaway model cannot be laundered clean through
// a save/restore cycle.
#define FLOOR_STATE_SCHEMA 2

// ===========================================================================
// Small numeric helpers - each matching a specific numpy or Python behaviour
// ===========================================================================
namespace floor_detail {

// Guards a caller that hands the bounds over in the wrong order - which here
// means a negative dt reaching the +/-maxDrift*dt window. np.clip returns a_max
// in that case and the Python therefore limps on; the plain two-comparison form
// below would return whichever bound it tested first, which is a different
// answer for the same input. Swapping costs one predictable branch and makes
// the two agree on a case neither should ever see.
static inline double clampD(double v, double lo, double hi) {
  if (hi < lo) { double t = lo; lo = hi; hi = t; }
  return v < lo ? lo : (v > hi ? hi : v);
}

// Python's round() and np.round() both break exact ties to even. A tie is a
// measure-zero event on a real jump/pitch ratio, but the one time it happens it
// is a single-sample floor disagreement with no other symptom, so it is worth
// five lines to rule out. Written out rather than nearbyint(), whose answer
// depends on the runtime rounding mode, or lround(), which rounds ties away
// from zero.
static inline double roundHalfEven(double v) {
  double f = floor(v);
  double d = v - f;
  if (d > 0.5) return f + 1.0;
  if (d < 0.5) return f;
  return (fmod(f, 2.0) == 0.0) ? f : f + 1.0;   // exact tie: to even
}

// Insertion sort. n is at most FLOOR_STILL_WIN_MAX, so anything cleverer is
// slower.
static inline void sortSmall(double *a, int n) {
  for (int i = 1; i < n; i++) {
    double k = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; }
    a[j + 1] = k;
  }
}

// np.median: the mean of the two middle values on even-length input. The rate
// ring holds an even count while it is filling, so this branch is live.
static inline double median(const double *src, int n) {
  if (n <= 0) return 0.0;
  double a[FLOOR_STILL_WIN_MAX > FLOOR_RATE_HISTORY ? FLOOR_STILL_WIN_MAX
                                                   : FLOOR_RATE_HISTORY];
  for (int i = 0; i < n; i++) a[i] = src[i];
  sortSmall(a, n);
  if (n & 1) return a[n / 2];
  return 0.5 * (a[n / 2 - 1] + a[n / 2]);
}

// np.std with ddof = 0. The sample std would be 12% larger over a 5-window,
// which is enough to move the stillness threshold off the gap it sits in.
static inline double stdPopulation(const double *src, int n) {
  if (n <= 0) return 0.0;
  double mean = 0.0;
  for (int i = 0; i < n; i++) mean += src[i];
  mean /= (double)n;
  double acc = 0.0;
  for (int i = 0; i < n; i++) {
    double d = src[i] - mean;
    acc += d * d;
  }
  return sqrt(acc / (double)n);
}

}  // namespace floor_detail

// ===========================================================================
// What the node puts on the air
// ===========================================================================

// Direction is for the display arrow only. The values match the state-byte
// encoding in the LoRa STATE packet (bits 1-2).
enum FloorDirection {
  FLOOR_DIR_IDLE = 0,
  FLOOR_DIR_UP   = 1,
  FLOOR_DIR_DOWN = 2
};

// Everything the Python Broadcast carries, plus the three things the
// transmitter needs that an offline analysis did not: a live position, a
// direction, and a 24 h odometer window.
//
// Doubles are carried unrounded. The Python rounds inside _out() for its CSV;
// here the formatting is the replay harness's business, and rounding on the way
// out of the algorithm would be a lossy step in the middle of the port.
struct FloorBroadcast {
  double t;
  int    floor;          // 1-based, re-based so the lowest landing ever seen is 1
  bool   moving;
  double confidence;     // 0..1, how cleanly the last jump landed on the lattice
  int    nFloors;
  double pitch;
  double rise;
  uint32_t trips;
  uint32_t stops;
  double distanceM;
  double drift;          // how far the reference has been pulled since boot
  double driftRateMps;   // measured weather rate
  double datum;          // current estimate of the lowest landing's raw altitude

  // Live fractional position in floors relative to floor 1, (level - datum)/pitch.
  // ALGORITHM.md section 6 shows the tracked datum is unreliable for *deciding*
  // a floor - its phase detector works modulo the pitch, so a datum that lags
  // half a floor slips silently. So this is for display animation only, and the
  // display always snaps to `floor` on arrival.
  double posFloors;

  FloorDirection direction;
  double dist24hM;       // sum of the 24 rolling hourly buckets
  bool   modelReady;     // pitch established; before this the floor is withheld
};

// ===========================================================================
// Persistent model - what nvs_model.cpp writes every 5 minutes
// ===========================================================================
//
// Plain old data with no pointers, so the persistence layer can treat it as a
// byte blob and needs to know nothing about the algorithm. It carries a schema
// version; nvs_model adds a CRC and treats a mismatch in either as a cold boot.
//
// Restoring this sets modelReady immediately, so a reboot or a battery swap
// does not re-run the 5-minute bootstrap. A genuinely fresh unit still withholds
// output until the pitch is established, per ALGORITHM.md section 8.
struct FloorModelState {
  uint16_t schemaVersion;
  uint8_t  pitchSet;
  uint8_t  refSet;

  double   pitch;
  double   ref;
  int16_t  floor;

  uint64_t occupied;                    // bit k set = heights[k] has a value
  double   heights[FLOOR_TABLE_N];
  uint16_t stopCounts[FLOOR_TABLE_N];

  double   distanceM;
  uint32_t trips;
  uint32_t stops;

  // Written as part of the snapshot but deliberately not loaded - see restore().
  double   odoBuckets[FLOOR_ODO_BUCKETS];
  uint32_t odoBucket;                   // bucket the odometer was last in
  uint8_t  odoBucketSet;

  // An index ran off the end of the 64-slot table before this was saved. The
  // whole record is then suspect, so restore() refuses it. Carrying it is the
  // point: without it the flag is cleared by the round trip while restored_ is
  // set, which declares the model ready and suppresses the bootstrap that would
  // otherwise have rebuilt it.
  uint8_t  tableClamped;
};

// ===========================================================================
// FloorMonitor - causal floor tracker plus running building model
// ===========================================================================
class FloorMonitor {
 public:
  FloorMonitor() { reset(); }

  // pitchHint seeds the pitch so a unit with a known building skips the
  // bootstrap. Pass 0 for the normal case of learning it.
  void configure(double stillStd, int win, double maxDriftMps, double alpha,
                 double pitchHint) {
    stillStd_ = stillStd;
    win_ = (win > FLOOR_STILL_WIN_MAX) ? FLOOR_STILL_WIN_MAX : (win < 1 ? 1 : win);
    maxDriftMps_ = maxDriftMps;
    alpha_ = alpha;
    pitchHint_ = pitchHint;
    pitchHintSet_ = (pitchHint != 0.0);
  }

  // Cold boot: configuration back to the measured defaults, everything learned
  // thrown away.
  void reset() {
    memset(this, 0, sizeof(*this));
    stillStd_ = FLOOR_STILL_STD_M;
    win_ = FLOOR_STILL_WIN;
    maxDriftMps_ = FLOOR_MAX_DRIFT_MPS;
    alpha_ = FLOOR_HEIGHT_ALPHA;
    confidence_ = 1.0;
    out(0.0);     // seeds last_, in case the very first sample is the bad one
  }

  // Same, but keeps whatever configure() was told. This is what restore() runs
  // before loading a record, so a stored model cannot silently revert a unit
  // that was built with a pitch hint or a different window.
  void clearState() {
    const double sS = stillStd_, mD = maxDriftMps_, al = alpha_, ph = pitchHint_;
    const int wn = win_;
    const bool phs = pitchHintSet_;
    memset(this, 0, sizeof(*this));
    stillStd_ = sS;
    win_ = wn;
    maxDriftMps_ = mD;
    alpha_ = al;
    pitchHint_ = ph;
    pitchHintSet_ = phs;
    confidence_ = 1.0;
    out(0.0);
  }

  // ---- the loop ---------------------------------------------------------
  // One sample in, one broadcast out. No batch fit, no lookahead: every number
  // reported at time t was computable at time t.
  //
  // `t` is uptime in seconds and `alt` is altitude in metres. The odometer's
  // hour buckets are indexed off `t`, which is why it is uptime and not an
  // arbitrary epoch.
  FloorBroadcast update(double t, double alt) {
    // A BMP390 conversion that fails mid-read hands back a non-finite altitude,
    // and one of those is permanent damage rather than one bad sample: the
    // first one is stored straight into ref_, every clamp downstream passes NaN
    // through untouched, and the unit is dead until it is power-cycled. So the
    // sample is dropped here, before anything reads it - which is also what
    // keeps NaN out of the stillness ring, out of departRef_ (it only ever
    // copies ref_) and out of the rate ring (its observations are differences
    // of ring medians over a bounded span). The caller sees the previous
    // broadcast, unchanged, plus the sensor-error flag the STATE packet carries
    // in bit 4 - a held floor is what a display should show while the sensor is
    // out, not a zero.
    if (!isfinite(t) || !isfinite(alt)) {
      sensorErr_ = true;
      sensorRejects_++;
      return last_;
    }
    sensorErr_ = false;

    rollOdometer(t);

    if (!refSet_) {
      ref_ = alt;
      refSet_ = true;
      t_ = t;
      t0_ = t;
      ref0_ = alt;
      level_ = alt;              // no window yet; the raw sample is the best guess
      setHeight(0, 0.0);
      pitch_ = pitchHint_;
      pitchSet_ = pitchHintSet_;
      return out(t);
    }

    double dt = t - t_;
    t_ = t;

    // Carry the datum at the measured weather rate every sample. The stop-time
    // correction below is modulo the floor pitch, so it cannot see a full-floor
    // error - if the datum is ever allowed to lag a half pitch the readout just
    // slips a floor and the residual goes quiet. Feed-forward is what stops the
    // lag from ever getting there.
    if (nbuf_ < win_) {
      buf_[nbuf_++] = alt;
    } else {
      for (int i = 1; i < win_; i++) buf_[i - 1] = buf_[i];
      buf_[win_ - 1] = alt;
    }
    if (nbuf_ < win_) return out(t);

    bool still = floor_detail::stdPopulation(buf_, nbuf_) <= stillStd_;
    double level = floor_detail::median(buf_, nbuf_);
    level_ = level;
    stillRun_ = still ? stillRun_ + 1 : 0;

    if (!still) {
      if (unconfirmed_) {
        // the car moved off before the dwell held: it coasted through this
        // floor rather than serving it. The travel still happened, so it stays
        // on the odometer - it just was not a stop.
        unconfirmed_ = false;
        passthroughs_++;
      }
      if (!moving_) {                           // departure
        closeDwell();
        departRef_ = ref_;
        departFloor_ = floor_;
        departT_ = t_;
        moving_ = true;
      }
      return out(t);
    }

    if (moving_) {                              // arrival - call it now
      parkT0Set_ = false;                       // a fresh dwell starts here
      parkL0Set_ = false;
      nParkTail_ = 0;
      arrive(level, (double)stillRun_ >= FLOOR_RESYNC_S);
      return out(t);
    }

    if (unconfirmed_ && stillRun_ >= FLOOR_CONFIRM_N) {
      confirm();                                // dwell held: it was a stop
    }

    // Parked: carry the reference at the weather rate we have measured, then
    // correct whatever is left over under a hard cap. The feed-forward term is
    // what makes this work in a storm - a pure follower only gets to correct
    // while the car is parked, which here is 29% of the time, so its effective
    // tracking rate is 29% of the cap and a brisk front outruns it.
    if (!parkT0Set_) {
      parkT0_ = t;
      parkT0Set_ = true;
      parkL0Set_ = false;
    }
    if (!parkL0Set_ && t - parkT0_ >= FLOOR_DWELL_SKIP_S) {
      parkL0_ = level;
      parkL0Set_ = true;
      parkTs_ = t;
    }
    if (parkL0Set_) {
      pushParkTail(t, level);
    }

    // dt <= 0 means the clock went backwards, which a caller deriving t from
    // millis()/1000.0 does every 49.7 days - the wrap puts dt at about
    // -4.29e6 s. Both terms below are rates times dt, so a single such sample
    // would slam ref_ by kilometres and take the floor with it. Skipping the
    // carry costs exactly the drift of one sample period; the parked follower
    // picks it up on the next one. dt == 0 is a duplicate timestamp and the
    // arithmetic below is already a no-op for it.
    if (dt > 0.0) {
      ref_ += driftRate_ * dt;
      double step = floor_detail::clampD(level - ref_, -maxDriftMps_ * dt,
                                         maxDriftMps_ * dt);
      ref_ += step;
      drift_ += step + driftRate_ * dt;
    }
    return out(t);
  }

  // ---- readouts ---------------------------------------------------------
  int nFloors() const {
    if (!occupied_) return 0;
    return maxIndex() - minIndex() + 1;
  }

  double rise() const {
    if (!occupied_) return 0.0;
    double lo = 0.0, hi = 0.0;
    bool first = true;
    for (int k = 0; k < FLOOR_TABLE_N; k++) {
      if (!(occupied_ & (1ULL << k))) continue;
      if (first) { lo = hi = heights_[k]; first = false; }
      else if (heights_[k] < lo) lo = heights_[k];
      else if (heights_[k] > hi) hi = heights_[k];
    }
    return hi - lo;
  }

  int floor1Based() const {
    if (!occupied_) return 1;
    return floor_ - minIndex() + 1;
  }

  // Raw altitude the lowest landing currently reads. Subtracting it from a live
  // sample puts that sample in building coordinates, so the trace lands on fixed
  // floor rails - and it is a running value the node already holds, not
  // something fitted after the fact.
  double datum() const {
    if (!refSet_) return 0.0;
    if (!occupied_) return ref_;
    return ref_ - (heightOr(floor_, 0.0) - minHeight());
  }

  bool modelReady() const { return pitchSet_ && pitch_ != 0.0; }

  // ---- diagnostics the replay harness and the STATS packet read ---------
  double pitch() const { return pitch_; }
  int floorIndex() const { return floor_; }
  uint32_t trips() const { return trips_; }
  uint32_t stops() const { return stops_; }
  double distanceM() const { return distanceM_; }
  double driftRate() const { return driftRate_; }
  uint32_t rejects() const { return rejects_; }
  uint32_t passthroughs() const { return passthroughs_; }
  // An index fell outside the 64-slot table and was clamped. Should never fire;
  // if it does the building model has run away and the record is not trustworthy.
  bool tableClamped() const { return tableClamped_; }
  // The most recent sample was not a finite number and was dropped. This is the
  // sensorErr bit in both wire formats - STATE bit 4, STATS bit 0.
  bool sensorError() const { return sensorErr_; }
  uint32_t sensorRejects() const { return sensorRejects_; }
  double dist24hM() const {
    double s = 0.0;
    for (int i = 0; i < FLOOR_ODO_BUCKETS; i++) s += odoBuckets_[i];
    return s;
  }
  // Confirmed dwells at a floor, by 1-based index. ALGORITHM.md section 3 uses
  // this to distinguish a real floor from a single stray index.
  uint16_t stopCount1Based(int floor1) const {
    if (!occupied_) return 0;
    return stopCountAt(floor1 - 1 + minIndex());
  }
  double height1Based(int floor1) const {
    if (!occupied_) return 0.0;
    return heightOr(floor1 - 1 + minIndex(), 0.0) - minHeight();
  }

  // ---- persistence ------------------------------------------------------
  void save(FloorModelState *s) const {
    memset(s, 0, sizeof(*s));
    s->schemaVersion = FLOOR_STATE_SCHEMA;
    s->pitchSet = pitchSet_ ? 1 : 0;
    s->refSet = refSet_ ? 1 : 0;
    s->pitch = pitch_;
    s->ref = ref_;
    s->floor = (int16_t)floor_;
    s->occupied = occupied_;
    for (int k = 0; k < FLOOR_TABLE_N; k++) {
      s->heights[k] = heights_[k];
      s->stopCounts[k] = stopCounts_[k];
    }
    s->distanceM = distanceM_;
    s->trips = trips_;
    s->stops = stops_;
    for (int i = 0; i < FLOOR_ODO_BUCKETS; i++) s->odoBuckets[i] = odoBuckets_[i];
    s->odoBucket = odoBucket_;
    s->odoBucketSet = odoBucketSet_ ? 1 : 0;
    s->tableClamped = tableClamped_ ? 1 : 0;
  }

  // Returns false and changes nothing if the record is from another schema, or
  // if the model that wrote it had already run off the end of the floor table.
  // The transient state - the stillness ring, the dwell timers, the rate ring -
  // is deliberately not restored: it is all seconds-scale and refills within a
  // dwell, and a stale weather rate carried across a reboot would be applied to
  // trips it never observed.
  bool restore(const FloorModelState *s) {
    if (s == NULL || s->schemaVersion != FLOOR_STATE_SCHEMA) return false;
    // A record written by a runaway model is worth less than no record: loading
    // it sets restored_ and modelReady, which is exactly what stops the
    // bootstrap from rebuilding the ladder from scratch. Cold boot instead.
    if (s->tableClamped) return false;
    clearState();
    pitchSet_ = (s->pitchSet != 0);
    refSet_ = (s->refSet != 0);
    pitch_ = s->pitch;
    ref_ = s->ref;
    level_ = s->ref;
    floor_ = s->floor;
    occupied_ = s->occupied;
    for (int k = 0; k < FLOOR_TABLE_N; k++) {
      heights_[k] = s->heights[k];
      stopCounts_[k] = s->stopCounts[k];
    }
    distanceM_ = s->distanceM;
    trips_ = s->trips;
    stops_ = s->stops;
    // The hour buckets are not loaded. They are indexed by uptime, and uptime
    // restarts at 0 on the boot that reads them, so there is nothing in the
    // record - and, with no RTC, nothing anywhere on the node - that says how
    // long the unit was off. Loading them would report a bucket filled last
    // Tuesday as part of "the last 24 h". So the window starts empty and grows
    // back over the following day, which is the claim the display already makes
    // and what spec section 9 promises. The lifetime odometer is unaffected:
    // distanceM_ above is a total, not a window.
    restored_ = true;
    return true;
  }

  bool restored() const { return restored_; }

  // ===========================================================================
  // estimate_pitch - floor spacing is the value that makes every jump integer
  // ===========================================================================
  //
  // Jumps are multiples of the floor pitch, so sweep the candidate pitch and
  // keep whichever minimises the distance-to-integer of jump/pitch. A plain 1-D
  // sweep, no solver: 3600 candidates over 12 jumps is ~43k double operations,
  // microseconds on this chip.
  //
  // Two numpy behaviours are reproduced rather than tidied up. np.arange
  // accumulates (p += step) instead of computing lo + i*step, which differs in
  // the last bits from the third candidate onward; and its element count is
  // ceil((hi - lo)/step), which for these bounds is 3600 and not the 3601 an
  // inclusive sweep would give. Neither changes the answer at 0.5 mm resolution,
  // but matching them keeps the port bit-comparable against the reference.
  static bool estimatePitch(const double *jumps, int n, double *outPitch,
                            double lo = FLOOR_PITCH_LO, double hi = FLOOR_PITCH_HI,
                            double step = FLOOR_PITCH_STEP) {
    double j[FLOOR_BOOTSTRAP_JUMPS * 2];
    int m = 0;
    const int cap = (int)(sizeof(j) / sizeof(j[0]));
    for (int i = 0; i < n && m < cap; i++) {
      double a = fabs(jumps[i]);
      if (a > FLOOR_MIN_JUMP_M) j[m++] = a;
    }
    if (m < 3) return false;

    long count = (long)ceil((hi - lo) / step);
    if (count < 1) return false;

    double best = 0.0;
    double berr = INFINITY;
    double p = lo;
    for (long i = 0; i < count; i++) {
      double err = 0.0;
      for (int k = 0; k < m; k++) {
        double r = j[k] / p;
        double d = fabs(r - floor_detail::roundHalfEven(r));
        if (d > 0.5) d = 0.5;
        err += d * d;
      }
      if (err < berr) { berr = err; best = p; }
      p += step;
    }
    *outPitch = best;
    return true;
  }

 private:
  // ---- building model, all running means --------------------------------

  // Running mean of a landing's height, pinned to the floor grid.
  //
  // Each observation is chained off another learned height, so any per-trip bias
  // - which is exactly what a pressure front produces - would otherwise
  // random-walk the whole ladder apart, inventing floors. Real buildings sit
  // close to uniform, so clamping each landing to within half a pitch of
  // index x pitch bounds that walk without hiding genuine non-uniformity (this
  // building's largest true deviation is 0.3 m, a fifth of the clamp).
  void learn(int f, double heightObs) {
    int k = slot(f);
    if (occupied_ & (1ULL << k)) {
      heights_[k] += alpha_ * (heightObs - heights_[k]);
    } else {
      heights_[k] = heightObs;
      occupied_ |= (1ULL << k);
    }
    if (pitch_ != 0.0 && pitchSet_) {
      double nominal = (double)f * pitch_;
      double lim = 0.5 * pitch_;
      heights_[k] = floor_detail::clampD(heights_[k], nominal - lim, nominal + lim);
    }
  }

  // Blend each trip's implied spacing into the running pitch estimate.
  void refinePitch(double jump, int n) {
    if (n) pitch_ += FLOOR_PITCH_GAIN * (fabs(jump) / fabs((double)n) - pitch_);
  }

  // Measure the weather from a completed dwell.
  //
  // While the car is parked its height is constant by definition, so any change
  // in the reading over the dwell is the reference moving - a direct observation
  // of the drift rate that needs no floor assignment and cannot be confused with
  // motion. One dwell is noisy (~1.5 hPa/h), but a running mean over a few dozen
  // of them resolves the rate to ~0.13 hPa/h, far finer than the drift that
  // breaks the floor decision.
  void closeDwell() {
    if (!parkT0Set_ || !parkL0Set_) {
      parkT0Set_ = false;
      return;
    }
    if (nParkTail_ == 0) {
      parkT0Set_ = false;
      parkL0Set_ = false;
      return;
    }
    // The oldest entry in the tail ring, which is DWELL_TAIL_N samples back
    // from the present - see the DWELL_TAIL_N note.
    double tEnd = parkTailT_[0];
    double lEnd = parkTailL_[0];
    double span = tEnd - parkTs_;
    if (span >= FLOOR_MIN_DWELL_RATE_S) {
      pushRateObs((lEnd - parkL0_) / span);
      driftRate_ = floor_detail::clampD(
          floor_detail::median(rateObs_, nRateObs_),
          -FLOOR_DRIFT_RATE_CAP, FLOOR_DRIFT_RATE_CAP);
    }
    parkT0Set_ = false;
    parkL0Set_ = false;
    nParkTail_ = 0;
  }

  double heightOf(int f) const {
    int k = slotConst(f);
    if (k >= 0 && (occupied_ & (1ULL << k))) return heights_[k];
    return (double)f * pitch_;   // nominal ladder, same as an unlearned landing
  }

  // Commit a landing from the jump, with the weather taken back out.
  //
  // The floor is read from the jump between two stops, which is accurate because
  // a trip is short. What that costs is an anchor: the index is integrated, so a
  // rounding that goes wrong stays wrong. An absolute readout off a drift-tracked
  // datum was tried instead and is worse - its phase detector works modulo the
  // pitch, so a datum allowed to lag half a floor simply slips and goes quiet
  // about it.
  //
  // So the budget is per trip: drift-during-trip + noise must stay under half a
  // pitch (1.44 m). Noise is not negotiable, which leaves about 0.5 m for drift.
  // Subtracting the measured weather rate over the trip's own duration is what
  // buys that back, and it is why the storm limit moves rather than the noise
  // floor.
  void arrive(double level, bool forced) {
    double jump = level - departRef_;
    double elapsed = t_ - departT_;
    if (elapsed < 1.0) elapsed = 1.0;
    double jumpTrue = jump - driftRate_ * elapsed;   // take the weather back out

    if (fabs(jumpTrue) < FLOOR_MIN_JUMP_M) {         // door cycle, same floor
      moving_ = false;
      ref_ = level;
      confidence_ = 1.0;
      return;
    }

    if (!pitchSet_) {                                // still bootstrapping
      moving_ = false;
      ref_ = level;
      pushBoot(departFloor_, jumpTrue, level);
      if (nBoot_ >= FLOOR_BOOTSTRAP_JUMPS) {
        double p = 0.0;
        pitchSet_ = estimatePitch(boot_, nBoot_, &p);
        pitch_ = pitchSet_ ? p : 0.0;
        if (pitchSet_ && pitch_ != 0.0) replay();
      }
      return;
    }

    double frac = jumpTrue / pitch_;
    int n = (int)floor_detail::roundHalfEven(frac);
    double off = fabs(frac - (double)n);

    if (off > FLOOR_OFF_LATTICE && !forced) {        // a plateau, not a landing
      rejects_++;
      ref_ = departRef_;
      return;                                        // still in the trip
    }

    moving_ = false;
    ref_ = level;
    if (n == 0) {
      confidence_ = 1.0;
      return;
    }
    confidence_ = 1.0 - 2.0 * off;
    if (confidence_ < 0.0) confidence_ = 0.0;
    floor_ = departFloor_ + n;
    trips_++;
    addDistance(fabs((double)n) * pitch_);
    unconfirmed_ = true;
    uFloor0_ = departFloor_;
    uJumpTrue_ = jumpTrue;
    uN_ = n;
    uLevel_ = level;
  }

  // A dwell that held is a real landing: count it and learn from it.
  void confirm() {
    int floor0 = uFloor0_;
    double jumpTrue = uJumpTrue_;
    int n = uN_;
    double level = uLevel_;
    unconfirmed_ = false;
    stops_++;
    bumpStopCount(floor_);
    refinePitch(jumpTrue, n);
    double base = heightOf(floor0);
    learn(floor_, base + jumpTrue);
    revisitRate(level);
  }

  // Measure the weather by returning to a floor you have already stood on.
  //
  // The car parks at the same landing over and over - the lobby alone takes a
  // fifth of all stops - and that landing does not move. So the change in the
  // reading between two visits to the same floor is drift, over a baseline of
  // minutes rather than the seconds a single dwell affords.
  //
  // It also cancels what a single dwell cannot: both readings are taken at the
  // same point in the same stop sequence, so the shaft's post-stop pressure
  // relaxation sits in both and subtracts out. Measuring inside one dwell leaves
  // that transient in, worth about 1 hPa/h, and needs a 30 s dwell to dilute it -
  // which in this building does not turn up for the first 49 minutes.
  void revisitRate(double level) {
    int k = slot(floor_);
    bool had = lastSeen_[k];
    double prevT = lastSeenT_[k];
    double prevL = lastSeenL_[k];
    lastSeen_[k] = true;
    lastSeenT_[k] = t_;
    lastSeenL_[k] = level;
    if (!had) return;
    double gap = t_ - prevT;
    if (!(FLOOR_REVISIT_MIN_S <= gap && gap <= FLOOR_REVISIT_MAX_S)) return;
    double obs = (level - prevL) / gap;
    // That is a misassignment, not weather.
    if (fabs(obs) * gap > 0.6 * pitch_) return;
    pushRateObs(obs);
    driftRate_ = trustedRate();
  }

  // Median of the recent observations, shrunk by how much they disagree.
  //
  // A weather front makes every revisit tell the same story, so the estimates
  // cluster and the median is worth acting on. Gusts make them scatter around
  // zero, and feeding that forward is worse than doing nothing - at 0.1 hPa of
  // gust it costs more than it ever gains. Comparing the median against the
  // spread separates the two: consistent evidence passes through intact,
  // scattered evidence is shrunk toward zero and the algorithm falls back to its
  // uncompensated behaviour, so the compensation can never do harm.
  //
  // This is the reason the whole file is double. med and mad are around 1e-4, so
  // med*med is around 1e-8 and the ratio below is a 1e-8/1e-8 quotient: float32
  // has 7 decimal digits and would return noise.
  double trustedRate() const {
    if (nRateObs_ < 3) return 0.0;
    double med = floor_detail::median(rateObs_, nRateObs_);
    double dev[FLOOR_RATE_HISTORY];
    for (int i = 0; i < nRateObs_; i++) dev[i] = fabs(rateObs_[i] - med);
    double mad = floor_detail::median(dev, nRateObs_);
    double km = FLOOR_RATE_TRUST_K * mad;
    double w = med * med / (med * med + km * km + 1e-18);
    return floor_detail::clampD(w * med, -FLOOR_DRIFT_RATE_CAP,
                                FLOOR_DRIFT_RATE_CAP);
  }

  // Re-run the buffered bootstrap trips now that the pitch is known.
  //
  // The floor is re-based to 0 first: the indices handed out during the
  // bootstrap were assigned without a pitch, so they are not on the same ladder
  // the replayed jumps build. Replaying on top of them would double-count.
  void replay() {
    floor_ = 0;
    for (int i = 0; i < nPending_; i++) {
      double jump = pendJump_[i];
      int n = (int)floor_detail::roundHalfEven(jump / pitch_);
      if (n == 0) continue;
      floor_ = floor_ + n;
      trips_++;
      stops_++;
      bumpStopCount(floor_);
      addDistance(fabs((double)n) * pitch_);
      double base = heightOf(floor_ - n);
      learn(floor_, base + jump);
    }
    nPending_ = 0;
  }

  FloorBroadcast out(double t) {
    FloorBroadcast b;
    b.t = t;
    b.floor = floor1Based();
    b.moving = moving_;
    b.confidence = confidence_;
    b.nFloors = nFloors();
    b.pitch = (pitchSet_ && pitch_ != 0.0) ? pitch_ : 0.0;
    b.rise = rise();
    b.trips = trips_;
    b.stops = stops_;
    b.distanceM = distanceM_;
    b.drift = drift_;
    b.driftRateMps = driftRate_;
    b.datum = datum();
    b.posFloors = (pitchSet_ && pitch_ != 0.0) ? (level_ - b.datum) / pitch_ : 0.0;
    b.direction = direction();
    b.dist24hM = dist24hM();
    b.modelReady = modelReady();
    last_ = b;    // what a dropped sample re-serves, rather than a zeroed frame
    return b;
  }

  // The display arrow. Sign of how far the car has climbed since it left, with
  // the stillness threshold as the deadband - below that the reading has not
  // actually moved, so there is nothing to point at yet.
  FloorDirection direction() const {
    if (!moving_) return FLOOR_DIR_IDLE;
    double d = level_ - departRef_;
    if (d > stillStd_) return FLOOR_DIR_UP;
    if (d < -stillStd_) return FLOOR_DIR_DOWN;
    return FLOOR_DIR_IDLE;
  }

  // ---- the fixed-size containers ----------------------------------------
  int slot(int f) {
    int k = f + FLOOR_TABLE_ORIGIN;
    if (k < 0) { k = 0; tableClamped_ = true; }
    if (k >= FLOOR_TABLE_N) { k = FLOOR_TABLE_N - 1; tableClamped_ = true; }
    return k;
  }
  // Read-side lookup: -1 for an index the table has no room for, never another
  // floor's slot. Clamping is right on the write side - the alternative to a
  // bound is a crash, and the flag says the model has run away - but a read
  // that clamps hands back floor 31's learned height for floor 40 and says
  // nothing, which is how datum() and posFloors go quietly wrong instead of
  // falling back to the nominal ladder.
  int slotConst(int f) const {
    int k = f + FLOOR_TABLE_ORIGIN;
    if (k < 0 || k >= FLOOR_TABLE_N) return -1;
    return k;
  }
  void setHeight(int f, double v) {
    int k = slot(f);
    heights_[k] = v;
    occupied_ |= (1ULL << k);
  }
  double heightOr(int f, double fallback) const {
    int k = slotConst(f);
    if (k < 0) return fallback;
    return (occupied_ & (1ULL << k)) ? heights_[k] : fallback;
  }
  uint16_t stopCountAt(int f) const {
    int k = slotConst(f);
    return (k < 0) ? 0 : stopCounts_[k];
  }
  void bumpStopCount(int f) {
    int k = slot(f);
    if (stopCounts_[k] != 0xFFFF) stopCounts_[k]++;
  }
  int minIndex() const {
    for (int k = 0; k < FLOOR_TABLE_N; k++) {
      if (occupied_ & (1ULL << k)) return k - FLOOR_TABLE_ORIGIN;
    }
    return 0;
  }
  int maxIndex() const {
    for (int k = FLOOR_TABLE_N - 1; k >= 0; k--) {
      if (occupied_ & (1ULL << k)) return k - FLOOR_TABLE_ORIGIN;
    }
    return 0;
  }
  double minHeight() const {
    double lo = 0.0;
    bool first = true;
    for (int k = 0; k < FLOOR_TABLE_N; k++) {
      if (!(occupied_ & (1ULL << k))) continue;
      if (first || heights_[k] < lo) { lo = heights_[k]; first = false; }
    }
    return lo;
  }

  void pushRateObs(double v) {
    // The last door a non-finite number could come through: both observations
    // are a height difference over a span the callers bound away from zero, so
    // this only fires if one of those invariants is ever broken. A single NaN
    // in the ring poisons the median, and the median is fed forward into ref_
    // on every sample from then on.
    if (!isfinite(v)) return;
    if (nRateObs_ < FLOOR_RATE_HISTORY) {
      rateObs_[nRateObs_++] = v;
    } else {
      for (int i = 1; i < FLOOR_RATE_HISTORY; i++) rateObs_[i - 1] = rateObs_[i];
      rateObs_[FLOOR_RATE_HISTORY - 1] = v;
    }
  }

  void pushParkTail(double t, double level) {
    if (nParkTail_ < FLOOR_DWELL_TAIL_N + 1) {
      parkTailT_[nParkTail_] = t;
      parkTailL_[nParkTail_] = level;
      nParkTail_++;
      return;
    }
    for (int i = 1; i <= FLOOR_DWELL_TAIL_N; i++) {
      parkTailT_[i - 1] = parkTailT_[i];
      parkTailL_[i - 1] = parkTailL_[i];
    }
    parkTailT_[FLOOR_DWELL_TAIL_N] = t;
    parkTailL_[FLOOR_DWELL_TAIL_N] = level;
  }

  // The Python lists grow without bound. They cannot in practice - arrive()
  // rejects anything under MIN_JUMP_M before it gets here, so estimatePitch
  // always has its 3 usable jumps by the twelfth and the buffers are emptied -
  // but a bound is still a bound, so the oldest entry is dropped rather than
  // the newest, which is what a ring would do anyway.
  void pushBoot(int departFloor, double jumpTrue, double level) {
    if (nBoot_ < FLOOR_BOOTSTRAP_JUMPS) {
      boot_[nBoot_++] = jumpTrue;
    } else {
      for (int i = 1; i < FLOOR_BOOTSTRAP_JUMPS; i++) boot_[i - 1] = boot_[i];
      boot_[FLOOR_BOOTSTRAP_JUMPS - 1] = jumpTrue;
    }
    if (nPending_ < FLOOR_BOOTSTRAP_JUMPS) {
      pendFloor_[nPending_] = departFloor;
      pendJump_[nPending_] = jumpTrue;
      pendLevel_[nPending_] = level;
      nPending_++;
    } else {
      for (int i = 1; i < FLOOR_BOOTSTRAP_JUMPS; i++) {
        pendFloor_[i - 1] = pendFloor_[i];
        pendJump_[i - 1] = pendJump_[i];
        pendLevel_[i - 1] = pendLevel_[i];
      }
      pendFloor_[FLOOR_BOOTSTRAP_JUMPS - 1] = departFloor;
      pendJump_[FLOOR_BOOTSTRAP_JUMPS - 1] = jumpTrue;
      pendLevel_[FLOOR_BOOTSTRAP_JUMPS - 1] = level;
    }
  }

  // ---- odometer ---------------------------------------------------------
  // 24 one-hour buckets indexed by uptime/3600 mod 24, cleared on entry. With no
  // RTC this is the only honest definition of "last 24 h": after a reboot it
  // reports less than 24 h of history rather than pretending otherwise.
  void rollOdometer(double t) {
    if (t < 0.0) return;
    uint32_t b = (uint32_t)(t / 3600.0) % FLOOR_ODO_BUCKETS;
    if (!odoBucketSet_) {
      odoBucket_ = b;
      odoBucketSet_ = true;
      return;
    }
    if (b == odoBucket_) return;
    // Clear every bucket stepped over, not just the one landed in: a gap longer
    // than an hour would otherwise leave stale metres from a day ago in place.
    uint32_t k = odoBucket_;
    for (int i = 0; i < FLOOR_ODO_BUCKETS; i++) {
      k = (k + 1) % FLOOR_ODO_BUCKETS;
      odoBuckets_[k] = 0.0;
      if (k == b) break;
    }
    odoBucket_ = b;
  }

  void addDistance(double m) {
    distanceM_ += m;
    if (odoBucketSet_) odoBuckets_[odoBucket_] += m;
  }

  // ---- configuration ----------------------------------------------------
  double stillStd_;
  int    win_;
  double maxDriftMps_;
  double alpha_;
  double pitchHint_;
  bool   pitchHintSet_;

  // ---- live state (mirrors the Python dataclass fields) -----------------
  int    floor_;
  double ref_;
  bool   refSet_;
  bool   moving_;
  double pitch_;
  bool   pitchSet_;
  double heights_[FLOOR_TABLE_N];
  uint64_t occupied_;
  uint16_t stopCounts_[FLOOR_TABLE_N];
  uint32_t trips_;
  uint32_t stops_;
  double distanceM_;
  double confidence_;
  double driftRate_;

  double buf_[FLOOR_STILL_WIN_MAX];
  int    nbuf_;
  double t_;
  double t0_;
  double ref0_;
  double drift_;
  double departRef_;
  int    departFloor_;
  double departT_;
  double level_;            // last median; the Python recomputes it per call

  double boot_[FLOOR_BOOTSTRAP_JUMPS];
  int    nBoot_;
  int    pendFloor_[FLOOR_BOOTSTRAP_JUMPS];
  double pendJump_[FLOOR_BOOTSTRAP_JUMPS];
  double pendLevel_[FLOOR_BOOTSTRAP_JUMPS];
  int    nPending_;

  double parkT0_;
  bool   parkT0Set_;
  double parkL0_;
  bool   parkL0Set_;
  double parkTs_;
  double parkTailT_[FLOOR_DWELL_TAIL_N + 1];
  double parkTailL_[FLOOR_DWELL_TAIL_N + 1];
  int    nParkTail_;

  double rateObs_[FLOOR_RATE_HISTORY];
  int    nRateObs_;

  bool     lastSeen_[FLOOR_TABLE_N];
  double   lastSeenT_[FLOOR_TABLE_N];
  double   lastSeenL_[FLOOR_TABLE_N];

  int      stillRun_;
  uint32_t rejects_;
  uint32_t passthroughs_;
  bool     tableClamped_;
  bool     restored_;
  bool     sensorErr_;
  uint32_t sensorRejects_;

  // The last broadcast handed out, re-served whenever a sample is dropped.
  FloorBroadcast last_;

  // The unconfirmed landing: committed to the broadcast, not yet to the
  // odometer or the building model. Cleared either by confirm() or by the car
  // moving off before the dwell held.
  bool   unconfirmed_;
  int    uFloor0_;
  double uJumpTrue_;
  int    uN_;
  double uLevel_;

  double   odoBuckets_[FLOOR_ODO_BUCKETS];
  uint32_t odoBucket_;
  bool     odoBucketSet_;
};
