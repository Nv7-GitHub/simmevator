//
// Host-side unit tests for the floor detector.
//
// sim/compare_to_python.py already proves the port is faithful on the one
// capture that exists, sample for sample. What it cannot prove is anything
// about inputs that capture does not contain, and that is what this file is
// for: synthetic traces where the right answer is known by construction, and a
// regression for each way a real sensor or a real clock can hand update()
// something the Python reference never sees.
//
// The traces are built from two primitives - hold an altitude, or ramp between
// two of them - at the 1 Hz the algorithm is specified at. That is enough to
// place a landing exactly on the lattice, 0.4 of a pitch off it, or 0.6 m
// away, which is what separates the cases below. Every expected floor index is
// arithmetic on the trace, not a number read back off a previous run.
//
//   pio test -e native
//
#include <unity.h>

#include <math.h>
#include <string.h>

#include "floor_monitor.h"

// Unity's TEST_ASSERT_*_DOUBLE macros compile to a failure unless unity itself
// was built with UNITY_INCLUDE_DOUBLE, which is a property of the unity build
// rather than of this file. Comparing by hand also keeps full precision, which
// the FLOAT variants would not: half the values checked below - drift rates
// around 1e-4, a datum carried to millimetres - are exactly the numbers the
// header is double for in the first place.
#define ASSERT_NEAR(tol, expected, actual)                                  \
  TEST_ASSERT_TRUE_MESSAGE(                                                 \
      fabs((double)(expected) - (double)(actual)) <= (double)(tol),         \
      #actual " is not within " #tol " of " #expected)
#define ASSERT_SAME(expected, actual)                                       \
  TEST_ASSERT_TRUE_MESSAGE((double)(expected) == (double)(actual),          \
                           #actual " differs from " #expected)

// Simmons' measured pitch. Used as a hint wherever a test is about something
// other than learning the pitch, so those tests do not have to spend twelve
// trips bootstrapping before they can say anything.
static const double kPitch = 2.87;

// A ride slow enough to be mistaken for a dwell would invalidate every trace
// here, so rides are built to clear the stillness test by a wide margin: one
// floor in 4 s is 0.72 m/s, whose 5-sample population std is 1.0 m against a
// 0.08 m threshold. Dwells are 20 s - long enough to confirm a stop (3 still
// samples) and to be revisited (30 s minimum gap between two visits is met by
// the round trip, not by one dwell), and short enough that closeDwell() never
// reaches its 30 s baseline. That keeps the rate ring fed by revisits alone,
// so the trusted-rate tests below control every observation in it.
static const int kRideS  = 4;
static const int kDwellS = 20;

// ===========================================================================
// Trace builder
// ===========================================================================
struct Rider {
  FloorMonitor fm;
  double t;
  FloorBroadcast b;

  Rider() : t(0.0) { memset(&b, 0, sizeof(b)); }

  void hint(double pitch) {
    fm.configure(FLOOR_STILL_STD_M, FLOOR_STILL_WIN, FLOOR_MAX_DRIFT_MPS,
                 FLOOR_HEIGHT_ALPHA, pitch);
  }

  void step(double alt) {
    b = fm.update(t, alt);
    t += 1.0;
  }

  void hold(double alt, int secs) {
    for (int i = 0; i < secs; i++) step(alt);
  }

  // The first sample of a ramp is already off the landing, and the last one is
  // exactly on the new one, so the window goes still the sample after a ride
  // ends and the arrival level is the target altitude exactly.
  void ride(double from, double to, int secs) {
    for (int i = 1; i <= secs; i++)
      step(from + (to - from) * (double)i / (double)secs);
  }

  void moveTo(double from, double to, int rideS) {
    ride(from, to, rideS);
    hold(to, kDwellS);
  }
};

// ===========================================================================
// Lattice ascents and descents
// ===========================================================================

// Four single-floor moves up, then one that covers two floors in a single
// ride. The two-floor move is the one that matters: it is the difference
// between reading the floor from a jump and counting arrivals.
void test_lattice_ascent(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);

  TEST_ASSERT_TRUE(r.b.modelReady);
  TEST_ASSERT_EQUAL_INT(1, r.b.floor);
  TEST_ASSERT_EQUAL_INT(0, r.fm.floorIndex());

  for (int k = 1; k <= 4; k++) {
    r.moveTo((k - 1) * kPitch, k * kPitch, kRideS);
    TEST_ASSERT_EQUAL_INT(k, r.fm.floorIndex());
    TEST_ASSERT_EQUAL_INT(k + 1, r.b.floor);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)k, r.b.trips);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)k, r.b.stops);
    // A landing dead on the lattice leaves nothing for the confidence to
    // discount, so anything below 1.0 here is a lattice fit going soft.
    ASSERT_NEAR(1e-9, 1.0, r.b.confidence);
  }

  r.moveTo(4 * kPitch, 6 * kPitch, 6);
  TEST_ASSERT_EQUAL_INT(6, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_INT(7, r.b.floor);
  TEST_ASSERT_EQUAL_UINT32(5u, r.b.trips);        // one trip, two floors
  TEST_ASSERT_EQUAL_INT(7, r.b.nFloors);          // 0..6 learned
  ASSERT_NEAR(0.01, 6 * kPitch, r.b.rise);
  TEST_ASSERT_FALSE(r.b.moving);
  TEST_ASSERT_EQUAL_INT(FLOOR_DIR_IDLE, r.b.direction);
}

// The same ladder run backwards, including one ride that gives back six floors
// at once. Distance is unsigned, so the odometer keeps climbing while the
// index falls.
void test_lattice_descent(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);
  for (int k = 1; k <= 6; k++) r.moveTo((k - 1) * kPitch, k * kPitch, kRideS);
  TEST_ASSERT_EQUAL_INT(6, r.fm.floorIndex());

  for (int k = 5; k >= 3; k--) {
    r.moveTo((k + 1) * kPitch, k * kPitch, kRideS);
    TEST_ASSERT_EQUAL_INT(k, r.fm.floorIndex());
    TEST_ASSERT_EQUAL_INT(k + 1, r.b.floor);
  }

  r.moveTo(3 * kPitch, 0.0, 8);
  TEST_ASSERT_EQUAL_INT(0, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_INT(1, r.b.floor);
  TEST_ASSERT_EQUAL_UINT32(10u, r.b.trips);       // 6 up, 3 down, 1 three-floor
  // 6 + 1 + 1 + 1 + 3 floors travelled, all of it positive metres.
  ASSERT_NEAR(0.01, 12 * kPitch, r.b.distanceM);
}

// Mid-ride the arrow points, and it points the way the car is going.
void test_direction_while_moving(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);
  r.ride(0.0, kPitch, kRideS);
  TEST_ASSERT_TRUE(r.b.moving);
  TEST_ASSERT_EQUAL_INT(FLOOR_DIR_UP, r.b.direction);
  r.hold(kPitch, kDwellS);

  r.ride(kPitch, 0.0, kRideS);
  TEST_ASSERT_TRUE(r.b.moving);
  TEST_ASSERT_EQUAL_INT(FLOOR_DIR_DOWN, r.b.direction);
  r.hold(0.0, kDwellS);
  TEST_ASSERT_EQUAL_INT(FLOOR_DIR_IDLE, r.b.direction);
}

// ===========================================================================
// A door cycle is not a trip
// ===========================================================================

// The car rocks 0.6 m on its ropes as people board. That is far enough to fail
// the stillness test - so the algorithm sees a departure and an arrival - but
// it is under MIN_JUMP_M, which is the whole reason that constant exists.
void test_door_cycle_below_min_jump_stays_on_the_floor(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);
  r.moveTo(0.0, kPitch, kRideS);

  const int      idx    = r.fm.floorIndex();
  const uint32_t trips  = r.b.trips;
  const uint32_t stops  = r.b.stops;
  const double   dist   = r.b.distanceM;

  // 0.6 m in 2 s: a 0.24 m std, three times the stillness threshold, so this
  // really does register as motion rather than being filtered away.
  r.ride(kPitch, kPitch + 0.6, 2);
  TEST_ASSERT_TRUE(r.b.moving);
  r.hold(kPitch + 0.6, kDwellS);
  r.ride(kPitch + 0.6, kPitch, 2);
  r.hold(kPitch, kDwellS);

  TEST_ASSERT_EQUAL_INT(idx, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_UINT32(trips, r.b.trips);
  TEST_ASSERT_EQUAL_UINT32(stops, r.b.stops);
  ASSERT_SAME(dist, r.b.distanceM);
  TEST_ASSERT_FALSE(r.b.moving);

  // The floor is still reachable afterwards: a door cycle re-bases ref_ onto
  // the new reading, so the next real jump has to measure from there.
  r.moveTo(kPitch, 2 * kPitch, kRideS);
  TEST_ASSERT_EQUAL_INT(idx + 1, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_UINT32(trips + 1, r.b.trips);
}

// ===========================================================================
// Off-lattice plateaus
// ===========================================================================

// A car decelerating through a floor sits briefly at a near-constant height,
// and a plain stillness test calls that an arrival. 1.4 pitches up is 0.4 off
// the grid, well past the 0.30 gate.
//
// Not 1.5: that is an exact tie, which lands on whichever side the division
// rounds to and would make the expected floor below a property of the
// arithmetic rather than of the algorithm. The tie itself is covered by
// test_round_half_even_matches_python.
void test_off_lattice_plateau_is_rejected(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);

  r.ride(0.0, 1.4 * kPitch, 5);
  r.hold(1.4 * kPitch, 15);      // under RESYNC_S still samples, so not forced

  TEST_ASSERT_EQUAL_INT(0, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_UINT32(0u, r.b.trips);
  TEST_ASSERT_TRUE(r.fm.rejects() > 0);
  // Rejecting is not the same as arriving: the trip is still open, which is
  // what lets the real landing below be measured from the original departure.
  TEST_ASSERT_TRUE(r.b.moving);

  r.ride(1.4 * kPitch, 2.0 * kPitch, 3);
  r.hold(2.0 * kPitch, kDwellS);

  // Two floors from where the trip started, not one from the plateau.
  TEST_ASSERT_EQUAL_INT(2, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_UINT32(1u, r.b.trips);
  TEST_ASSERT_EQUAL_UINT32(1u, r.b.stops);
}

// ...unless the car simply sits there, at which point it was a landing after
// all and the tracker has to resync rather than stay lost forever.
void test_off_lattice_plateau_resyncs_when_it_persists(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);
  r.ride(0.0, 1.4 * kPitch, 5);
  r.hold(1.4 * kPitch, 40);      // past RESYNC_S, so the rejection is overridden

  TEST_ASSERT_FALSE(r.b.moving);
  TEST_ASSERT_EQUAL_INT(1, r.fm.floorIndex());   // 1.4 rounds to 1
  TEST_ASSERT_EQUAL_UINT32(1u, r.b.trips);
  // A resync is the algorithm admitting it had to guess, and the confidence is
  // where it says so: 1 - 2*0.4.
  ASSERT_NEAR(1e-9, 0.2, r.b.confidence);
}

// ===========================================================================
// Bootstrap and replay
// ===========================================================================

// A fresh unit knows no pitch, so the first twelve trips are buffered rather
// than acted on. When estimate_pitch() finally succeeds, replay() re-bases the
// index to 0 and walks the buffer forward - the risk being that it walks it on
// top of indices the bootstrap already handed out and double-counts every one.
void test_bootstrap_replay_rebases_the_index_to_zero(void) {
  Rider r;                       // deliberately no pitch hint
  r.hold(0.0, 15);

  TEST_ASSERT_FALSE(r.b.modelReady);
  TEST_ASSERT_EQUAL_INT(0, r.fm.floorIndex());

  for (int k = 1; k <= FLOOR_BOOTSTRAP_JUMPS; k++) {
    r.ride((k - 1) * kPitch, k * kPitch, kRideS);
    r.hold(k * kPitch, 12);
    if (k < FLOOR_BOOTSTRAP_JUMPS) {
      // Nothing is published before the pitch is known, per ALGORITHM.md 8.
      TEST_ASSERT_FALSE(r.b.modelReady);
      TEST_ASSERT_EQUAL_UINT32(0u, r.b.trips);
      TEST_ASSERT_EQUAL_INT(0, r.fm.floorIndex());
    }
  }

  TEST_ASSERT_TRUE(r.b.modelReady);
  ASSERT_NEAR(1e-3, kPitch, r.b.pitch);
  // Twelve one-floor jumps replayed from 0 land on 12. Twenty-four would mean
  // the buffered trips were counted twice.
  TEST_ASSERT_EQUAL_INT(FLOOR_BOOTSTRAP_JUMPS, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_INT(FLOOR_BOOTSTRAP_JUMPS + 1, r.b.floor);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)FLOOR_BOOTSTRAP_JUMPS, r.b.trips);
  TEST_ASSERT_EQUAL_UINT32((uint32_t)FLOOR_BOOTSTRAP_JUMPS, r.b.stops);
  TEST_ASSERT_EQUAL_INT(FLOOR_BOOTSTRAP_JUMPS + 1, r.b.nFloors);
  ASSERT_NEAR(0.05, FLOOR_BOOTSTRAP_JUMPS * kPitch, r.b.rise);
}

// estimate_pitch on its own, away from any trace. Three usable jumps is the
// documented minimum and a shorter list has to fail rather than guess.
void test_estimate_pitch_needs_three_usable_jumps(void) {
  double p = 0.0;
  const double two[] = { 2 * kPitch, 3 * kPitch };
  TEST_ASSERT_FALSE(FloorMonitor::estimatePitch(two, 2, &p));

  // Below MIN_JUMP_M these are door cycles, not trips, so they do not count
  // toward the three even though the array is long enough.
  const double doors[] = { 0.4, 0.5, 0.6, 0.3, kPitch };
  TEST_ASSERT_FALSE(FloorMonitor::estimatePitch(doors, 5, &p));

  const double jumps[] = { kPitch, 2 * kPitch, 3 * kPitch, kPitch };
  TEST_ASSERT_TRUE(FloorMonitor::estimatePitch(jumps, 4, &p));
  ASSERT_NEAR(1e-3, kPitch, p);
}

// ===========================================================================
// trustedRate - clustered evidence passes, scattered evidence is shrunk
// ===========================================================================
//
// Both scenarios are the same shuttle between two landings, so they differ
// only in the shape of the drift they inject. Each landing is offset by a node
// value; the algorithm sees the difference between two visits to the same
// landing divided by the gap, which is the observation that reaches the rate
// ring. The dwells are too short for closeDwell() to contribute, so the ring
// holds nothing but these.

// A scatter sequence, one node per landing. Chosen for what it does to the
// statistic rather than for looking random: its consecutive-pair differences
// straddle zero with a spread twice the median, which is the shape a gust has
// and the shape the shrinkage exists to discount.
static const double kGust[24] = {
   0.0,  1.0, -0.2,  0.6, -1.0,  0.3, -0.6,  0.9,
  -0.4,  0.2, -0.8,  0.7, -0.1,  0.5, -0.9,  0.4,
  -0.3,  1.0, -0.7,  0.1, -0.5,  0.8, -0.2,  0.6 };

// 0.15 m of gust perturbs a jump by at most 0.26 m, which is 0.09 of a pitch -
// comfortably inside the 0.30 off-lattice gate, so the shuttle still tracks
// floors correctly and the test is measuring the rate estimate and nothing else.
static const double kGustAmp = 0.15;
static const int    kLegs    = 25;

// One landing to the next and back, twelve times over. Returns the last
// altitude fed, so a caller can carry on from where the shuttle left the car.
//
// The ramp is measured from the clock the shuttle started on rather than from
// zero, so the trace is the same shape wherever it is placed in time - which
// is what lets the wrap test below run it up against the millis() rollover.
static double shuttle(Rider *r, bool scattered, double rate) {
  const double t0 = r->t;
  r->hint(kPitch);
  double prev = scattered ? kGustAmp * kGust[0] : 0.0;
  r->hold(prev, kDwellS);
  for (int i = 1; i < kLegs; i++) {
    const double landing = (i & 1) ? kPitch : 0.0;
    const double from    = (i & 1) ? 0.0 : kPitch;
    if (scattered) {
      const double next = landing + kGustAmp * kGust[i % 24];
      r->ride(prev, next, kRideS);
      r->hold(next, kDwellS);
      prev = next;
    } else {
      // A steady ramp in time, which is what a front looks like: every revisit
      // tells the same story and the observations cluster.
      for (int k = 1; k <= kRideS; k++)
        r->step(from + (landing - from) * ((double)k / (double)kRideS) +
                rate * (r->t - t0));
      for (int k = 0; k < kDwellS; k++) r->step(landing + rate * (r->t - t0));
      prev = landing + rate * (r->t - 1.0 - t0);
    }
  }
  return prev;
}

void test_trusted_rate_passes_clustered_observations_through(void) {
  const double rate = 2e-4;      // 0.72 m/h, a plausible front
  Rider r;
  shuttle(&r, false, rate);

  TEST_ASSERT_EQUAL_UINT32(0u, r.fm.rejects());
  TEST_ASSERT_EQUAL_UINT32((uint32_t)(kLegs - 1), r.b.trips);
  // Every revisit sees the same rate, so the spread is nil, the weight is 1
  // and the median is acted on intact.
  ASSERT_NEAR(0.05 * rate, rate, r.fm.driftRate());
}

void test_trusted_rate_shrinks_scattered_observations(void) {
  Rider r;
  shuttle(&r, true, 0.0);

  TEST_ASSERT_EQUAL_UINT32(0u, r.fm.rejects());
  TEST_ASSERT_EQUAL_UINT32((uint32_t)(kLegs - 1), r.b.trips);

  // The scale of what was actually injected, computed from the same table the
  // trace was built from: a revisit spans two legs, and a leg is one ride plus
  // one dwell. Asserting against this rather than a recorded number is what
  // makes the test about shrinkage instead of about one past run's output.
  const double gap = 2.0 * (kRideS + kDwellS);
  double raw = 0.0;
  int    n   = 0;
  for (int i = 0; i + 2 < 24; i++) {
    raw += fabs(kGustAmp * (kGust[i + 2] - kGust[i]) / gap);
    n++;
  }
  raw /= (double)n;

  // Same order of magnitude going in as the clustered case, and at least four
  // times smaller coming out. Feeding gust forward is worse than doing nothing,
  // so the compensation has to stand down rather than chase it.
  TEST_ASSERT_TRUE(raw > 1e-3);
  TEST_ASSERT_TRUE(fabs(r.fm.driftRate()) < 0.25 * raw);
}

// The ring will not act on one or two observations at all, whatever they say.
void test_trusted_rate_is_zero_before_three_observations(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, kDwellS);
  r.moveTo(0.0, kPitch, kRideS);
  r.moveTo(kPitch, 0.0, kRideS);
  ASSERT_SAME(0.0, r.fm.driftRate());
}

// ===========================================================================
// Hardening 1 - a non-finite sample is dropped, not absorbed
// ===========================================================================

// The failure this guards is permanent, not transient: the first sample is
// stored straight into ref_, every clamp downstream passes NaN through
// untouched, and the unit reports a dead floor until it is power-cycled.
void test_non_finite_first_sample_does_not_poison_the_reference(void) {
  Rider r;
  r.hint(kPitch);
  r.step(NAN);

  TEST_ASSERT_TRUE(r.fm.sensorError());
  TEST_ASSERT_EQUAL_UINT32(1u, r.fm.sensorRejects());
  TEST_ASSERT_TRUE(isfinite(r.b.datum));
  TEST_ASSERT_TRUE(isfinite(r.b.posFloors));

  // The next finite sample is the one that seeds the reference, so the unit
  // behaves as though the bad sample never arrived.
  r.hold(0.0, 15);
  TEST_ASSERT_FALSE(r.fm.sensorError());
  r.moveTo(0.0, kPitch, kRideS);
  r.moveTo(kPitch, 2 * kPitch, kRideS);
  TEST_ASSERT_EQUAL_INT(2, r.fm.floorIndex());
  TEST_ASSERT_EQUAL_UINT32(2u, r.b.trips);
  TEST_ASSERT_TRUE(isfinite(r.b.datum));
}

void test_non_finite_sample_reserves_the_previous_broadcast(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, 15);
  r.moveTo(0.0, kPitch, kRideS);

  const FloorBroadcast before = r.b;

  r.step(NAN);
  TEST_ASSERT_TRUE(r.fm.sensorError());
  // A held floor is what a display should show while the sensor is out. Not a
  // zero, and not the floor drifting while nothing is being measured.
  TEST_ASSERT_EQUAL_INT(before.floor, r.b.floor);
  ASSERT_SAME(before.datum, r.b.datum);
  ASSERT_SAME(before.posFloors, r.b.posFloors);
  ASSERT_SAME(before.drift, r.b.drift);
  TEST_ASSERT_EQUAL_UINT32(before.trips, r.b.trips);
  ASSERT_SAME(before.distanceM, r.b.distanceM);

  r.step(INFINITY);
  r.step(-INFINITY);
  r.b = r.fm.update(NAN, kPitch);            // a non-finite timestamp too
  TEST_ASSERT_EQUAL_UINT32(4u, r.fm.sensorRejects());
  TEST_ASSERT_EQUAL_INT(before.floor, r.b.floor);

  // The sensor coming back must not leave the stillness ring, departRef_ or
  // the rate ring carrying anything the bad samples touched.
  r.hold(kPitch, 15);
  TEST_ASSERT_FALSE(r.fm.sensorError());
  r.moveTo(kPitch, 3 * kPitch, 8);
  TEST_ASSERT_EQUAL_INT(3, r.fm.floorIndex());
  TEST_ASSERT_TRUE(isfinite(r.fm.driftRate()));
  TEST_ASSERT_TRUE(isfinite(r.b.datum));
  TEST_ASSERT_TRUE(isfinite(r.b.posFloors));
  TEST_ASSERT_TRUE(isfinite(r.b.drift));
}

// ===========================================================================
// Hardening 2 - the clock going backwards
// ===========================================================================

// clampD is reached with its bounds swapped whenever dt is negative. np.clip
// returns a_max there and the Python limps on; the two-comparison form would
// return whichever bound it tested first, which is a different answer to the
// same input.
void test_clamp_tolerates_swapped_bounds(void) {
  ASSERT_SAME(5.0, floor_detail::clampD(5.0, 10.0, -10.0));
  ASSERT_SAME(10.0, floor_detail::clampD(50.0, 10.0, -10.0));
  ASSERT_SAME(-10.0, floor_detail::clampD(-50.0, 10.0, -10.0));
  // Same answers as the ordered form, which is the point of the guard.
  ASSERT_SAME(floor_detail::clampD(5.0, -10.0, 10.0),
                           floor_detail::clampD(5.0, 10.0, -10.0));
  ASSERT_SAME(floor_detail::clampD(50.0, -10.0, 10.0),
                           floor_detail::clampD(50.0, 10.0, -10.0));
}

// A caller deriving t from millis()/1000.0 wraps every 49.7 days: the sample
// after 4294967.295 arrives as 0.0, which puts dt at about -4.29e6 s.
//
// Both halves of the parked correction then misbehave, and this asserts the
// pair. The feed-forward term is a rate times dt, so it alone moves the
// reference 859 m at the drift rate below. The follower's cap becomes
// +/-0.02*dt = -/+85899 m, which is the bounds handed to clampD backwards; the
// plain two-comparison form returns whichever it tests first, so the reference
// takes that 85899 m instead of the 0.02 m the cap exists to allow. Measured
// on this trace with both guards backed out, one sample moves the datum by
// 85 km. Either guard on its own bounds the damage; the test wants both.
//
// The shuttle runs first, and runs up against the rollover, so driftRate_ is
// non-zero and the feed-forward term is live rather than incidentally
// harmless.
void test_millis_wrap_does_not_slam_the_reference(void) {
  const double kWrapS = 4294967.296;      // 2^32 ms in seconds

  Rider r;
  r.t = kWrapS - (double)(kLegs * (kRideS + kDwellS) + kDwellS);
  const double lastAlt = shuttle(&r, false, 2e-4);
  TEST_ASSERT_TRUE(fabs(r.fm.driftRate()) > 1e-5);

  const FloorBroadcast before = r.b;
  TEST_ASSERT_TRUE(isfinite(before.datum));

  const FloorBroadcast w = r.fm.update(0.0, lastAlt);   // the rollover sample
  TEST_ASSERT_EQUAL_INT(before.floor, w.floor);
  ASSERT_SAME(before.datum, w.datum);
  ASSERT_SAME(before.drift, w.drift);
  TEST_ASSERT_TRUE(isfinite(w.posFloors));

  // A duplicate timestamp is dt == 0, which has to be a no-op rather than a
  // divide or a zero-width clamp doing something surprising.
  const FloorBroadcast d = r.fm.update(0.0, lastAlt);
  TEST_ASSERT_EQUAL_INT(before.floor, d.floor);
  ASSERT_SAME(before.drift, d.drift);

  // And the tracker still works on the far side of the rollover: skipping the
  // carry costs one sample period of drift, not the model.
  r.t = 1.0;
  r.hold(lastAlt, 15);
  r.moveTo(lastAlt, lastAlt + kPitch, kRideS);
  TEST_ASSERT_EQUAL_INT(before.floor + 1, r.b.floor);
  TEST_ASSERT_TRUE(isfinite(r.b.datum));
}

// ===========================================================================
// Hardening 3 - the 24 h odometer after a reboot
// ===========================================================================

// There is no RTC, so nothing on the node knows how long it was off. A bucket
// filled last Tuesday cannot be aged, so it is not restored at all and the
// window honestly reports less than 24 h - which is what the file header, the
// display and spec section 9 all claim.
void test_restore_reports_the_shorter_odometer_window(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, kDwellS);
  for (int k = 1; k <= 3; k++) r.moveTo((k - 1) * kPitch, k * kPitch, kRideS);

  ASSERT_NEAR(0.01, 3 * kPitch, r.fm.distanceM());
  ASSERT_NEAR(0.01, 3 * kPitch, r.fm.dist24hM());

  FloorModelState s;
  r.fm.save(&s);
  TEST_ASSERT_EQUAL_UINT16(FLOOR_STATE_SCHEMA, s.schemaVersion);

  FloorMonitor fresh;
  TEST_ASSERT_TRUE(fresh.restore(&s));
  TEST_ASSERT_TRUE(fresh.restored());
  TEST_ASSERT_TRUE(fresh.modelReady());

  // The lifetime odometer is a total and survives; the window is a window and
  // starts empty.
  ASSERT_NEAR(0.01, 3 * kPitch, fresh.distanceM());
  ASSERT_SAME(0.0, fresh.dist24hM());

  // Everything the bootstrap would have had to rebuild came back.
  ASSERT_NEAR(1e-9, kPitch, fresh.pitch());
  TEST_ASSERT_EQUAL_INT(3, fresh.floorIndex());
  TEST_ASSERT_EQUAL_INT(4, fresh.nFloors());
  TEST_ASSERT_EQUAL_UINT32(3u, fresh.trips());
  TEST_ASSERT_EQUAL_UINT32(3u, fresh.stops());
}

void test_restore_rejects_a_foreign_schema(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, kDwellS);
  r.moveTo(0.0, kPitch, kRideS);

  FloorModelState s;
  r.fm.save(&s);
  s.schemaVersion = (uint16_t)(FLOOR_STATE_SCHEMA + 1);

  FloorMonitor fresh;
  TEST_ASSERT_FALSE(fresh.restore(&s));
  TEST_ASSERT_FALSE(fresh.restored());
  TEST_ASSERT_FALSE(fresh.modelReady());
  TEST_ASSERT_FALSE(fresh.restore(NULL));
}

// ===========================================================================
// Hardening 4 and 5 - a runaway model, saved and read back
// ===========================================================================

// Forty floors in one ride puts the index past the 64-slot table. That is a
// runaway model rather than a real building, and the two things it must not do
// are launder itself clean through NVS and alias its out-of-range index onto a
// real floor's learned height.
static void runaway(Rider *r) {
  r->hint(kPitch);
  r->hold(0.0, kDwellS);
  r->ride(0.0, 40 * kPitch, 60);
  r->hold(40 * kPitch, kDwellS);
}

void test_table_clamp_survives_the_nvs_round_trip(void) {
  Rider r;
  runaway(&r);

  TEST_ASSERT_EQUAL_INT(40, r.fm.floorIndex());
  TEST_ASSERT_TRUE(r.fm.tableClamped());

  FloorModelState s;
  r.fm.save(&s);
  // Without this byte the flag is cleared by the round trip while restored_ is
  // set, which declares the model ready and suppresses the bootstrap that
  // would otherwise have rebuilt the ladder from scratch.
  TEST_ASSERT_EQUAL_UINT8(1u, s.tableClamped);

  FloorMonitor fresh;
  TEST_ASSERT_FALSE(fresh.restore(&s));
  TEST_ASSERT_FALSE(fresh.restored());
  TEST_ASSERT_FALSE(fresh.modelReady());
  // A cold boot, so nothing from the runaway record leaked in.
  TEST_ASSERT_EQUAL_UINT32(0u, fresh.trips());
  TEST_ASSERT_EQUAL_UINT32(0u, fresh.stops());
  ASSERT_SAME(0.0, fresh.distanceM());
  TEST_ASSERT_FALSE(fresh.tableClamped());
}

// Index 40 and index 31 share slot 63 once the write side clamps. A read that
// clamped too would hand back floor 31's learned height for floor 40 and say
// nothing about it.
void test_read_paths_do_not_alias_an_out_of_range_index(void) {
  Rider r;
  runaway(&r);

  // The write side did clamp, so the aliasing target is real and occupied:
  // slot 63 holds the runaway landing's height and its stop.
  TEST_ASSERT_EQUAL_INT(32, r.b.nFloors);                  // indices 0..31
  ASSERT_NEAR(0.01, 40 * kPitch, r.fm.height1Based(32));
  TEST_ASSERT_EQUAL_UINT16(1u, r.fm.stopCount1Based(32));

  // Index 40 is 1-based floor 41 and has no slot of its own. It has to fall
  // back rather than return slot 63's contents.
  ASSERT_SAME(0.0, r.fm.height1Based(41));
  TEST_ASSERT_EQUAL_UINT16(0u, r.fm.stopCount1Based(41));

  // datum() and posFloors are the two readouts that go quietly wrong when the
  // read aliases: the car is parked at the landing it just reached, so its
  // position relative to that datum is zero. Aliasing would put it forty
  // floors away and the display would animate to a floor nobody is on.
  ASSERT_NEAR(0.01, 40 * kPitch, r.b.datum);
  ASSERT_NEAR(0.01, 0.0, r.b.posFloors);
}

// The same read paths on a model that never ran away, so the fallback above is
// not quietly firing everywhere.
void test_read_paths_return_learned_values_in_range(void) {
  Rider r;
  r.hint(kPitch);
  r.hold(0.0, kDwellS);
  for (int k = 1; k <= 3; k++) r.moveTo((k - 1) * kPitch, k * kPitch, kRideS);

  TEST_ASSERT_FALSE(r.fm.tableClamped());
  for (int k = 0; k <= 3; k++)
    ASSERT_NEAR(0.05, k * kPitch, r.fm.height1Based(k + 1));
  TEST_ASSERT_EQUAL_UINT16(1u, r.fm.stopCount1Based(4));
  TEST_ASSERT_EQUAL_UINT16(0u, r.fm.stopCount1Based(1));   // boot landing, never confirmed

  // Parked on the top landing, so the live position sits on it.
  ASSERT_NEAR(0.01, 3.0, r.b.posFloors);
  ASSERT_NEAR(0.05, 0.0, r.b.datum);
}

// ===========================================================================
// Numeric helpers, where the port had to reproduce a specific numpy behaviour
// ===========================================================================

void test_round_half_even_matches_python(void) {
  ASSERT_SAME(0.0, floor_detail::roundHalfEven(0.5));
  ASSERT_SAME(2.0, floor_detail::roundHalfEven(1.5));
  ASSERT_SAME(2.0, floor_detail::roundHalfEven(2.5));
  ASSERT_SAME(4.0, floor_detail::roundHalfEven(3.5));
  ASSERT_SAME(-2.0, floor_detail::roundHalfEven(-1.5));
  ASSERT_SAME(-2.0, floor_detail::roundHalfEven(-2.5));
  ASSERT_SAME(1.0, floor_detail::roundHalfEven(0.6));
  ASSERT_SAME(0.0, floor_detail::roundHalfEven(0.4));
}

void test_median_averages_the_two_middle_values(void) {
  const double even[] = { 4.0, 1.0, 3.0, 2.0 };
  ASSERT_SAME(2.5, floor_detail::median(even, 4));
  const double odd[] = { 4.0, 1.0, 3.0 };
  ASSERT_SAME(3.0, floor_detail::median(odd, 3));
  ASSERT_SAME(0.0, floor_detail::median(odd, 0));
}

void test_std_is_the_population_form(void) {
  const double v[] = { 1.0, 2.0, 3.0, 4.0, 5.0 };
  // ddof = 0 gives sqrt(2); the sample std would be sqrt(2.5), 12% larger.
  ASSERT_NEAR(1e-12, sqrt(2.0), floor_detail::stdPopulation(v, 5));
  ASSERT_SAME(0.0, floor_detail::stdPopulation(v, 0));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_lattice_ascent);
  RUN_TEST(test_lattice_descent);
  RUN_TEST(test_direction_while_moving);
  RUN_TEST(test_door_cycle_below_min_jump_stays_on_the_floor);
  RUN_TEST(test_off_lattice_plateau_is_rejected);
  RUN_TEST(test_off_lattice_plateau_resyncs_when_it_persists);
  RUN_TEST(test_bootstrap_replay_rebases_the_index_to_zero);
  RUN_TEST(test_estimate_pitch_needs_three_usable_jumps);
  RUN_TEST(test_trusted_rate_passes_clustered_observations_through);
  RUN_TEST(test_trusted_rate_shrinks_scattered_observations);
  RUN_TEST(test_trusted_rate_is_zero_before_three_observations);
  RUN_TEST(test_non_finite_first_sample_does_not_poison_the_reference);
  RUN_TEST(test_non_finite_sample_reserves_the_previous_broadcast);
  RUN_TEST(test_clamp_tolerates_swapped_bounds);
  RUN_TEST(test_millis_wrap_does_not_slam_the_reference);
  RUN_TEST(test_restore_reports_the_shorter_odometer_window);
  RUN_TEST(test_restore_rejects_a_foreign_schema);
  RUN_TEST(test_table_clamp_survives_the_nvs_round_trip);
  RUN_TEST(test_read_paths_do_not_alias_an_out_of_range_index);
  RUN_TEST(test_read_paths_return_learned_values_in_range);
  RUN_TEST(test_round_half_even_matches_python);
  RUN_TEST(test_median_averages_the_two_middle_values);
  RUN_TEST(test_std_is_the_population_form);
  return UNITY_END();
}
