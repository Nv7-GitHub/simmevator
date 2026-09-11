//
// Replay harness: the 3.04 h reference capture through the real FloorMonitor.
//
// This links src/floor_monitor.h itself, not a host-side reimplementation of
// it, so a PASS here is evidence about the code the car actually runs. Samples
// go in one at a time in file order and nothing is read ahead of the monitor -
// the harness has no more information than the node does.
//
// The output is diffed sample by sample against floor_algorithm.py by
// sim/compare_to_python.py, which means the CSV has to be byte-comparable with
// what pandas writes for the golden trace. That is most of this file: the
// algorithm call is three lines and the rest is reproducing Python's rounding
// and float formatting exactly. A formatting mismatch reads as an algorithm
// divergence and costs an afternoon, so it is worth the code.
//

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "floor_monitor.h"

// Overridable so the harness can be pointed at a re-prepared capture; the
// default is the path relative to the repo root, where pio leaves you.
static const char *kDefaultCapture = "sim/filled_capture.csv";

// ===========================================================================
// Python-compatible number formatting
//
// compare_to_python.py builds the golden CSV with pandas, which writes a
// float64 with str(), which is repr(), which is the shortest decimal string
// that reads back as the same double. printf has no such mode: "%g" drops
// digits that matter and "%.17g" adds digits that do not. Both are reproduced
// here from the two primitives that are guaranteed correctly rounded.
// ===========================================================================

// Python's round(x, n): correctly rounded to n decimals, ties to even. That is
// exactly what "%.*f" does on a correctly rounding libc, so round-trip through
// the decimal text rather than doing it in binary - x * 1000 introduces an
// error of its own before the rounding ever happens.
static double pyRound(double v, int n) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.*f", n, v);
  return strtod(buf, nullptr);
}

// Shortest round-tripping decimal digits, found by asking for one more each
// time until the text reads back bit-identical. At most 17 attempts per number
// and 6 numbers per row over 10950 rows, which is nothing next to the file I/O.
//
// The minimal precision never leaves a trailing zero: if "1.20e+00" read back
// correctly then so would "1.2e+00", and the loop would have stopped a digit
// earlier. So the digit string here is the same one Python's dtoa produces.
static int shortestDigits(double v, char *digits, int cap, int *decpt) {
  char buf[64];
  int prec = 0;
  for (; prec < 17; prec++) {
    snprintf(buf, sizeof(buf), "%.*e", prec, v);
    if (strtod(buf, nullptr) == v) break;
  }
  const char *p = buf;
  if (*p == '-' || *p == '+') p++;

  int n = 0;
  for (; *p && *p != 'e' && *p != 'E'; p++) {
    if (*p == '.') continue;
    if (n < cap - 1) digits[n++] = *p;
  }
  digits[n] = '\0';
  // decpt is the position of the decimal point among those digits, matching
  // CPython's format_float_short: 1.234e2 has three digits before the point.
  *decpt = (*p ? atoi(p + 1) : 0) + 1;
  return n;
}

// repr() of a finite double. CPython switches to exponential when the decimal
// point falls outside (-4, 16]; inside that band it writes plain digits and
// appends ".0" to anything integral. Nothing in this trace reaches either
// cutoff, but a drift rate that underflows to 1e-05 does, and the golden CSV
// contains those - so the rule is implemented rather than assumed away.
static void pyRepr(double v, char *out, int cap) {
  if (!isfinite(v)) {                       // never produced here; fail loudly
    snprintf(out, (size_t)cap, "%s", isnan(v) ? "nan" : (v > 0 ? "inf" : "-inf"));
    return;
  }

  const char *sign = signbit(v) ? "-" : "";  // signbit, so round() to -0.0 keeps it
  if (v == 0.0) {
    snprintf(out, (size_t)cap, "%s0.0", sign);
    return;
  }

  char d[32];
  int decpt = 0;
  const int n = shortestDigits(fabs(v), d, (int)sizeof(d), &decpt);

  if (decpt < -3 || decpt > 16) {
    const int e = decpt - 1;
    if (n > 1) {
      snprintf(out, (size_t)cap, "%s%c.%se%c%02d", sign, d[0], d + 1,
               e < 0 ? '-' : '+', abs(e));
    } else {
      snprintf(out, (size_t)cap, "%s%ce%c%02d", sign, d[0], e < 0 ? '-' : '+', abs(e));
    }
  } else if (decpt <= 0) {
    snprintf(out, (size_t)cap, "%s0.%*s%s", sign, -decpt, "", d);
  } else if (decpt >= n) {
    snprintf(out, (size_t)cap, "%s%s%*s.0", sign, d, decpt - n, "");
  } else {
    snprintf(out, (size_t)cap, "%s%.*s.%s", sign, decpt, d, d + decpt);
  }

  // The two padded cases above insert spaces, because printf has no zero-fill
  // for strings. Turn them into the zeros they stand for.
  for (char *p = out; *p; p++)
    if (*p == ' ') *p = '0';
}

// Round then repr, which is the order pandas sees: floor_algorithm.py rounds
// inside its output helper and pandas formats whatever came out.
static void pyField(double v, int decimals, char *out, int cap) {
  pyRepr(pyRound(v, decimals), out, cap);
}

// ===========================================================================
// Capture reader
//
// Four known columns and one known row shape, so a hand-rolled split beats a
// dependency. Anything unparseable is a hard error: a replay that silently
// skipped samples would still produce a plausible-looking CSV.
// ===========================================================================

struct Sample {
  double t;
  double alt;
};

static bool parseRow(char *line, Sample *s) {
  // t,pressure_pa,altitude_m,filled - pressure and the filled flag are the
  // preparation stage's working, not inputs to the algorithm.
  const char *field[4] = {nullptr, nullptr, nullptr, nullptr};
  int n = 0;
  field[n++] = line;
  for (char *p = line; *p && n < 4; p++) {
    if (*p == ',') {
      *p = '\0';
      field[n++] = p + 1;
    }
  }
  if (n < 3) return false;

  char *end = nullptr;
  s->t = strtod(field[0], &end);
  if (end == field[0]) return false;
  s->alt = strtod(field[2], &end);
  if (end == field[2]) return false;
  return true;
}

int main(int argc, char **argv) {
  const char *outPath = (argc > 1) ? argv[1] : nullptr;
  const char *inPath = (argc > 2) ? argv[2] : kDefaultCapture;

  FILE *in = fopen(inPath, "r");
  if (!in) {
    fprintf(stderr, "replay: cannot open capture %s\n", inPath);
    return 2;
  }
  FILE *out = stdout;
  if (outPath) {
    out = fopen(outPath, "w");
    if (!out) {
      fprintf(stderr, "replay: cannot write %s\n", outPath);
      fclose(in);
      return 2;
    }
  }

  fprintf(out, "t,floor,moving,confidence,n_floors,pitch,"
               "trips,stops,distance_m,drift_rate_mps,datum\n");

  FloorMonitor mon;
  char line[256];
  long rows = 0;
  FloorBroadcast b = {};

  if (!fgets(line, sizeof(line), in)) {
    fprintf(stderr, "replay: %s is empty\n", inPath);
    fclose(in);
    return 2;
  }
  // Header, unless the file was handed over without one.
  if (strncmp(line, "t,", 2) != 0) rewind(in);

  while (fgets(line, sizeof(line), in)) {
    line[strcspn(line, "\r\n")] = '\0';
    if (!line[0]) continue;

    Sample s;
    if (!parseRow(line, &s)) {
      fprintf(stderr, "replay: malformed row %ld in %s\n", rows + 1, inPath);
      fclose(in);
      return 2;
    }

    b = mon.update(s.t, s.alt);
    rows++;

    char ct[32], cconf[32], cpitch[32], cdist[32], crate[32], cdatum[32];
    pyRepr(b.t, ct, sizeof(ct));              // t is passed through unrounded
    pyField(b.confidence, 3, cconf, sizeof(cconf));
    pyField(b.pitch, 3, cpitch, sizeof(cpitch));
    pyField(b.distanceM, 1, cdist, sizeof(cdist));
    pyField(b.driftRateMps, 5, crate, sizeof(crate));
    pyField(b.datum, 3, cdatum, sizeof(cdatum));

    fprintf(out, "%s,%d,%d,%s,%d,%s,%u,%u,%s,%s,%s\n", ct, b.floor,
            b.moving ? 1 : 0, cconf, b.nFloors, cpitch, b.trips, b.stops,
            cdist, crate, cdatum);
  }

  fclose(in);
  if (out != stdout) fclose(out);

  // The summary is the gate values from spec 7.2, so someone running this by
  // hand gets the verdict without going through the comparator.
  fprintf(stderr,
          "replay: %ld samples  floors=%d  pitch=%.3f m  trips=%u  stops=%u  "
          "distance=%.1f m\n",
          rows, b.nFloors, b.pitch, b.trips, b.stops, b.distanceM);
  if (mon.tableClamped())
    fprintf(stderr, "replay: floor index left the 64-slot table - model ran away\n");
  fprintf(stderr, "replay: %u samples rejected off-lattice, %u passed through\n",
          mon.rejects(), mon.passthroughs());
  return 0;
}
