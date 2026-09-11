#!/usr/bin/env python3
"""Generate the golden broadcast trace, and diff a C++ replay against it.

The Python implementation in elevatormons is the definition of the algorithm,
so this script imports it from that repo rather than copying it - a copy would
drift, and a drifted golden reference is worse than none. elevatormons is the
bring-up bench and is never modified from here.

Run with no arguments to (re)generate sim/golden_broadcasts.csv from
sim/filled_capture.csv and print the acceptance gates. Run with a C++ replay CSV
as argv[1] to diff it against the golden trace. Exit status is the verdict.

The spec's gate values were measured on the raw capture; this runs on the
gap-filled grid, so the numbers here are the ones the port is actually held to.
They are printed as measured next to what is required, and nothing is quietly
adjusted to match.
"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, "/Users/nv/Documents/Embedded/elevatormons/tools")
from floor_algorithm import FloorMonitor  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
CAPTURE = os.path.join(HERE, "filled_capture.csv")
GOLDEN = os.path.join(HERE, "golden_broadcasts.csv")

COLUMNS = ["t", "floor", "moving", "confidence", "n_floors", "pitch",
           "trips", "stops", "distance_m", "drift_rate_mps", "datum"]

# Spec section 7.2. Tolerances are on the quantity, not on the comparison: the
# pitch sweep steps 5e-4 m so 1 mm is two steps, and 0.1% of the odometer is
# about one floor of travel out of 1186.
#
# trips and stops are 357/255 here, not the 356/256 that ALGORITHM.md section 5
# reports. That is not a port defect and not a tuning difference: section 5
# measured the *raw* capture, which has 49 dropouts in it, and this runs on the
# gap-filled stream the node would actually have seen. Filling the worst gap - a
# 41 s hole - invents one smooth ride across it, which is one extra floor-to-floor
# move and one fewer dwell long enough to confirm. The Python reference produces
# 357/255 on this input too; both are checked below, so a future divergence shows
# up as the C++ disagreeing with the Python rather than as a moved goalpost.
#
# The real gate is the floor-sequence comparison, not these. These only bound how
# far the gap-filled run may drift from the published measurement.
GATES = {
    "n_floors": (10, 0),
    "pitch": (2.871, 0.001),
    "distance_m": (3409.0, 3409.0 * 0.001),
    "trips": (357, 1),
    "stops": (255, 1),
}

# How many samples of context to show either side of a floor divergence. The
# useful question is always "what was the altitude doing here", so the diff
# prints the trace rather than just the index.
CONTEXT = 3
MAX_DIVERGENCES = 20


def run_reference(capture):
    mon = FloorMonitor()
    rows = []
    for t, alt in zip(capture["t"].to_numpy(float),
                      capture["altitude_m"].to_numpy(float)):
        b = mon.update(float(t), float(alt))
        rows.append((b.t, b.floor, int(b.moving), b.confidence, b.n_floors,
                     b.pitch, b.trips, b.stops, b.distance_m,
                     b.drift_rate_mps, b.datum))
    return pd.DataFrame(rows, columns=COLUMNS)


def check_gates(final):
    """Returns (ok, lines). Reports measured against required either way."""
    ok, lines = True, []
    for name, (want, tol) in GATES.items():
        got = float(final[name])
        good = abs(got - want) <= tol
        ok &= good
        lines.append(f"  {'PASS' if good else 'FAIL'}  {name:<12} "
                     f"{got:>10.3f}   required {want:g}"
                     + (f" +/- {tol:g}" if tol else ""))
    return ok, lines


def diff_floors(golden, cpp, capture):
    """Sample-for-sample floor comparison, with the altitude around each miss."""
    n = min(len(golden), len(cpp))
    if len(golden) != len(cpp):
        print(f"FAIL  length: python {len(golden)} samples, "
              f"C++ {len(cpp)} samples; comparing the first {n}")

    exp = golden["floor"].to_numpy(int)[:n]
    got = cpp["floor"].to_numpy(int)[:n]
    bad = np.flatnonzero(exp != got)
    if not len(bad):
        print(f"PASS  floor sequence identical over {n} samples")
        return len(golden) == len(cpp)

    print(f"FAIL  floor sequence differs at {len(bad)} of {n} samples; "
          f"first {min(len(bad), MAX_DIVERGENCES)}:")
    t = capture["t"].to_numpy(float)
    alt = capture["altitude_m"].to_numpy(float)
    fill = capture["filled"].to_numpy(int)
    for i in bad[:MAX_DIVERGENCES]:
        print(f"  t={t[i]:.0f}s  expected {exp[i]}  got {got[i]}")
        lo, hi = max(0, i - CONTEXT), min(n, i + CONTEXT + 1)
        for k in range(lo, hi):
            print(f"        {'>' if k == i else ' '} t={t[k]:8.0f}  "
                  f"alt={alt[k]:8.3f}{'  filled' if fill[k] else ''}  "
                  f"py_floor={exp[k]}  cpp_floor={got[k]}")
    return False


def main():
    capture = pd.read_csv(CAPTURE)
    golden = run_reference(capture)
    golden.to_csv(GOLDEN, index=False)
    print(f"golden     {len(golden)} rows -> {GOLDEN}")

    final = golden.iloc[-1]
    ok, lines = check_gates(final)
    print("python reference against the spec 7.2 gates:")
    print("\n".join(lines))

    if len(sys.argv) > 1:
        cpp = pd.read_csv(sys.argv[1])
        missing = [c for c in COLUMNS if c not in cpp.columns]
        if missing:
            print(f"FAIL  C++ output is missing columns: {missing}")
            sys.exit(1)
        print(f"\nC++ replay {sys.argv[1]}:")
        ok &= diff_floors(golden, cpp, capture)
        cpp_ok, cpp_lines = check_gates(cpp.iloc[-1])
        ok &= cpp_ok
        print("\n".join(cpp_lines))

    print("\nVERDICT: " + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
