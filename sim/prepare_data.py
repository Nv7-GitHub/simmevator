#!/usr/bin/env python3
"""Gap-fill the reference capture onto an even 1 Hz grid.

The capture is a radio log, not a sensor log: 49 dropouts totalling 570 samples,
worst case a 41 s hole, which is 40 missing 1 Hz samples between the two real
readings either side of it - the summary below counts the samples, so it prints
40. The real node holds the BMP390 on its own I2C bus, so it never
sees any of them - the gaps are an artefact of how the data reached the laptop.
Replaying the capture as-is would therefore measure the port against a stream
the firmware can never be handed, and the algorithm's stillness test is a
trailing 5-sample window that a missing second silently reshapes.

So the gaps are filled and the filled samples are marked. The 41 s hole becomes
an invented smooth ride; that is visible in the `filled` column rather than
hidden.

Interpolation is on PRESSURE, not on altitude. Altitude is a nonlinear function
of pressure, so interpolating the derived quantity puts the fill slightly off
the curve the sensor would have traced - small enough to look fine, large enough
to be wrong. Altitude is recomputed from the interpolated pressure with the same
barometric formula the firmware uses, so the replay input is consistent with
what the device would have produced.
"""

import os

import numpy as np
import pandas as pd

CAPTURE = ("/Users/nv/Documents/Embedded/elevatormons/data/"
           "baro-20260910-195011.csv")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "filled_capture.csv")

# Sea-level reference and exponent from the firmware's altitude conversion.
# p0 is nominal, never calibrated: the algorithm reads floors from differences,
# so a wrong p0 shifts every altitude by a constant and changes nothing.
P0_PA = 101325.0
BARO_EXP = 1.0 / 5.255
BARO_SCALE = 44330.0


def altitude_from_pressure(pressure_pa):
    return BARO_SCALE * (1.0 - (pressure_pa / P0_PA) ** BARO_EXP)


def main():
    cap = pd.read_csv(CAPTURE)
    t_src = cap["host_unix"].to_numpy(float)
    p_src = cap["pressure_pa"].to_numpy(float)

    # host_unix carries the receiver's arrival stamp, which for this capture
    # lands on exact whole seconds - the transmitter's 1 Hz cadence survives the
    # link. The grid is therefore the same phase as the real samples and every
    # original sample keeps its own value rather than being resampled off it.
    t0 = t_src[0]
    n = int(round(t_src[-1] - t0)) + 1
    t = np.arange(n, dtype=float)

    pressure = np.interp(t, t_src - t0, p_src)
    altitude = altitude_from_pressure(pressure)

    # A grid point is real if a source sample sits within half a sample period
    # of it. Half a period rather than an exact match so that clock jitter in a
    # future capture does not mark every sample invented.
    filled = np.ones(n, dtype=int)
    idx = np.rint(t_src - t0).astype(int)
    keep = (np.abs((t_src - t0) - idx) <= 0.5) & (idx >= 0) & (idx < n)
    filled[idx[keep]] = 0

    pd.DataFrame({
        "t": t,
        "pressure_pa": np.round(pressure, 4),
        "altitude_m": np.round(altitude, 6),
        "filled": filled,
    }).to_csv(OUT, index=False)

    # Longest run of invented samples: run-length encode the filled flag.
    runs, cur = [], 0
    for f in filled:
        if f:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)

    print(f"source     {len(cap)} samples, {t_src[-1] - t0:.0f} s span")
    print(f"grid       {n} samples at 1 Hz")
    print(f"invented   {int(filled.sum())} samples in {len(runs)} gaps "
          f"({100.0 * filled.sum() / n:.2f}% of the grid)")
    if runs:
        print(f"longest    {max(runs)} s")
    print(f"wrote      {OUT}")


if __name__ == "__main__":
    main()
