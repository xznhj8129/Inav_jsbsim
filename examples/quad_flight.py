#!/usr/bin/env python3
"""Example: a quad takes off, holds position, flies away and comes home by RTH.

    python3 examples/quad_flight.py /path/to/inav/build_sitl/bin/SITL.elf

Prints what the FC thinks next to what the vehicle is really doing, once a
second, and writes the whole flight to quad_flight.csv.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from inavbench import Bench


def report(b, phase):
    t = b.truth()
    alt, _ = b.altitude()
    modes = ", ".join(sorted(b.active_modes() - {"ARM"})) or "-"
    print(f"{b.elapsed:5.0f}s  {phase:<9} height {t['agl_m']:5.1f} m (FC {alt:5.1f})  "
          f"from home {math.hypot(t['north_m'], t['east_m']):5.1f} m  "
          f"roll {t['roll']:+5.1f} (FC {b.fc.roll:+5.1f})  pitch {t['pitch']:+5.1f} (FC {b.fc.pitch:+5.1f})  "
          f"{'ARMED' if b.fc.armed else 'disarmed'}  [{modes}]")


def fly(b, seconds, phase):
    for _ in range(int(seconds)):
        b.run(1)
        report(b, phase)


with Bench("quad", sitl=sys.argv[1], log="quad_flight.csv") as b:
    report(b, "on ground")
    b.mode("ANGLE")
    b.arm()

    b.sticks(throttle=0.62)
    fly(b, 4, "climb")

    b.sticks(throttle=0.5)
    b.mode("NAV POSHOLD")
    fly(b, 5, "poshold")

    b.sticks(pitch=-0.6)                  # stick back: nose up, flies backwards (south)
    fly(b, 5, "away")
    b.sticks(pitch=0)

    b.mode("NAV RTH")
    while b.fc.armed:
        fly(b, 1, "RTH")

print("flight log: quad_flight.csv")
