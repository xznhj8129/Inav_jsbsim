#!/usr/bin/env python3
"""Example: the Rascal leaves a launcher: a catapult in ACRO or in INAV's launch mode, or an unpowered rail.

    python3 examples/launch.py /path/to/inav/build_sitl/bin/SITL.elf [catapult-acro|catapult-nav|rail]

catapult-acro (default): no flight mode while held, so nothing self-levels against the attitude the
    catapult holds; the motor runs up before the push, and ANGLE takes over once it is flying.
catapult-nav: INAV's NAV LAUNCH waits for the throw and flies the climb-out itself.
rail: an unpowered launch rail, like a model rocket's rod: nothing pushes, the motor's own thrust takes it
    up the rail once it beats weight and drag. INAV only arms within small_angle (25 deg) of level; a
    steeper rail needs it raised: Bench(cli=["set small_angle = 90"]).

All run on SITL's stepped clock with --faster; drop stepped/faster for wall time.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from inavbench import Bench, Catapult, LaunchRail


def report(b, phase):
    t = b.truth()
    print(f"{b.elapsed:5.1f}s  {phase:<8} height {t['agl_m']:5.1f} m  airspeed {t['ias_kts']:4.0f} kt  "
          f"roll {t['roll']:+6.1f} (FC {b.fc.roll:+6.1f})  pitch {t['pitch']:+5.1f} (FC {b.fc.pitch:+5.1f})  "
          f"motor {b.motors[0]}")


def fly(b, seconds, phase):
    for _ in range(int(seconds * 2)):
        b.run(0.5)
        report(b, phase)


style = sys.argv[2] if len(sys.argv) > 2 else "catapult-acro"
launchers = {
    "catapult-acro": Catapult(pitch_deg=10, length_m=2.0, speed_ms=15.0, height_m=1.5),
    "catapult-nav": Catapult(pitch_deg=10, length_m=2.0, speed_ms=15.0, height_m=1.5),
    # the Rascal's static thrust (9.3 lb) is below its weight (14.5 lb): it only climbs a shallow rail, and
    # leaves it below its stall speed, sagging to about half a metre before it flies
    "rail": LaunchRail(pitch_deg=20, length_m=3.0, height_m=1.0),
}

with Bench("rascal", sitl=sys.argv[1], launcher=launchers[style], stepped=True, faster=True) as b:
    if style == "catapult-nav":
        # NAV LAUNCH with nav_fw_launch_idle_thr spins the motor up to idle over 1.5 s before the throw
        b.launch(throttle=0.7, modes=("NAV LAUNCH",), wait_s=2.0)
        report(b, "launched")
        fly(b, 6, "climb")
    else:
        b.launch(throttle=0.8 if style == "catapult-acro" else 1.0, wait_s=1.0)
        report(b, "launched")
        fly(b, 1, "acro")
        b.mode("ANGLE")
        b.sticks(throttle=0.8)
        fly(b, 5, "angle")
