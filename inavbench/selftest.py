"""Check the bench itself against a SITL build: sensors, outputs and nav all line up.

    python3 -m inavbench.selftest /path/to/inav/build_sitl/bin/SITL.elf [quad|plane]

This tests the bench, not a firmware feature. Run it when you switch firmware
or change the bench; if it fails, fix that before trusting any other result.
"""
import math
import statistics
import sys

from .bench import Bench


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _attitude_error(b):
    t = b.truth()
    return max(abs(b.fc.roll - t["roll"]), abs(b.fc.pitch - t["pitch"]), _angle_diff(b.fc.yaw, t["yaw"]))


def _from_home(b):
    t = b.truth()
    return math.hypot(t["north_m"], t["east_m"])


def quad(sitl):
    checks = []
    with Bench("quad", sitl=sitl) as b:
        b.mode("ANGLE")
        b.arm()
        b.sticks(throttle=0.62)
        b.run(4)
        b.sticks(throttle=0.5)
        b.mode("NAV POSHOLD")
        b.run(3)
        worst_att, worst_pos, worst_alt = 0.0, 0.0, 0.0
        tilt = []
        for _ in range(32):
            b.run(0.25)
            tilt += [b.truth()["roll"], b.truth()["pitch"]]
            worst_att = max(worst_att, _attitude_error(b))
            worst_pos = max(worst_pos, _from_home(b))
            worst_alt = max(worst_alt, abs(b.altitude()[0] - (b.truth()["agl_m"] - 0.15)))
        checks += [("took off and climbed above 20 m", b.truth()["agl_m"] > 20),
                   (f"FC attitude within 5 deg of truth in POSHOLD (worst {worst_att:.1f})", worst_att < 5),
                   # an unsteady sim loop (too low a tick rate) shows up as a wobble, not as drift
                   (f"steady in POSHOLD: true roll/pitch spread under 1 deg ({statistics.pstdev(tilt):.2f})",
                    statistics.pstdev(tilt) < 1),
                   (f"POSHOLD within 2 m of the take-off point (worst {worst_pos:.1f})", worst_pos < 2),
                   (f"FC altitude within 1.5 m of truth (worst {worst_alt:.2f})", worst_alt < 1.5)]

        b.sticks(pitch=-0.6)
        b.run(5)
        b.sticks(pitch=0)
        b.run(2)
        away = _from_home(b)
        b.mode("NAV RTH")
        b.run(90, until=lambda x: not x.fc.armed, check_every=0.5)
        checks += [(f"flew away ({away:.0f} m)", away > 30),
                   ("RTH landed and disarmed", not b.fc.armed),
                   (f"landed within 3 m of home ({_from_home(b):.1f} m)", _from_home(b) < 3)]
    return checks


def plane(sitl):
    checks = []
    with Bench("plane", sitl=sitl) as b:
        b.mode("ANGLE")
        b.arm()
        b.sticks(throttle=0.7)
        b.release()
        worst = 0.0
        for stick in (0.0, 0.5, 0.0, -0.5, 0.0):
            b.sticks(roll=stick)
            b.run(2)
            worst = max(worst, _attitude_error(b))
        # a banking plane's AHRS legitimately lags by a few degrees; a sensor frame
        # or sign error shows up as tens of degrees
        checks.append((f"FC attitude within 8 deg of truth through banks (worst {worst:.1f})", worst < 8))

        b.mode("NAV ALTHOLD")
        b.run(20)
        away = _from_home(b)
        b.mode("NAV RTH")
        closest = away
        for _ in range(45):
            b.run(2)
            closest = min(closest, _from_home(b))
        checks += [(f"flew away ({away:.0f} m)", away > 800),
                   (f"RTH came back within 200 m of home (closest {closest:.0f} m)", closest < 200),
                   ("still flying", b.truth()["agl_m"] > 50)]
    return checks


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sitl = sys.argv[1]
    which = sys.argv[2:] or ["quad", "plane"]
    ok = True
    for name in which:
        print(f"--- {name}")
        for text, passed in {"quad": quad, "plane": plane}[name](sitl):
            print(("PASS " if passed else "FAIL ") + text)
            ok &= passed
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
