"""Check the bench itself against a SITL build: sensors, outputs and nav all line up.

    python3 -m inavbench.selftest /path/to/inav/build_sitl/bin/SITL.elf [quad|plane] [--stepped] [--rate=HZ]

--stepped flies on SITL's stepped clock (Bench(stepped=True)) instead of wall time.
--rate sets the bench's tick rate (Bench(rate_hz=...)); stepped, each tick is a 1/HZ step.

This tests the bench, not a firmware feature. Run it when you switch firmware
or change the bench; if it fails, fix that before trusting any other result.
"""
import math
import statistics
import sys
import time

from .bench import RATE_HZ, Bench
from .vehicles import VEHICLES


def _angle_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _attitude_error(b):
    t = b.truth()
    return max(abs(b.fc.roll - t["roll"]), abs(b.fc.pitch - t["pitch"]), _angle_diff(b.fc.yaw, t["yaw"]))


def _from_home(b):
    t = b.truth()
    return math.hypot(t["north_m"], t["east_m"])


def quad(sitl, stepped, rate_hz):
    checks = []
    with Bench("quad", sitl=sitl, stepped=stepped, rate_hz=rate_hz, faster=stepped) as b:
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


# MSP_NAV_STATUS state while INAV's RTH lands, and once it has landed (navigation.h, MW_NAV_STATE_*)
NAV_STATE_LAND_IN_PROGRESS, NAV_STATE_LANDED = 9, 10


def _rth_end(b):
    """RTH ends either still flying or landed by INAV (nav_rth_allow_landing) near home; a crash already
    stopped the bench with GroundStrike."""
    agl, state = b.truth()["agl_m"], b.nav_status()["state"]
    if state in (NAV_STATE_LAND_IN_PROGRESS, NAV_STATE_LANDED) and agl < 1.0:
        landed_at = _from_home(b)
        return (f"RTH landed it {landed_at:.0f} m from home", landed_at < 200)
    return (f"still flying ({agl:.0f} m above ground)", agl > 50)


def plane(sitl, stepped, rate_hz, vehicle="plane"):
    """A vehicle with a default launcher takes off from it in NAV LAUNCH; the rest start in the air."""
    checks = []
    launcher = VEHICLES[vehicle].launcher
    with Bench(vehicle, sitl=sitl, stepped=stepped, rate_hz=rate_hz, launcher=launcher, faster=stepped) as b:
        if launcher:
            # INAV's launch spins the motor up to its idle over 1.5 s before it looks for the throw
            b.launch(throttle=0.7, modes=("NAV LAUNCH",), wait_s=2.0)
            b.run(6)
            agl = b.truth()["agl_m"]
            checks.append((f"launch climbed out ({agl:.0f} m after 6 s)", agl > 15))
            b.mode("NAV LAUNCH", on=False)
            b.mode("ANGLE")
        else:
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
                   _rth_end(b)]
    return checks


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    sitl = sys.argv[1]
    options = [arg for arg in sys.argv[2:] if arg.startswith("--")]
    stepped = "--stepped" in options
    rate_hz = next((int(arg.split("=", 1)[1]) for arg in options if arg.startswith("--rate=")), RATE_HZ)
    which = [arg for arg in sys.argv[2:] if not arg.startswith("--")] or ["quad", "plane"]
    ok = True
    for name in which:
        print(f"--- {name} at {rate_hz} Hz" + (" (stepped clock)" if stepped else ""))
        started = time.monotonic()
        tests = {"quad": quad, "plane": plane, "rascal": lambda *a: plane(*a, vehicle="rascal")}
        for text, passed in tests[name](sitl, stepped, rate_hz):
            print(("PASS " if passed else "FAIL ") + text)
            ok &= passed
        print(f"     {time.monotonic() - started:.1f} s wall")
    print("SELFTEST " + ("PASS" if ok else "FAIL"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
