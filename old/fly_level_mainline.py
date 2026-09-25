#!/usr/bin/env python3
"""Closed-loop JSBSim smoke flight against a mainline INAV SITL.

Sequence: settle AHRS (frozen plant) -> arm -> 25 s level cruise in ANGLE
mode with the bundled c172p. Passes if the FC arms, its attitude estimate
tracks JSBSim ground truth, and the plane holds roughly level flight.

Prereqs: SITL running on tcp:5760, provisioned via provision_mainline.py
(and restarted after provisioning).

Usage: python3 fly_level_mainline.py [--flip-ail] [--flip-ele] [--flip-rud]

c172p NEEDS --flip-ele: its FCS maps positive elevator-cmd-norm to nose-down,
while INAV's positive stab_pitch means pitch-up. Without the flip the ANGLE
controller is in positive feedback and the plane dives into the ground
(verified 2026-07-12: without flip it crashed in ~15 s; with --flip-ele all
checks pass — mean roll delta 0.1 deg, pitch 2.1 deg, IAS 72-79 kts).
"""
import os
import sys
import time

BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "inav-sitl-bench")
sys.path.insert(0, BENCH)

from msp import MspClient
from hitl import sim_step
from jsbsim_plant import JSBSimPlant
from bench import arming_flags, wait_boot_calibration, FLAG_ARMED

FLIP_AIL = "--flip-ail" in sys.argv
FLIP_ELE = "--flip-ele" in sys.argv
FLIP_RUD = "--flip-rud" in sys.argv

RC_LOW, RC_MID, RC_HIGH = 1000, 1500, 2000
DT = 0.001


def rc_ch(thr=RC_LOW, arm=RC_LOW, angle=RC_LOW, ele=RC_MID, ail=RC_MID, rud=RC_MID):
    # AETR + AUX1(arm) AUX2(angle) AUX3 AUX4
    return [ail, ele, thr, rud, arm, angle, RC_LOW, RC_LOW]


m = MspClient()
plant = JSBSimPlant(model="c172p", alt_ft=1500, kts=60)
samples = []          # (t, fc_r, fc_p, js_r, js_p, ias_kts, alt_m) during cruise


def loop(secs, phase, rc, freeze=False, record=False, print_every=2.0):
    last = 0.0
    t0 = time.time()
    while time.time() - t0 < secs:
        it0 = time.perf_counter()
        r = sim_step(m, plant.acc_mg(), plant.gyro_dps16(), rc, baro_pa=plant.baro_pa())
        ail = -r.stab_roll if FLIP_AIL else r.stab_roll
        ele = -r.stab_pitch if FLIP_ELE else r.stab_pitch
        rud = -r.stab_yaw if FLIP_RUD else r.stab_yaw
        thr = (r.stab_throttle + 1.0) / 2.0
        if freeze:
            plant._a_earth = (0.0, 0.0, 0.0)   # static: pure gravity, zero rates
        else:
            plant.set_controls(ail, ele, rud, thr)
            plant.step(dt=DT)
        jr, jp, jy = plant.rpy()
        if record:
            samples.append((time.time() - t0, r.att_roll_deg, r.att_pitch_deg,
                            jr, jp, plant.ias_kts(), plant.z))
        if time.time() - last > print_every:
            print(f"  [{phase:7}] FC {r.att_roll_deg:+6.1f}/{r.att_pitch_deg:+6.1f} | "
                  f"JS {jr:+6.1f}/{jp:+6.1f} | IAS {plant.ias_kts():3.0f} kts "
                  f"alt {plant.z:5.0f} m armed={r.armed}")
            last = time.time()
        rem = DT - (time.perf_counter() - it0)
        if rem > 0:
            time.sleep(rem)
    return r


print("=== boot calibration ===")
wait_boot_calibration(m)
print("calibration done, flags=0x%X" % arming_flags(m))

print("=== settle AHRS (frozen plant, 5 s) ===")
loop(5, "settle", rc_ch(angle=RC_HIGH), freeze=True)

print("=== arm (toggle cycles) ===")
t0 = time.time()
while not (arming_flags(m) & FLAG_ARMED) and time.time() - t0 < 20:
    loop(1.0, "armL", rc_ch(arm=RC_LOW, angle=RC_HIGH), freeze=True, print_every=9)
    loop(1.2, "armH", rc_ch(arm=RC_HIGH, angle=RC_HIGH), freeze=True, print_every=9)
armed = bool(arming_flags(m) & FLAG_ARMED)
print(f"ARMED: {armed} flags=0x{arming_flags(m):X}")
if not armed:
    print("FAIL: never armed")
    sys.exit(1)

print("=== level cruise, ANGLE, 25 s ===")
loop(25, "cruise", rc_ch(thr=1650, arm=RC_HIGH, angle=RC_HIGH), record=True)

# --- verdict ---------------------------------------------------------------
tail = [s for s in samples if s[0] > 10.0]       # ignore the entry transient
mean = lambda xs: sum(xs) / len(xs)
d_roll = mean([abs(s[1] - s[3]) for s in tail])
d_pitch = mean([abs(s[2] - s[4]) for s in tail])
roll_js = mean([abs(s[3]) for s in tail])
ias_min, ias_max = min(s[5] for s in tail), max(s[5] for s in tail)
alt_end = tail[-1][6]
alt_start = samples[0][6]

print(f"\nattitude agreement (t>10s): mean |dRoll| {d_roll:.1f} deg, "
      f"mean |dPitch| {d_pitch:.1f} deg")
print(f"level hold: mean |JS roll| {roll_js:.1f} deg")
print(f"IAS range {ias_min:.0f}..{ias_max:.0f} kts, alt {alt_start:.0f} -> {alt_end:.0f} m")

checks = [
    ("FC/JSBSim roll agree < 15 deg", d_roll < 15),
    ("FC/JSBSim pitch agree < 15 deg", d_pitch < 15),
    ("wings held level < 15 deg", roll_js < 15),
    ("IAS stayed 35..110 kts", ias_min > 35 and ias_max < 110),
    ("altitude kept (lost < 150 m)", alt_end > alt_start - 150),
]
ok = all(passed for _, passed in checks)
for name, passed in checks:
    print(("PASS " if passed else "FAIL ") + name)
print("\n" + ("LEVEL-FLIGHT PASS" if ok else "LEVEL-FLIGHT FAIL"))
sys.exit(0 if ok else 1)
