#!/usr/bin/env python3
"""Get a quad or a plane airborne, then hold it in NAV POSHOLD + GCS NAV for a GCS to take over.

    python3 examples/gcsnav.py quad  /path/to/inav/build_sitl/bin/SITL.elf
    python3 examples/gcsnav.py plane /path/to/inav/build_sitl/bin/SITL.elf --gcs PHONE_IP

quad:  takes off vertically in ANGLE and climbs to the height.
plane: Cessna 172 runway takeoff in ACRO (see plane_runway_takeoff.py), climbs
       and levels off near the height.

Once it reports "ready", connect a GCS and send it somewhere; until then it
holds position (the plane loiters). Ctrl-C ends the flight.

Without --gcs, MAVLink is on TCP 5762 and MSP on TCP 5761 for a direct
connection (QGroundControl: add a TCP link to this PC, port 5762). With --gcs,
all MAVLink is relayed over UDP to that address instead (QGroundControl
listens on 14550 by default), and TCP 5762 is taken by the relay.
"""
import argparse
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from inavbench import Bench
from inavbench.gcs import GCSNAV_AUX
from inavbench.mavudp import Relay
from plane_runway_takeoff import RUNWAY_HEADING, takeoff

PLANE_OVERSHOOT_M = 20                   # the plane climbs on this much while levelling off


def status(b, phase):
    t = b.truth()
    modes = ", ".join(sorted(b.active_modes() - {"ARM"})) or "-"
    print(f"{b.elapsed:5.0f}s  {phase:<6} height {t['agl_m']:6.1f} m (FC {b.altitude()[0]:6.1f})  "
          f"N {t['north_m']:+7.1f} E {t['east_m']:+7.1f} m  {t['ias_kts'] * 0.5144:5.1f} m/s  "
          f"heading {t['yaw']:5.1f}  [{modes}]", flush=True)


def quad_airborne(b, alt):
    b.mode("ANGLE")
    b.arm()
    b.sticks(throttle=0.55)              # a steady ~5 m/s climb
    while b.truth()["agl_m"] < alt - 2:  # POSHOLD's altitude hold stops it
        b.run(0.5)
        status(b, "climb")
    b.sticks(throttle=0.5)


def plane_airborne(b, alt):
    takeoff(b, alt - PLANE_OVERSHOOT_M)
    # Settle in straight and level flight before the loiter: turning straight out of the
    # climb, INAV's attitude estimate is still off and the loiter dives into the ground.
    b.mode("ANGLE")
    b.mode("NAV ALTHOLD")
    for _ in range(3):
        b.run(5)
        status(b, "settle")


args = argparse.ArgumentParser(description=__doc__.splitlines()[0])
args.add_argument("vehicle", choices=("quad", "plane"))
args.add_argument("sitl", help="path to SITL.elf")
args.add_argument("--alt", type=float, default=100.0, help="height in m (default 100)")
args.add_argument("--gcs", action="append", default=[], metavar="HOST[:PORT]",
                  help="relay MAVLink over UDP to this GCS (port 14550 if omitted); repeatable")
args = args.parse_args()

if args.vehicle == "quad":
    bench = Bench("quad", sitl=args.sitl, cli=[GCSNAV_AUX], log="gcsnav_quad.csv")
else:
    bench = Bench("plane", sitl=args.sitl, airborne=False, heading_deg=RUNWAY_HEADING,
                  cli=[GCSNAV_AUX], log="gcsnav_plane.csv")

with bench as b:
    stop = threading.Event()
    if args.gcs:
        to = [(h.rsplit(":", 1)[0], int(h.rsplit(":", 1)[1])) if ":" in h else (h, 14550) for h in args.gcs]
        threading.Thread(target=Relay([b.gcs_mavlink_port], to=to).run, args=(stop,), daemon=True).start()
        print("relaying MAVLink over UDP to " + ", ".join(f"{h}:{p}" for h, p in to))

    (quad_airborne if args.vehicle == "quad" else plane_airborne)(b, args.alt)
    b.mode("NAV POSHOLD")                # AUX3 high: NAV POSHOLD and GCS NAV together
    b.run(3)
    modes = b.active_modes()
    if not {"NAV POSHOLD", "GCS NAV"} <= modes:
        raise RuntimeError(f"expected NAV POSHOLD + GCS NAV, the FC has {sorted(modes)}")

    where = "UDP to " + ", ".join(args.gcs) if args.gcs else f"TCP {b.gcs_mavlink_port} (MSP on {b.gcs_msp_port})"
    print(f"--- ready: {args.vehicle} in NAV POSHOLD + GCS NAV at {b.truth()['agl_m']:.0f} m, "
          f"MAVLink sysid 1 on {where}. Ctrl-C to end.", flush=True)
    try:
        while True:
            b.run(5)
            status(b, "hold")
    except KeyboardInterrupt:
        print("--- ending the flight")
    stop.set()
