#!/usr/bin/env python3
"""Example: four quads fly at once, each its own SITL, watched over UDP MAVLink.

    python3 examples/quad_swarm.py /path/to/inav/build_sitl/bin/SITL.elf [count] [--gcs host[:port]]

Quad n (1..count, default 4) gets MAVLink sysid n, its own port block (5760,
5770, ...) and a launch spot 5 m east of the previous one. They climb to different heights, fly
out north / east / south / west, then RTH and land where they took off.

All four MAVLink streams are relayed to UDP 14550, so a QGroundControl running
on this machine shows the whole flight with no setup; --gcs also sends it to another
machine, e.g. a phone running QGroundControl. Each quad is also sent
alone to UDP 14560+n, where a MAVSDK System watches it; the table printed here
is built only from MAVSDK telemetry, not from the bench's ground truth.

Needs mavsdk (the `drone` conda env has it).
"""
import argparse
import asyncio
import math
import multiprocessing
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mavsdk import System

from inavbench import HOME, Bench
from inavbench.mavudp import Relay

_args = argparse.ArgumentParser(description="Several quads at once over UDP MAVLink")
_args.add_argument("sitl", help="path to SITL.elf")
_args.add_argument("count", nargs="?", type=int, default=4, help="number of quads (default 4)")
_args.add_argument("--gcs", action="append", default=[], metavar="HOST[:PORT]",
                   help="also send all MAVLink to this GCS (port 14550 if omitted); repeatable")
ARGS = _args.parse_args()
COUNT = ARGS.count
SPACING_M = 5.0
QGC = ("127.0.0.1", 14550)
EACH = 14560            # quad n alone on udp 14560+n, for its MAVSDK System
LEGS = [dict(pitch=0.5), dict(roll=0.5), dict(pitch=-0.5), dict(roll=-0.5)]   # N, E, S, W


def launch_spot(i):
    return HOME[0], HOME[1] + i * SPACING_M / (111320.0 * math.cos(math.radians(HOME[0])))


def fly(i, sitl, results):
    sys.stdout = open(os.devnull, "w")           # the parent does the talking
    with Bench("quad", sitl=sitl, port=5760 + 10 * i, home=launch_spot(i),
               cli=[f"set mavlink_sysid = {i + 1}"]) as b:
        b.mode("ANGLE")
        b.arm()
        b.sticks(throttle=0.62)
        b.run(3 + i % 4)                         # staggered climb: different heights
        b.sticks(throttle=0.5)
        b.mode("NAV POSHOLD")
        b.run(3)
        b.sticks(**LEGS[i % 4])
        b.run(5)
        b.sticks(roll=0, pitch=0)
        b.run(4)
        b.mode("NAV RTH")
        t0, ticks0 = time.monotonic(), b.ticks
        b.run(150, until=lambda x: not x.fc.armed, check_every=0.5)
        rate = (b.ticks - ticks0) / (time.monotonic() - t0)
        t = b.truth()
        results.put((i + 1, not b.fc.armed, math.hypot(t["north_m"], t["east_m"]), rate))


async def watch(sysid, state):
    s = state.setdefault(sysid, {})
    drone = System(port=50050 + sysid)           # each System runs its own mavsdk_server
    await drone.connect(system_address=f"udpin://0.0.0.0:{EACH + sysid}")

    async def connection():
        async for c in drone.core.connection_state():
            s["connected"] = c.is_connected

    async def armed():
        async for a in drone.telemetry.armed():
            s["armed"] = a

    async def position():
        async for p in drone.telemetry.position():
            s["north"] = (p.latitude_deg - HOME[0]) * 111320.0
            s["east"] = (p.longitude_deg - HOME[1]) * 111320.0 * math.cos(math.radians(HOME[0]))
            s["alt"] = p.relative_altitude_m

    await asyncio.gather(connection(), armed(), position())


def monitor(state):
    async def watch_all():
        await asyncio.gather(*(watch(n, state) for n in range(1, COUNT + 1)))
    asyncio.run(watch_all())


def main():
    sitl = ARGS.sitl
    gcs = [(h.rsplit(":", 1)[0], int(h.rsplit(":", 1)[1])) if ":" in h else (h, 14550) for h in ARGS.gcs]
    results = multiprocessing.Queue()
    pilots = [multiprocessing.Process(target=fly, args=(i, sitl, results)) for i in range(COUNT)]
    for p in pilots:
        p.start()

    stop = threading.Event()
    state = {}
    relay = Relay([5762 + 10 * i for i in range(COUNT)], to=[QGC, *gcs], each=EACH)
    threading.Thread(target=relay.run, args=(stop,), daemon=True).start()
    threading.Thread(target=monitor, args=(state,), daemon=True).start()
    print(f"relaying {COUNT} vehicles to udp " + ", ".join(f"{h}:{p}" for h, p in [QGC, *gcs]))

    t0 = time.monotonic()
    while any(p.is_alive() for p in pilots):
        time.sleep(2)
        now = time.monotonic()
        cells = []
        for sysid in range(1, COUNT + 1):
            s = state.get(sysid)
            if not s or not s.get("connected"):
                cells.append(f"#{sysid} ---")
            elif "alt" in s and COUNT > 4:
                cells.append(f"#{sysid} {s['alt']:3.0f}m{'' if s.get('armed') else ' dis'}")
            elif "alt" in s:
                cells.append(f"#{sysid} {'ARM' if s.get('armed') else 'dis'} N{s['north']:+6.1f} E{s['east']:+6.1f} "
                             f"alt{s['alt']:5.1f}")
            else:
                cells.append(f"#{sysid} {'ARM' if s.get('armed') else 'dis'}")
        print(f"{now - t0:4.0f}s  " + ("  " if COUNT > 4 else "  |  ").join(cells), flush=True)
    stop.set()

    print()
    ok = True
    while not results.empty():
        sysid, landed, dist, rate = results.get()
        print(f"quad #{sysid}: {'landed and disarmed' if landed else 'STILL ARMED'}, "
              f"{dist:.1f} m from its launch spot, sim loop {rate:.0f} ticks/s during RTH")
        ok &= landed
    ok &= all(p.exitcode == 0 for p in pilots)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
