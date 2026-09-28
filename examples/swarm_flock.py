#!/usr/bin/env python3
"""A flock of INAV quads, each steered by an external swarm "brain" over GCS NAV.

    python3 examples/swarm_flock.py /path/to/SITL.elf /path/to/swarming2.py [--count 6] [--minutes 3] [--formation VEE]

The chain, end to end:

  live METAR (KMOR, nearest to Sneedville)  ->  wind in every JSBSim sim
  N real INAV SITLs flying JSBSim F450s     ->  take off, hold in POSHOLD + GCS NAV
  each drone's companion "brain"            ->  swarming2.Agent, configured for copters, flocking
                                                or holding a --formation
  a simulated lossy radio (UDP hub)         ->  2 Hz beacons (the brain's own, carried as-is), range
                                                limit, packet loss: the only way a brain learns where
                                                the others are
  brain velocity -> MSP_SET_WP #255         ->  placed where INAV's position P flies that velocity;
                                                the nose follows the brain's slow-turning drive
  all MAVLink -> UDP 14550                  ->  QGroundControl shows the flock live
  end: RTH and land, then a replay file     ->  swarm_replay.json (see swarm_replay.py)

The brain module is passed in by path and switched to its copter profile.
Needs pygame (the brain uses its Vector3) and network access for the METAR.
"""
import argparse
import importlib.util
import json
import math
import multiprocessing
import os
import random
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from inavbench import HOME, Bench
from inavbench.gcs import GCSNAV_AUX, M_PER_DEG_LON, gcs_addresses, pos_xy_gain, set_target, set_velocity
from inavbench.mavudp import Relay
from inavbench.metar import fetch_wind

LAUNCH_SPACING_M = 6.0            # launch pads in a line, west to east
TAKEOFF_ALT_M = 30.0
BRAIN_HZ = 5.0
BEACON_HZ = 2.0
RADIO_RANGE_M = 250.0
RADIO_LOSS = 0.15                 # fraction of beacons that never arrive
HUB_PORT = 47000
QGC = ("127.0.0.1", 14550)


def load_brain(path):
    spec = importlib.util.spec_from_file_location("swarm_brain", path)
    brain = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(brain)
    brain.configure("copter")
    brain.FORMATION, brain.FORMATION_SPACING = ARGS.formation, ARGS.spacing
    return brain


def pad(i):
    """Launch pad of quad i: (north, east) metres from HOME, and its lat/lon."""
    east = (i - (ARGS.count - 1) / 2.0) * LAUNCH_SPACING_M
    return (0.0, east), (HOME[0], HOME[1] + east / M_PER_DEG_LON)


# --- one quad: SITL + JSBSim + its brain ---------------------------------------------

def fly(i, route_points, wind, ready, go, results):
    sys.stdout = open(os.devnull, "w")
    brain = load_brain(ARGS.brain)
    V = brain.Vector3
    route = brain.Route.__new__(brain.Route)
    route.points = [V(p) for p in route_points]
    sysid = i + 1
    (pad_n, pad_e), home = pad(i)

    radio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    radio.bind(("127.0.0.1", HUB_PORT + sysid))
    radio.setblocking(False)

    # INAV must not cap or soften the brain's velocity: speed limit at its V_MAX, no position expo
    speed = round(brain.V_MAX * 100)
    with Bench("quad", sitl=ARGS.sitl, port=5760 + 10 * i, home=home,
               cli=[f"set mavlink_sysid = {sysid}", GCSNAV_AUX, f"set nav_auto_speed = {speed}",
                    f"set nav_max_auto_speed = {speed}", "set nav_mc_pos_expo = 0"]) as b:
        b.wind(*wind)
        gain = pos_xy_gain(b)

        def pose():
            # the FC's own fused estimate: local NEU from its own origin (its pad), cm and cm/s
            raw = b.msp.request(0x2220)
            v = struct.unpack_from("<ihihih", raw, 6)
            return (V(pad_e + v[2] / 100.0, pad_n + v[0] / 100.0, v[4] / 100.0),
                    V(v[3] / 100.0, v[1] / 100.0, v[5] / 100.0))

        b.mode("ANGLE")
        b.arm()
        b.sticks(throttle=0.55)
        b.run(1)
        b.sticks(throttle=0.5)
        b.mode("NAV POSHOLD")                   # with GCS NAV: from here on #255 sets position and height
        climb = V(pad_e, pad_n, TAKEOFF_ALT_M + 3 * (i % 3))
        while b.truth()["agl_m"] < climb.z - 1:
            set_target(b, climb.x, climb.y, climb.z, 0)
            b.run(0.5)
        ready.put(sysid)
        while not go.is_set():                  # keep flying (holding) until the whole flock is up
            b.run(0.5)

        pos, vel = pose()
        me = brain.Agent(sysid, pos, vel)
        me.roster = list(range(1, ARGS.count + 1))
        log = []
        next_beacon = 0.0
        last = time.monotonic()
        end = last + ARGS.minutes * 60
        while time.monotonic() < end:
            b.run(1.0 / BRAIN_HZ)
            now = time.monotonic()
            dt, last = now - last, now
            me.pos, me.vel = pose()

            if now >= next_beacon:
                next_beacon = now + 1.0 / BEACON_HZ
                radio.sendto(json.dumps(me.beacon(now)).encode(), ("127.0.0.1", HUB_PORT))
            while True:
                try:
                    me.hear(json.loads(radio.recv(4096)))   # beacon times are time.monotonic(): one machine
                except BlockingIOError:
                    break
            peers = me.peers(now)

            me.refresh_neighbors(peers)
            cmd = me.compute_velocity(me.active_neighbors({p.id: p for p in peers}), route, dt)
            alt = me.pos.z + cmd.z / brain.ALT_GAIN     # the route altitude, pushed by vertical repulsion
            heading = math.degrees(math.atan2(me.drive_dir.x, me.drive_dir.y))
            target = set_velocity(b, me.pos.x, me.pos.y, cmd.x, cmd.y, alt, heading, gain, 1.0 / BRAIN_HZ)

            t = b.truth()
            log.append({"t": round(now, 2), "seg": me.segment_index, "n": sorted(p.id for p in peers),
                        "truth": [round(t["east_m"] + pad_e, 2), round(t["north_m"] + pad_n, 2), round(t["agl_m"], 2)],
                        "fc": [round(c, 2) for c in me.pos], "vel": [round(c, 2) for c in me.vel],
                        "cmd": [round(c, 2) for c in cmd], "heading": round(heading, 1),
                        "target": [round(c, 2) for c in target],
                        "slot": me.slot, "emergency": round(me.emergency_level, 2),
                        "slot_target": None if me.slot_target is None else [round(c, 2) for c in me.slot_target]})

        b.mode("NAV RTH")
        landed = b.run(150, until=lambda x: not x.fc.armed, check_every=0.5)
        results.put((sysid, landed, log))


# --- the radio: range-limited, lossy beacon relay -----------------------------------------

def radio_hub(stop, links, last):
    hub = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hub.bind(("127.0.0.1", HUB_PORT))
    hub.settimeout(0.2)
    while not stop.is_set():
        try:
            data = hub.recv(4096)
        except socket.timeout:
            continue
        msg = json.loads(data)
        last[msg["id"]] = msg["p"]
        for other, p in last.items():
            if other == msg["id"] or math.dist(p, msg["p"]) > RADIO_RANGE_M:
                continue
            if random.random() < RADIO_LOSS:
                continue
            hub.sendto(data, ("127.0.0.1", HUB_PORT + other))
            links.append((round(time.monotonic(), 2), msg["id"], other))


# --- conductor -------------------------------------------------------------------------------

def main():
    brain = load_brain(ARGS.brain)
    random.seed(ARGS.seed)
    route = [list(p) for p in brain.Route().points]
    wind, metar = fetch_wind()
    print(f"METAR {metar}\nwind over Sneedville: {math.hypot(*wind):.1f} m/s toward "
          f"{math.degrees(math.atan2(wind[1], wind[0])) % 360:.0f} deg")
    print(f"route: {len(route)} waypoints, {ARGS.count} quads, {ARGS.minutes} min of flocking")

    go = multiprocessing.Event()
    ready = multiprocessing.Queue()
    results = multiprocessing.Queue()
    quads = [multiprocessing.Process(target=fly, args=(i, route, wind, ready, go, results)) for i in range(ARGS.count)]
    for q in quads:
        q.start()

    stop = threading.Event()
    links = []
    last = {}                                   # sysid -> last beaconed position, from the hub
    threading.Thread(target=radio_hub, args=(stop, links, last), daemon=True).start()
    relay = Relay([5762 + 10 * i for i in range(ARGS.count)], to=[QGC, *ARGS.gcs_to])
    threading.Thread(target=relay.run, args=(stop,), daemon=True).start()
    print(f"MAVLink for all {ARGS.count} quads on udp {QGC[0]}:{QGC[1]}")

    print("taking off...", flush=True)
    for _ in range(ARGS.count):
        print(f"  quad #{ready.get(timeout=120)} up", flush=True)
    go.set()
    t0 = time.monotonic()
    print("flocking", flush=True)

    logs = {}
    next_status = time.monotonic()
    while len(logs) < ARGS.count:
        flocking = time.monotonic() - t0 < ARGS.minutes * 60
        if flocking and time.monotonic() >= next_status and len(last) > 1:
            next_status += 10
            pts = list(last.values())
            c = [sum(p[k] for p in pts) / len(pts) for k in range(3)]
            spread = max(math.dist(p, c) for p in pts)
            closest = min(math.dist(a, b) for n, a in enumerate(pts) for b in pts[n + 1:])
            recent = sum(1 for l in links[-400:] if l[0] > time.monotonic() - 10) / 10
            print(f"{time.monotonic() - t0:5.0f}s  flock centre E{c[0]:+6.0f} N{c[1]:+6.0f} m, {c[2]:4.0f} m up | "
                  f"spread {spread:4.0f} m, closest pair {closest:4.1f} m | radio {recent:4.1f} msgs/s", flush=True)
        while not results.empty():
            sysid, landed, log = results.get()
            logs[sysid] = {"landed": landed, "log": log}
            print(f"quad #{sysid}: {'landed' if landed else 'NOT landed'}, {len(log)} brain steps", flush=True)
        if not any(q.is_alive() for q in quads) and results.empty():
            break
        time.sleep(1)
    stop.set()

    replay = {"metar": metar, "wind": wind, "route": route, "home": HOME, "t0": t0,
              "radio_range_m": RADIO_RANGE_M, "radio_loss": RADIO_LOSS,
              "quads": logs, "links": [l for l in links if l[0] >= t0]}
    with open("swarm_replay.json", "w") as f:
        json.dump(replay, f)
    print("replay: swarm_replay.json")
    sys.exit(0 if all(q["landed"] for q in logs.values()) and len(logs) == ARGS.count else 1)


if __name__ == "__main__":
    args = argparse.ArgumentParser(description="INAV quads flocking under an external swarm brain")
    args.add_argument("sitl")
    args.add_argument("brain", help="path to swarming2.py")
    args.add_argument("--count", type=int, default=6)
    args.add_argument("--minutes", type=float, default=3.0)
    args.add_argument("--seed", type=int, default=7)
    args.add_argument("--formation", default="NONE", help="a swarming2 FORMATIONS name; NONE flocks")
    args.add_argument("--spacing", type=float, default=10.0, help="formation slot spacing, m")
    args.add_argument("--gcs", action="append", default=[], metavar="HOST[:PORT]", help="also relay MAVLink here")
    ARGS = args.parse_args()
    ARGS.gcs_to = gcs_addresses(ARGS.gcs)
    main()
