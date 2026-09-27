# inav-jsbsim-bench

Headless test bench for INAV: the real firmware, built as SITL, flying a quad or a plane on JSBSim physics, driven from Python. No X-Plane, no GUI, no containers.

Full guide (how it works, API, gotchas): AI_SHARED `projects/software/inav-development/jsbsim-sitl-bench.md`.

## Needs

- JSBSim 1.3+; the `drone` conda env has it, plus mavsdk for `quad_swarm.py`
- the bench itself, installed from this checkout: `pip install -e .` (the `drone` env has it)
- a SITL build of whatever INAV you're testing: `build_sitl/bin/SITL.elf`

## Check it works

```bash
python3 -m inavbench.selftest /path/to/inav/build_sitl/bin/SITL.elf
```

Flies the quad (takeoff, POSHOLD, RTH to landing) and the plane (banks, RTH) and prints PASS/FAIL. Run it whenever you switch firmware.

## Use it

```python
from inavbench import Bench

with Bench("quad", sitl="/path/to/SITL.elf") as b:
    b.mode("ANGLE")
    b.arm()
    b.sticks(throttle=0.6)
    b.run(4)
    b.mode("NAV POSHOLD")
    b.run(10)
    print(b.truth(), b.fc, b.active_modes())
```

## Watch from a GCS

While a script runs, connect QGroundControl (TCP, `127.0.0.1:5762`, MAVLink) or INAV Configurator (TCP, `127.0.0.1:5761`, MSP). The bench prints both when it starts.

For UDP (QGroundControl's default 14550) or several vehicles at once: `python3 -m inavbench.mavudp 5762 5772 ...` relays each vehicle's MAVLink TCP port to UDP 14550.

## Layout

- `inavbench/` — the bench: `bench.py` (SITL, RC, modes, loop, FC queries), `vehicles.py` (JSBSim plant and vehicle definitions), `hitl.py` (MSP_SIMULATOR frames), `msp.py` (MSP + CLI), `mavudp.py` (MAVLink TCP-to-UDP relay), `selftest.py`
- `inavbench/profiles/` — INAV CLI config per vehicle, applied to a fresh eeprom on every start
- `inavbench/gcs.py` — GCS NAV: move a vehicle's POSHOLD target (waypoint #255) from outside; `inavbench/metar.py` — live METAR wind
- `examples/` — run with the SITL path as the argument:
    - `quad_flight.py`: takeoff, POSHOLD, RTH landing
    - `plane_runway_takeoff.py`: Cessna runway takeoff to NAV ALTHOLD
    - `gcsnav.py`: a quad or the plane airborne and held in NAV POSHOLD + GCS NAV, for a GCS to fly
    - `quad_swarm.py`: four quads at once over UDP MAVLink
    - `swarm_flock.py`: a flock steered by an external swarm brain over GCS NAV, with `swarm_replay.py` for a 3D replay page
- `old/` — superseded scripts from before this bench, kept for reference

## Built on it

- OPERATION CLINCH (its own repo, `clinch`): a system-of-systems exercise with six quads and the Cessna, a lossy mesh, a command post, and CoT to TAK.
