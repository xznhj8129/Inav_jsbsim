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

Vehicles: `quad` (JSBSim F450), `plane` (JSBSim Cessna 172, airborne start), `rascal` (ArduPilot's Rascal 110 RC trainer in `inavbench/models`, launched from its catapult in NAV LAUNCH). `Bench(catapult=Catapult(...))` starts any vehicle on a perfect launcher and `launch()` fires it.

`--stepped` flies on SITL's stepped clock (`Bench(stepped=True)`, needs an INAV SITL with the stepped clock), with SITL's `--faster` (`Bench(faster=True)`: about twice the frame rate, one CPU core held; single runs only); `--rate=HZ` sets the tick rate.

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

`Bench(..., log="run.csv")` writes a flight log (FC attitude vs truth, RC, outputs). `log_fc=True` adds what INAV reports over MSP each log row (battery, altitude estimate, GPS, navigation state, throttle, load, every flight mode); `log_extra=fn` adds your own columns; `battery=fn` feeds INAV's battery voltage and current meters every tick (the config needs `feature VBAT` and `feature CURRENT_METER`; at most 25.5 V).

## Watch from a GCS

While a script runs, connect QGroundControl (TCP, `127.0.0.1:5762`, MAVLink) or INAV Configurator (TCP, `127.0.0.1:5761`, MSP). The bench prints both when it starts.

For UDP (QGroundControl's default 14550) or several vehicles at once: `python3 -m inavbench.mavudp 5762 5772 ...` relays each vehicle's MAVLink TCP port to UDP 14550.

## Layout

- `inavbench/` — the bench: `bench.py` (SITL, RC, modes, loop, FC queries), `vehicles.py` (JSBSim plant and vehicle definitions; a `Vehicle` can load a model from outside JSBSim's bundled set with `model_dir`, and `tailsitter=True` hands INAV its sensors in the multirotor hover frame, since MSP_SIMULATOR bypasses INAV's tailsitter alignment), `hitl.py` (MSP_SIMULATOR frames), `msp.py` (MSP + CLI), `mavudp.py` (MAVLink TCP-to-UDP relay), `selftest.py`
- `inavbench/profiles/` — INAV CLI config per vehicle, applied to a fresh eeprom on every start
- `inavbench/gcs.py` — GCS NAV: move a vehicle's POSHOLD target (waypoint #255) from outside, or place it so a quad flies a given velocity; `inavbench/metar.py` — live METAR wind
- `examples/` — run with the SITL path as the argument:
    - `quad_flight.py`: takeoff, POSHOLD, RTH landing
    - `plane_runway_takeoff.py`: Cessna runway takeoff to NAV ALTHOLD
    - `gcsnav.py`: a quad or the plane airborne and held in NAV POSHOLD + GCS NAV, for a GCS to fly
    - `quad_swarm.py`: four quads at once over UDP MAVLink
    - `swarm_flock.py`: a flock steered by an external swarm brain (`swarming2.py` in its copter profile) over GCS NAV, with `swarm_replay.py` for a 3D replay page
- `old/` — superseded scripts from before this bench, kept for reference
