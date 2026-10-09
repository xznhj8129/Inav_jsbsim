"""The simulated vehicle: a JSBSim model, its INAV config, and how INAV's outputs drive it.

A vehicle is data, not code. Adding one means a JSBSim model name, a CLI profile
in profiles/, and the JSBSim property each INAV motor/servo output drives.
"""
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field

import jsbsim

from .launchers import Catapult, LaunchRail, Rail, launcher_model
from .units import FT

HOME = (36.530440, -83.216383)          # Sneedville, TN
PROFILES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")


GROUND_STRIKE_FT = -0.5 / FT    # below this the airframe is in the ground: only its gear contacts collide


class GroundStrike(RuntimeError):
    """The vehicle is in the ground: past here JSBSim's state is meaningless."""


@dataclass(frozen=True)
class Vehicle:
    model: str                  # JSBSim aircraft name
    profile: str                # INAV CLI file in profiles/, applied to a fresh eeprom
    motors: tuple               # JSBSim property driven by INAV motor output 1, 2, ...
    servos: tuple = ()          # JSBSim property driven by INAV servo output 0, 1, ...; a tuple
                                # drives several, and a leading "-" drives one inverted
    airborne: bool = False      # default start: in the air, or on the ground
    alt_m: float = 0.0          # default start height above ground
    speed_kts: float = 0.0      # default start airspeed
    init: dict = field(default_factory=dict)   # extra JSBSim properties set at start
    known_errors: tuple = ()    # JSBSim error messages this model raises harmlessly
    model_dir: str = ""         # directory with aircraft/<model>/ and engine/ for a model outside
                                # JSBSim's bundled set; empty = bundled
    tailsitter: bool = False    # body x is the nose and the thrust axis; while INAV runs the hover
                                # mixer profile, sensors go in the hover frame (see tailsitter_hover_frame)
    hover_mixer_profile: int = 0  # INAV mixer profile index (0-based) flown in the hover frame
    launcher: "Catapult | LaunchRail | None" = None  # what it launches from by default (tests, scenarios); None = none


VEHICLES = {
    # JSBSim's bundled DJI F450. Its four ESC inputs are the *_ext_nd properties;
    # INAV's quad-X order is rear-right, front-right, rear-left, front-left.
    "quad": Vehicle(
        model="F450",
        profile="quad_f450.cli",
        motors=("fcs/cmdAR_ext_nd", "fcs/cmdFR_ext_nd", "fcs/cmdAL_ext_nd", "fcs/cmdFL_ext_nd"),
        # its file claims a newer format than JSBSim 1.3 reads, and its own onboard
        # sensor systems fail to register; the bench computes its own sensors
        known_errors=("INCOMPATIBLE CFG FILE", "Failed to tie property fcs/"),
    ),
    # JSBSim's bundled Cessna 172. Servo 0/1/2 = aileron/elevator/rudder, as set up in the profile.
    # The rudder servo also steers the nosewheel, like the real pedals; JSBSim's steering
    # turns the opposite way to its rudder, hence the "-".
    "plane": Vehicle(
        model="c172p",
        profile="plane_c172p.cli",
        motors=("fcs/throttle-cmd-norm",),
        servos=("fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm", ("fcs/rudder-cmd-norm", "-fcs/steer-cmd-norm")),
        airborne=True,
        alt_m=450.0,
        speed_kts=70.0,
        init={"fcs/mixture-cmd-norm": 1.0},
    ),
    # ArduPilot's Rascal 110 RC trainer (inavbench/models), electric. Servo 0/1/2 = aileron/elevator/rudder.
    "rascal": Vehicle(
        model="Rascal",
        profile="plane_rascal.cli",
        motors=("fcs/throttle-cmd-norm",),
        servos=("fcs/aileron-cmd-norm", "fcs/elevator-cmd-norm", "fcs/rudder-cmd-norm"),
        airborne=True,
        alt_m=150.0,
        speed_kts=35.0,
        model_dir=MODELS,
        launcher=Catapult(),
    ),
}


def tailsitter_hover_frame(v):
    """Body forward-left-up (x nose) -> the tailsitter's multirotor hover frame, forward-left-up with
    forward = belly, up = nose. INAV's MSP_SIMULATOR writes acc/gyro/mag past its board alignment, so the
    rotation tailsitter_orientation_offset applies to real sensors has to happen here."""
    return (-v[2], v[1], v[0])


class _ErrorsOnly(jsbsim.FGLogger):
    """Drops JSBSim's chatter and the vehicle's known harmless errors; any other
    error still reaches stderr."""

    def __init__(self, known_errors):
        super().__init__()
        self._known = known_errors
        self._keep = False
        self._text = []

    def set_level(self, level):
        self._keep = level >= jsbsim.LogLevel.ERROR
        self._text = []

    def file_location(self, filename, line):
        pass

    def message(self, text):
        if self._keep:
            self._text.append(text)

    def format(self, fmt):
        pass

    def flush(self):
        text = "".join(self._text).strip()
        if self._keep and text and not any(k in text for k in self._known):
            sys.stderr.write(f"JSBSim: {text}\n")
        self._text = []


class Plant:
    """JSBSim flying one Vehicle, converted to and from INAV's HITL units."""

    DT = 0.001

    def __init__(self, vehicle, airborne=None, alt_m=None, speed_kts=None, heading_deg=0.0, home=HOME, launcher=None):
        self.vehicle = vehicle
        self.home = home
        self.hover_frame = vehicle.tailsitter   # the bench keeps this on the FC's active mixer profile
        if launcher:
            airborne, alt_m, speed_kts = True, launcher.height_m, 0.0
        airborne = vehicle.airborne if airborne is None else airborne
        alt_m = (vehicle.alt_m if airborne else 0.0) if alt_m is None else alt_m
        speed_kts = (vehicle.speed_kts if airborne else 0.0) if speed_kts is None else speed_kts

        jsbsim.set_logger(_ErrorsOnly(vehicle.known_errors))
        self.fdm = f = jsbsim.FGFDMExec(None)
        f.set_debug_level(0)
        if launcher:
            self._launcher_dir = tempfile.TemporaryDirectory(prefix="inavbench-launcher-")
            aircraft_path, engine_path = launcher_model(vehicle.model, vehicle.model_dir, self._launcher_dir.name)
            f.set_aircraft_path(aircraft_path)
            f.set_engine_path(engine_path)
        elif vehicle.model_dir:
            f.set_aircraft_path(os.path.join(vehicle.model_dir, "aircraft"))
            f.set_engine_path(os.path.join(vehicle.model_dir, "engine"))
        if not f.load_model(vehicle.model):
            raise RuntimeError(f"JSBSim could not load model {vehicle.model!r} (model_dir {vehicle.model_dir!r})")
        f.set_dt(self.DT)
        f["ic/lat-geod-deg"], f["ic/long-gc-deg"] = home
        f["ic/terrain-elevation-ft"] = 0.0
        f["ic/h-agl-ft"] = alt_m / FT
        f["ic/vc-kts"] = speed_kts
        f["ic/psi-true-deg"] = heading_deg
        if launcher:
            f["ic/theta-deg"] = launcher.pitch_deg
        else:
            f["ic/gamma-deg"] = 0.0
        for prop, value in vehicle.init.items():
            f[prop] = value
        f.run_ic()
        if not airborne:
            f["simulation/do_simple_trim"] = 2      # settle onto the landing gear
        f["propulsion/set-running"] = -1
        self.time = 0.0
        self._steps = 0
        self._target = 0.0
        self.rail = Rail(f, launcher, self.DT) if launcher else None

    @property
    def on_rail(self):
        """On its launcher, not yet off the rail's end."""
        return bool(self.rail) and self.rail.on_rail

    def fire(self):
        """Fire the catapult: from here the vehicle runs along the rail to its launch speed, then flies free."""
        if not self.rail:
            raise RuntimeError("no launcher: start the vehicle on one (launcher=...)")
        self.rail.fire()

    def advance(self, seconds):
        # count whole steps against the accumulated target, so a partial step
        # carries over instead of every call rounding up
        self._target += seconds
        while (self._steps + 1) * self.DT <= self._target:
            if self.rail:
                self.rail.before_step()
            self.fdm.run()
            self._steps += 1
            if self.rail:
                self.rail.after_step()
        self.time = self._steps * self.DT
        agl_ft = self.fdm["position/h-agl-ft"]
        if math.isnan(agl_ft) or agl_ft < GROUND_STRIKE_FT:
            raise GroundStrike(f"the vehicle hit the ground at {self.time:.3f} s (agl {agl_ft * FT:.2f} m)")

    def set_outputs(self, motors_us, servos_us):
        """INAV motor outputs (1000..2000 us) and servo outputs (1000..2000 us, 1500 = centre)."""
        f = self.fdm
        for prop, us in zip(self.vehicle.motors, motors_us):
            f[prop] = min(1.0, max(0.0, (us - 1000) / 1000.0))
        for props, us in zip(self.vehicle.servos, servos_us):
            value = min(1.0, max(-1.0, (us - 1500) / 500.0))
            for prop in (props,) if isinstance(props, str) else props:
                if prop.startswith("-"):
                    f[prop[1:]] = -value
                else:
                    f[prop] = value

    def set_wind(self, north_ms=0.0, east_ms=0.0, down_ms=0.0):
        f = self.fdm
        f["atmosphere/wind-north-fps"] = north_ms / FT
        f["atmosphere/wind-east-fps"] = east_ms / FT
        f["atmosphere/wind-down-fps"] = down_ms / FT

    # --- sensors, in INAV HITL units -------------------------------------------
    #
    # INAV's HITL body frame is forward-left-up (see the RealFlight bridge in
    # target/SITL/sim/realFlight.c), JSBSim's is forward-right-down: y and z flip.

    def acc_mg(self):
        # specific force; JSBSim's is (Nx, Ny, -Nz) g in forward-right-down
        f = self.fdm
        if self.rail and self.rail.held:
            # JSBSim zeroes its accelerations while it holds the vehicle; the accelerometer still feels what
            # holds it up (anything held still reads exactly that)
            return self.static_acc_mg()
        return self._fc_frame((f["accelerations/Nx"] * 1000, -f["accelerations/Ny"] * 1000, f["accelerations/Nz"] * 1000))

    def _fc_frame(self, v):
        return tailsitter_hover_frame(v) if self.hover_frame else v

    def static_acc_mg(self):
        """What the accelerometer reads with the vehicle held still at its current attitude."""
        f = self.fdm
        phi, theta = f["attitude/phi-rad"], f["attitude/theta-rad"]
        return self._fc_frame((math.sin(theta) * 1000, math.sin(phi) * math.cos(theta) * 1000, math.cos(phi) * math.cos(theta) * 1000))

    def gyro_dps16(self):
        f = self.fdm
        p, q, r = (math.degrees(f[a]) * 16 for a in ("velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec"))
        return self._fc_frame((p, -q, -r))

    def baro_pa(self):
        return 101325.0 * (1.0 - 2.25577e-5 * self.fdm["position/h-sl-ft"] * FT) ** 5.25588

    def mag(self):
        # a flat north field (no dip), 16000 units = 800 after INAV's /20, as the
        # RealFlight bridge injects it: north in forward-right-down, then y, z flipped
        f = self.fdm
        phi, theta, psi = f["attitude/phi-rad"], f["attitude/theta-rad"], f["attitude/psi-rad"]
        north = (math.cos(theta) * math.cos(psi),
                 math.sin(phi) * math.sin(theta) * math.cos(psi) - math.cos(phi) * math.sin(psi),
                 math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi))
        return self._fc_frame((16000 * north[0], -16000 * north[1], -16000 * north[2]))

    def gps(self, fix=True):
        f = self.fdm
        vn, ve, vd = (f[p] * FT for p in ("velocities/v-north-fps", "velocities/v-east-fps", "velocities/v-down-fps"))
        return {
            "fix": 2 if fix else 0,             # 2 = 3D fix, 0 = no fix
            "sats": 14 if fix else 0,
            "lat_e7": int(round(f["position/lat-geod-deg"] * 1e7)),
            "lon_e7": int(round(f["position/long-gc-deg"] * 1e7)),
            "alt_cm": int(round(f["position/h-sl-ft"] * FT * 100)),
            "speed_cms": math.hypot(vn, ve) * 100,
            "course_dd": math.degrees(math.atan2(ve, vn)) % 360.0 * 10,
            "vel_ned_cms": (vn * 100, ve * 100, vd * 100),
        }

    def airspeed_cms(self):
        return self.fdm["velocities/vc-kts"] * 51.4444

    def rangefinder_cm(self, max_m=10.0):
        """Distance along the body's down axis to flat ground, or -1 when out of range."""
        f = self.fdm
        if self.hover_frame:            # hover-frame down is the body's tail direction
            tilt = math.sin(f["attitude/theta-rad"])
        else:
            tilt = math.cos(f["attitude/phi-rad"]) * math.cos(f["attitude/theta-rad"])
        if tilt < 0.5:
            return -1
        d = f["position/h-agl-ft"] * FT / tilt
        return d * 100 if d <= max_m else -1

    # --- ground truth ------------------------------------------------------------

    def truth(self):
        f = self.fdm
        lat, lon = f["position/lat-geod-deg"], f["position/long-gc-deg"]
        return {
            "lat": lat,
            "lon": lon,
            "north_m": (lat - self.home[0]) * 111320.0,
            "east_m": (lon - self.home[1]) * 111320.0 * math.cos(math.radians(self.home[0])),
            "agl_m": f["position/h-agl-ft"] * FT,
            "roll": f["attitude/phi-deg"],
            "pitch": f["attitude/theta-deg"],
            "yaw": f["attitude/psi-deg"] % 360.0,
            "vn": f["velocities/v-north-fps"] * FT,
            "ve": f["velocities/v-east-fps"] * FT,
            "vd": f["velocities/v-down-fps"] * FT,
            "ias_kts": f["velocities/vc-kts"],
        }
