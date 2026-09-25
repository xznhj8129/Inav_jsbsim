"""Headless INAV SITL test bench: real firmware, JSBSim physics, driven from Python.

    with Bench("quad", sitl="/path/to/inav/build_sitl/bin/SITL.elf") as b:
        b.arm()
        b.sticks(throttle=0.6)
        b.run(3)
        b.mode("NAV POSHOLD")
        b.run(10)
        print(b.truth(), b.fc)
"""
import csv
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time

from . import hitl
from .msp import Msp
from .vehicles import HOME, PROFILES, VEHICLES, Plant

MSP_SERVO = 103
MSP_MOTOR = 104
MSP_SONAR_ALTITUDE = 58
MSP_ALTITUDE = 109
MSP_ACTIVEBOXES = 113
MSP_BOXIDS = 119
MSP_NAV_STATUS = 121
MSP_MODE_RANGES = 34
MSP2_INAV_STATUS = 0x2000
MSP2_INAV_AIR_SPEED = 0x2009
_MOTORS = struct.Struct("<8H")

ARMING_BLOCKERS = {
    6: "GEOZONE", 7: "FAILSAFE_SYSTEM", 8: "NOT_LEVEL", 9: "SENSORS_CALIBRATING",
    10: "SYSTEM_OVERLOADED", 11: "NAVIGATION_UNSAFE", 12: "COMPASS_NOT_CALIBRATED",
    13: "ACCELEROMETER_NOT_CALIBRATED", 14: "ARM_SWITCH", 15: "HARDWARE_FAILURE",
    16: "BOXFAILSAFE", 17: "TELEM_LINK", 18: "RC_LINK", 19: "THROTTLE", 20: "CLI",
    21: "CMS_MENU", 22: "OSD_MENU", 23: "ROLLPITCH_NOT_CENTERED", 24: "SERVO_AUTOTRIM",
    25: "OOM", 26: "INVALID_SETTING", 27: "PWM_OUTPUT_ERROR", 28: "NO_PREARM",
    29: "DSHOT_BEEPER", 30: "LANDING_DETECTED",
}
SENSORS_CALIBRATING = 1 << 9

# INAV's permanent mode ids (fc_msp_box.c). MSP_BOXNAMES would give the same,
# but INAV refuses it when the list outgrows the reply buffer (airplanes do).
BOX_NAMES = {
    0: "ARM",
    1: "ANGLE",
    2: "HORIZON",
    3: "NAV ALTHOLD",
    5: "HEADING HOLD",
    6: "HEADFREE",
    7: "HEADADJ",
    8: "CAMSTAB",
    10: "NAV RTH",
    11: "NAV POSHOLD",
    12: "MANUAL",
    13: "BEEPER",
    15: "LEDS OFF",
    16: "LIGHTS",
    19: "OSD OFF",
    20: "TELEMETRY",
    21: "AUTO TUNE",
    26: "BLACKBOX",
    27: "FAILSAFE",
    28: "NAV WP",
    29: "AIR MODE",
    30: "HOME RESET",
    31: "GCS NAV",
    32: "FPV ANGLE MIX",
    33: "SURFACE",
    34: "FLAPERON",
    35: "TURN ASSIST",
    36: "NAV LAUNCH",
    37: "SERVO AUTOTRIM",
    39: "CAMERA CONTROL 1",
    40: "CAMERA CONTROL 2",
    41: "CAMERA CONTROL 3",
    42: "OSD ALT 1",
    43: "OSD ALT 2",
    44: "OSD ALT 3",
    45: "NAV COURSE HOLD",
    46: "MC BRAKING",
    47: "USER1",
    48: "USER2",
    57: "USER3",
    58: "USER4",
    49: "LOITER CHANGE",
    50: "CONTROL OVERRIDE",
    51: "PREARM",
    52: "TURTLE",
    53: "NAV CRUISE",
    54: "AUTO LEVEL TRIM",
    55: "WP PLANNER",
    56: "SOARING",
    59: "MISSION CHANGE",
    60: "BEEPER MUTE",
    61: "MULTI FUNCTION",
    62: "MIXER PROFILE 2",
    63: "MIXER TRANSITION",
    64: "ANGLE HOLD",
    65: "GIMBAL LEVEL TILT",
    66: "GIMBAL LEVEL ROLL",
    67: "GIMBAL CENTER",
    68: "GIMBAL HEADTRACKER",
    69: "AUTO SPEED",
    70: "TERRAIN AGL HOLD",
    71: "IN FLIGHT MENU",
    72: "THRUST REVERSE",
}

RATE_HZ = 400        # sim-loop ticks per second: flies like uncapped for ~2/3 of the bench CPU.
                     # At 250 the quad's pitch goes into a growing +-11 deg oscillation in POSHOLD.
GPS_HZ = 10          # rate of new GPS solutions sent to the FC
LOG_HZ = 50
SETTLE_S = 3.0       # streamed after start so HITL, baro calibration and the AHRS settle


class ArmingError(RuntimeError):
    pass


class Bench:
    def __init__(self, vehicle, sitl, *, port=5760, workdir=None, cli=(), airborne=None, alt_m=None,
                 speed_kts=None, heading_deg=0.0, home=HOME, log=None, pitot=False, rangefinder=False,
                 rate_hz=RATE_HZ):
        """vehicle: a key of VEHICLES ("quad", "plane") or a Vehicle.
        port: SITL's first TCP port; it takes one per UART, so space parallel benches 10 apart.
        cli: extra INAV CLI lines applied after the vehicle profile, e.g. your own diff.
        home: (lat, lon) start position; truth() north/east are measured from it.
        rate_hz: sim-loop ticks per second; None runs as fast as SITL answers (~850/s, twice the CPU).
        log: CSV path for a per-run flight log (FC vs truth, RC, outputs)."""
        self.vehicle = VEHICLES[vehicle] if isinstance(vehicle, str) else vehicle
        self.sitl = os.path.abspath(sitl)
        self.port = port
        self.gcs_msp_port = port + 1
        self.gcs_mavlink_port = port + 2
        self.extra_cli = list(cli)
        self.airborne = self.vehicle.airborne if airborne is None else airborne
        self.start_state = dict(airborne=self.airborne, alt_m=alt_m, speed_kts=speed_kts, heading_deg=heading_deg,
                                home=home)
        self.use_pitot = pitot
        self.use_rangefinder = rangefinder
        self.log_path = log
        self._period = 1.0 / rate_hz if rate_hz else 0.0
        self._own_workdir = workdir is None
        self.workdir = workdir or tempfile.mkdtemp(prefix="inavbench-")

        # AETR + AUX1..4, the 8 channels MSP_SIMULATOR carries
        self.rc = [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000]
        self.rc_link = True
        self.gps_fix = True
        self.fc = None          # last hitl.Reply: armed + the FC's attitude estimate
        self.ticks = 0          # sim-loop ticks run so far
        self.motors = ()
        self.servos = ()
        self.hold = False       # True pins the vehicle in place (no physics, pure gravity)
        self.plant = None
        self.msp = None
        self._proc = None

    # --- lifecycle ------------------------------------------------------------------

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, *_):
        # keep the SITL log and eeprom when something went wrong
        self.stop(keep_workdir=exc_type is not None)

    def start(self):
        eeprom = os.path.join(self.workdir, "eeprom.bin")
        if os.path.exists(eeprom):
            os.remove(eeprom)
        self._sitl_log = open(os.path.join(self.workdir, "sitl.log"), "w")
        self._proc = subprocess.Popen(
            [self.sitl, f"--path={eeprom}", f"--tcpbaseport={self.port}"],
            cwd=self.workdir, stdout=self._sitl_log, stderr=subprocess.STDOUT, start_new_session=True)

        msp = self._connect()
        msp.cli(self._profile_lines())
        msp.cli_save()          # reboots the SITL into the new config
        self.msp = self._connect()
        print(f"GCS: MSP on tcp://127.0.0.1:{self.gcs_msp_port}, MAVLink on tcp://127.0.0.1:{self.gcs_mavlink_port}")

        self._read_modes()
        self._wait_boot_calibration()
        self.plant = Plant(self.vehicle, **self.start_state)
        self.hold = self.airborne
        self._t_last = time.monotonic()
        self._next_gps = 0.0
        self._t0 = self._t_last
        self._next_tick = self._t_last
        self._open_log()
        self.run(SETTLE_S)

    def stop(self, keep_workdir=False):
        if self.msp:
            self.msp.close()
            self.msp = None
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            self._proc.wait(timeout=5)
        if getattr(self, "_log_file", None):
            self._log_file.close()
            self._log_file = None
        if getattr(self, "_sitl_log", None):
            self._sitl_log.close()
        if keep_workdir:
            sys.stderr.write(f"bench workdir kept: {self.workdir}\n")
        elif self._own_workdir:
            shutil.rmtree(self.workdir, ignore_errors=True)

    def _profile_lines(self):
        with open(os.path.join(PROFILES, self.vehicle.profile)) as f:
            lines = [ln.split("#", 1)[0].strip() for ln in f]
        # SITL maps UART n to TCP port base + n - 1. UART1 is the bench's own link;
        # UART2 and UART3 are left for a GCS / configurator to connect to
        lines += ["serial 1 1 115200 115200 0 115200",        # UART2: MSP
                  "serial 2 256 115200 115200 115200 115200",  # UART3: MAVLink
                  "feature TELEMETRY",                         # MAVLink is only sent with it on
                  "set mavlink_autopilot_type = ARDUPILOT"]    # what GCSs (QGC) handle properly
        # INAV only takes HITL airspeed / rangefinder data into a configured sensor
        if self.use_pitot:
            lines.append("set pitot_hardware = FAKE")
        if self.use_rangefinder:
            lines.append("set rangefinder_hardware = FAKE")
        return [ln for ln in lines if ln] + self.extra_cli

    def _connect(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while True:
            if self._proc.poll() is not None:
                raise RuntimeError(f"SITL exited with code {self._proc.returncode}; log: {self._sitl_log.name}")
            try:
                return Msp(self.port)
            except ConnectionRefusedError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.1)

    def _wait_boot_calibration(self, timeout=20.0):
        # The gyro calibration never finishes if HITL frames start before it does
        # (gyroUpdate() returns early under HITL), so wait for it on the fake IMU first.
        # Right after boot the flags read 0 because they have not been evaluated yet;
        # once they are, RC_LINK at least is always set (no RC before HITL starts).
        deadline = time.monotonic() + timeout
        while True:
            flags = self.arming_flags()
            if flags and not flags & SENSORS_CALIBRATING:
                return
            if time.monotonic() > deadline:
                raise RuntimeError(f"boot sensor calibration did not finish: {self.arming_blockers()}")
            time.sleep(0.1)

    # --- the loop -------------------------------------------------------------------

    def run(self, seconds, until=None, check_every=0.05):
        """Fly for `seconds` of wall time. `until(bench)` ends it early by returning True;
        it is checked every `check_every` s. Returns True if `until` fired."""
        end = time.monotonic() + seconds
        next_check = 0.0
        while time.monotonic() < end:
            self._tick()
            if until is not None and time.monotonic() >= next_check:
                next_check = time.monotonic() + check_every
                if until(self):
                    return True
        return False

    def _tick(self):
        self.ticks += 1
        now = time.monotonic()
        p = self.plant
        if not self.hold:
            # advance by the full wall-clock gap: the FC kept running with the last outputs
            p.advance(now - self._t_last)
        self._t_last = now

        gps = None
        if now >= self._next_gps:
            self._next_gps = now + 1.0 / GPS_HZ
            gps = p.gps(self.gps_fix)
        flags = hitl.BASE_FLAGS | (0 if self.rc_link else hitl.HITL_FAILSAFE_TRIGGERED)
        if self.hold:
            acc, gyro = p.static_acc_mg(), (0, 0, 0)
        else:
            acc, gyro = p.acc_mg(), p.gyro_dps16()
        frame = hitl.pack(flags, acc, gyro, p.baro_pa(), p.mag(), self.rc, gps=gps,
                          airspeed_cms=p.airspeed_cms() if self.use_pitot else None,
                          rangefinder_cm=p.rangefinder_cm() if self.use_rangefinder else None)
        replies = self.msp.request_many(self._requests(frame))
        self.fc = hitl.unpack(replies[0])
        self.motors = _MOTORS.unpack(replies[1])
        if self.vehicle.servos:
            self.servos = struct.unpack(f"<{len(replies[2]) // 2}H", replies[2])
        p.set_outputs(self.motors, self.servos)
        self._log_row(now)

        if self._period:
            # hold the tick rate: sleep off what is left of this tick
            self._next_tick = max(self._next_tick + self._period, now)
            delay = self._next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def _requests(self, frame):
        requests = [(hitl.MSP_SIMULATOR, frame), (MSP_MOTOR, b"")]
        if self.vehicle.servos:
            requests.append((MSP_SERVO, b""))
        return requests

    def release(self):
        """Let physics take over after a held (airborne) start."""
        self.hold = False
        self._t_last = time.monotonic()

    @property
    def elapsed(self):
        return time.monotonic() - self._t0

    # --- pilot inputs ---------------------------------------------------------------

    def sticks(self, roll=None, pitch=None, yaw=None, throttle=None):
        """roll/pitch/yaw in -1..1, throttle in 0..1. Omitted sticks keep their position."""
        for ch, v in ((0, roll), (1, pitch), (3, yaw)):
            if v is not None:
                self.rc[ch] = int(1500 + 500 * max(-1.0, min(1.0, v)))
        if throttle is not None:
            self.rc[2] = int(1000 + 1000 * max(0.0, min(1.0, throttle)))

    def mode(self, name, on=True):
        """Switch a flight mode by its INAV name (e.g. "ANGLE", "NAV POSHOLD", "NAV RTH"),
        using whatever aux range the profile gave it."""
        if name not in self._ranges:
            raise KeyError(f"no aux range for {name!r}; the profile has {sorted(self._ranges)}")
        aux, lo, hi = self._ranges[name][0]
        if on:
            self.rc[4 + aux] = (lo + hi) // 2
        else:
            self.rc[4 + aux] = next(v for v in (1000, 1500, 2000)
                                    if not any(a == aux and l <= v < h for a, l, h in self._ranges[name]))

    def arm(self, timeout=10.0):
        self.sticks(throttle=0.0)
        self.mode("ARM")
        if not self.run(timeout, until=lambda b: b.fc.armed):
            self.mode("ARM", on=False)
            raise ArmingError(f"FC did not arm: {', '.join(self.arming_blockers()) or 'no blocker flagged'}")

    def disarm(self):
        self.mode("ARM", on=False)
        self.run(0.3)

    def wind(self, north_ms=0.0, east_ms=0.0, down_ms=0.0):
        self.plant.set_wind(north_ms, east_ms, down_ms)

    # --- what the FC thinks ---------------------------------------------------------

    def arming_flags(self):
        # u16 cycle time, i2c errors, sensor status, load; u8 profiles; then u32 arming flags
        return struct.unpack_from("<I", self.msp.request(MSP2_INAV_STATUS), 9)[0]

    def arming_blockers(self):
        flags = self.arming_flags()
        return [n for bit, n in ARMING_BLOCKERS.items() if flags & (1 << bit)]

    def active_modes(self):
        mask = int.from_bytes(self.msp.request(MSP_ACTIVEBOXES), "little")
        return {n for i, n in enumerate(self._box_names) if mask >> i & 1}

    def altitude(self):
        """The FC's altitude estimate (m above its home/arming point) and climb rate (m/s)."""
        alt_cm, vario = struct.unpack_from("<ih", self.msp.request(MSP_ALTITUDE))
        return alt_cm / 100.0, vario / 100.0

    def airspeed(self):
        """The FC's airspeed in m/s (from the pitot with pitot=True, otherwise its estimate)."""
        return struct.unpack_from("<I", self.msp.request(MSP2_INAV_AIR_SPEED))[0] / 100.0

    def rangefinder(self):
        """The FC's rangefinder reading in m (needs rangefinder=True), negative when out of range."""
        return struct.unpack_from("<i", self.msp.request(MSP_SONAR_ALTITUDE))[0] / 100.0

    def nav_status(self):
        mode, state, wp_action, wp_number, error = struct.unpack_from("<5B", self.msp.request(MSP_NAV_STATUS))
        return dict(mode=mode, state=state, wp_action=wp_action, wp_number=wp_number, error=error)

    def truth(self):
        return self.plant.truth()

    def _read_modes(self):
        ids = list(self.msp.request(MSP_BOXIDS))
        self._box_names = [BOX_NAMES.get(i, f"BOX{i}") for i in ids]
        name_by_id = dict(zip(ids, self._box_names))
        raw = self.msp.request(MSP_MODE_RANGES)
        self._ranges = {}
        for i in range(0, len(raw), 4):
            perm_id, aux, start, end = raw[i:i + 4]
            if start < end and perm_id in name_by_id:
                if aux > 3:
                    raise ValueError(f"{name_by_id[perm_id]} is on AUX{aux + 1}; MSP_SIMULATOR only carries AUX1-4")
                self._ranges.setdefault(name_by_id[perm_id], []).append((aux, 900 + 25 * start, 900 + 25 * end))

    # --- log ------------------------------------------------------------------------

    def _open_log(self):
        self._log_file = None
        if not self.log_path:
            return
        self._log_file = open(self.log_path, "w", newline="")
        self._log = csv.writer(self._log_file)
        self._log_fields = list(self.plant.truth())
        self._log.writerow(["t", "armed", "fc_roll", "fc_pitch", "fc_yaw", *self._log_fields,
                            *(f"rc{i + 1}" for i in range(8)),
                            *(f"motor{i + 1}" for i in range(len(self.vehicle.motors))),
                            *(f"servo{i}" for i in range(len(self.vehicle.servos)))])
        self._next_log = 0.0

    def _log_row(self, now):
        if not getattr(self, "_log_file", None) or now < self._next_log:
            return
        self._next_log = now + 1.0 / LOG_HZ
        t = self.plant.truth()
        self._log.writerow([f"{now - self._t0:.3f}", int(self.fc.armed), self.fc.roll, self.fc.pitch, self.fc.yaw,
                            *(f"{t[k]:.7f}" if k in ("lat", "lon") else f"{t[k]:.3f}" for k in self._log_fields),
                            *self.rc, *self.motors[:len(self.vehicle.motors)],
                            *self.servos[:len(self.vehicle.servos)]])
