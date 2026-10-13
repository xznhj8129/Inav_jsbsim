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
import ctypes
import os
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time

from . import hitl
from .msp import Msp, MspError
from .launchers import Catapult
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
MSP2_INAV_ANALOG = 0x2002
MSP2_INAV_AIR_SPEED = 0x2009
MSP2_INAV_MISC2 = 0x203A
MSP_RAW_GPS = 106
MSP2_COMMON_SETTING = 0x1003
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


# What INAV reports, logged with log_fc=True: (MSP command, its reply's leading fields, then the columns they give)
_ANALOG = struct.Struct("<BHhiIiiB")        # flags, V x100, A x100, W x100, mAh, mWh, remaining, percent
_ALTITUDE = struct.Struct("<ihi")           # estimated altitude cm, climb cm/s, baro altitude cm
_RAW_GPS = struct.Struct("<BBiihHHH")       # fix, sats, lat, lon, alt m, speed cm/s, course deg x10, hdop x100
_NAV_STATUS = struct.Struct("<5B")          # mode, state, waypoint action, waypoint number, error
_MISC2 = struct.Struct("<IIBB")             # on time s, flight time s, throttle %, navigation controls throttle
_STATUS = struct.Struct("<HHHHBI")          # cycle us, i2c errors, sensors, cpu load %, profiles, arming flags; then modes, mixer profile
FC_LOG_REQUESTS = (MSP2_INAV_STATUS, MSP2_INAV_ANALOG, MSP_ALTITUDE, MSP2_INAV_AIR_SPEED, MSP_RAW_GPS, MSP_NAV_STATUS, MSP2_INAV_MISC2)
FC_LOG_FIELDS = (
    "fc_stab_roll", "fc_stab_pitch", "fc_stab_yaw", "fc_stab_throttle",
    "fc_battery_v", "fc_battery_a", "fc_battery_w", "fc_battery_mah", "fc_battery_wh", "fc_battery_pct", "fc_battery_state",
    "fc_alt_m", "fc_climb_m_s", "fc_baro_alt_m", "fc_airspeed_m_s",
    "fc_gps_fix", "fc_gps_sats", "fc_gps_alt_m", "fc_gps_speed_m_s", "fc_gps_course_deg",
    "fc_nav_mode", "fc_nav_state", "fc_nav_wp", "fc_nav_error",
    "fc_throttle_pct", "fc_auto_throttle", "fc_flight_time_s",
    "fc_cycle_us", "fc_cpu_load_pct", "fc_arming_flags", "fc_mixer_profile",
)


def fc_telemetry(replies, stabilized, box_names):
    """FC_LOG_FIELDS plus mode_<box> (1 = active) for every box INAV lists, from the FC_LOG_REQUESTS replies
    (a dict by command) and the simulator reply's stabilised commands."""
    flags, volts, amps, power, mah, mwh, _remaining, percent = _ANALOG.unpack_from(replies[MSP2_INAV_ANALOG])
    alt, climb, baro_alt = _ALTITUDE.unpack_from(replies[MSP_ALTITUDE])
    fix, sats, _lat, _lon, gps_alt, speed, course, _hdop = _RAW_GPS.unpack_from(replies[MSP_RAW_GPS])
    nav_mode, nav_state, _wp_action, wp_number, nav_error = _NAV_STATUS.unpack_from(replies[MSP_NAV_STATUS])
    _on_time, flight_time, throttle, auto_throttle = _MISC2.unpack_from(replies[MSP2_INAV_MISC2])
    status = replies[MSP2_INAV_STATUS]
    cycle, _i2c, _sensors, load, _profiles, arming = _STATUS.unpack_from(status)
    modes = int.from_bytes(status[_STATUS.size:-1], "little")
    values = (*stabilized, volts / 100.0, amps / 100.0, power / 100.0, mah, mwh / 1000.0, percent, flags >> 2 & 3,
              alt / 100.0, climb / 100.0, baro_alt / 100.0, struct.unpack_from("<I", replies[MSP2_INAV_AIR_SPEED])[0] / 100.0,
              fix, sats, gps_alt, speed / 100.0, course / 10.0,
              nav_mode, nav_state, wp_number, nav_error,
              throttle, auto_throttle, flight_time,
              cycle, load, arming, status[-1] + 1)
    telemetry = dict(zip(FC_LOG_FIELDS, values))
    telemetry.update({mode_column(name): modes >> index & 1 for index, name in enumerate(box_names)})
    return telemetry


def mode_column(box_name):
    return "mode_" + box_name.replace(" ", "_")


class ArmingError(RuntimeError):
    pass


_PR_SET_PDEATHSIG = 1


def _die_with_parent(parent_pid):
    """Runs in the SITL child before exec: Linux sends it SIGTERM when the bench process dies, however it
    dies, so a killed bench never leaves SITL running. If the bench is already gone, stop here."""
    def setup():
        ctypes.CDLL(None, use_errno=True).prctl(_PR_SET_PDEATHSIG, signal.SIGTERM)
        if os.getppid() != parent_pid:
            os._exit(1)
    return setup


class Bench:
    def __init__(self, vehicle, sitl, *, port=5760, workdir=None, cli=(), airborne=None, alt_m=None,
                 speed_kts=None, heading_deg=0.0, home=HOME, log=None, log_extra=None, log_fc=False, battery=None,
                 pitot=False, rangefinder=False, rate_hz=RATE_HZ, stepped=False, launcher=None, faster=False,
                 gcs_msp=True, gcs_mavlink=True):
        """vehicle: a key of VEHICLES ("quad", "plane") or a Vehicle.
        port: SITL's first TCP port; it takes one per UART, so space parallel benches 10 apart.
        cli: extra INAV CLI lines applied after the vehicle profile, e.g. your own diff.
        home: (lat, lon) start position; truth() north/east are measured from it.
        rate_hz: sim-loop ticks per second; None runs as fast as SITL answers (~850/s, twice the CPU).
        faster: with stepped, SITL waits for the next frame without sleeping (--faster): about twice as
            fast, but SITL holds a CPU core. For a single run; leave it off when several run at once.
        launcher: a launchers.Catapult or launchers.LaunchRail to start on (overrides airborne/alt/speed);
            launch() does the INAV side of leaving it.
        stepped: drive SITL's clock (needs a SITL with the stepped clock): every tick carries a 1/rate_hz step that
            SITL runs before replying, so flight time is sim time and runs as fast as the two processes can go.
        log: CSV path for a per-run flight log (FC vs truth, RC, outputs).
        log_extra: function(bench) -> {column: number}, called for every log row; its columns follow the standard ones.
        log_fc: also log what INAV reports (FC_LOG_FIELDS and every flight mode), read over MSP on each log row.
        gcs_msp/gcs_mavlink: enable external configurator/telemetry UARTs. The bench's own MSP link stays active.
        battery: function(bench) -> (volts, amps) fed to INAV's battery voltage and current meters every tick (the
            config needs feature VBAT and feature CURRENT_METER); None leaves INAV on a simulated full 3S pack."""
        self.vehicle = VEHICLES[vehicle] if isinstance(vehicle, str) else vehicle
        self.sitl = os.path.abspath(sitl)
        self.port = port
        self.gcs_msp_port = port + 1
        self.gcs_mavlink_port = port + 2
        self.gcs_msp = gcs_msp
        self.gcs_mavlink = gcs_mavlink
        self.extra_cli = list(cli)
        self.airborne = self.vehicle.airborne if airborne is None else airborne
        self.start_state = dict(airborne=self.airborne, alt_m=alt_m, speed_kts=speed_kts, heading_deg=heading_deg,
                                home=home, launcher=launcher)
        self.use_pitot = pitot
        self.use_rangefinder = rangefinder
        self.log_path = log
        self.log_extra = log_extra
        self.log_fc = log_fc
        self.battery = battery
        self.fc_telemetry = {}  # the last FC_LOG_FIELDS and mode columns read (log_fc)
        if stepped and not rate_hz:
            raise ValueError("stepped needs a rate_hz: it sets the step")
        self._period = 1.0 / rate_hz if rate_hz else 0.0
        self.stepped = stepped
        self.faster = faster
        self._sim_t = 0.0       # flight time on the stepped clock, s
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
        self.mixer_profile = None   # tailsitters: the FC's active mixer profile (0-based), read every tick
        self.hold = False       # True pins the vehicle in place (no physics, pure gravity)
        self.plant = None
        self.msp = None
        self._proc = None

    # --- lifecycle ------------------------------------------------------------------

    def __enter__(self):
        try:
            self.start()
        except BaseException:
            # a failed start never reaches __exit__: stop the SITL it launched, keep its log and eeprom
            self.stop(keep_workdir=True)
            raise
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
            [self.sitl, f"--path={eeprom}", f"--tcpbaseport={self.port}",
             *(["--faster"] if self.stepped and self.faster else [])],
            cwd=self.workdir, stdout=self._sitl_log, stderr=subprocess.STDOUT, start_new_session=True,
            preexec_fn=_die_with_parent(os.getpid()) if sys.platform.startswith("linux") else None)

        msp = self._connect()
        # INAV with explicit MAVLink port binding refuses to arm while UART3's MAVLink port has no slot
        bind_mavlink = self._has_setting(msp, "mavlink_port1_serial")
        msp.cli(self._profile_lines(bind_mavlink))
        msp.cli_save()          # reboots the SITL into the new config
        self.msp = self._connect()
        print(f"GCS: MSP {'on tcp://127.0.0.1:' + str(self.gcs_msp_port) if self.gcs_msp else 'disabled'}, "
              f"MAVLink {'on tcp://127.0.0.1:' + str(self.gcs_mavlink_port) if self.gcs_mavlink else 'disabled'}")

        self._read_modes()
        self._wait_boot_calibration()
        self.plant = Plant(self.vehicle, **self.start_state)
        # an airborne start is pinned in place until release(); on a launcher JSBSim runs and the launcher holds it
        self.hold = self.airborne and not self.plant.rail
        self._t_last = self._now()
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

    @staticmethod
    def _has_setting(msp, name):
        try:
            msp.request(MSP2_COMMON_SETTING, name.encode() + b"\0")
        except MspError:
            return False
        return True

    def _profile_lines(self, bind_mavlink):
        with open(os.path.join(PROFILES, self.vehicle.profile)) as f:
            lines = [ln.split("#", 1)[0].strip() for ln in f]
        # SITL maps UART n to TCP port base + n - 1. UART1 is the bench's own link;
        # UART2 and UART3 are left for a GCS / configurator to connect to
        gcs = []
        for protocol, enabled in (("msp", self.gcs_msp), ("mavlink", self.gcs_mavlink)):
            with open(os.path.join(PROFILES, f"gcs_{protocol}_{'on' if enabled else 'off'}.cli")) as f:
                gcs.extend(ln.split("#", 1)[0].strip() for ln in f)
        if bind_mavlink and self.gcs_mavlink:
            gcs.append("set mavlink_port1_serial = 2")         # UART3 in the serial command's numbering
        # INAV only takes HITL airspeed / rangefinder data into a configured sensor
        if self.use_pitot:
            lines.append("set pitot_hardware = FAKE")
        if self.use_rangefinder:
            lines.append("set rangefinder_hardware = FAKE")
        return [ln for ln in lines if ln] + self.extra_cli + [ln for ln in gcs if ln]

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

    def _now(self):
        """Flight time: the stepped clock's when stepped, wall time otherwise."""
        return self._sim_t if self.stepped else time.monotonic()

    def run(self, seconds, until=None, check_every=0.05):
        """Fly for `seconds` of flight time. `until(bench)` ends it early by returning True;
        it is checked every `check_every` s. Returns True if `until` fired."""
        end = self._now() + seconds
        next_check = 0.0
        while self._now() < end:
            self._tick()
            if until is not None and self._now() >= next_check:
                next_check = self._now() + check_every
                if until(self):
                    return True
        return False

    def _tick(self):
        self.ticks += 1
        if self.stepped:
            # the FC ran the last step with the last outputs; its next one starts here
            self._sim_t += self._period
        now = self._now()
        p = self.plant
        if not self.hold:
            # advance by the full gap: the FC kept running with the last outputs
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
                          rangefinder_cm=p.rangefinder_cm() if self.use_rangefinder else None,
                          battery=self.battery(self) if self.battery else None,
                          step_us=round(self._period * 1e6) if self.stepped else None)
        fc_log_due = self.log_fc and getattr(self, "_log_file", None) and now >= self._next_log
        requests = self._requests(frame, fc_log_due)
        replies = dict(zip((cmd for cmd, _ in requests), self.msp.request_many(requests)))
        self.fc = hitl.unpack(replies[hitl.MSP_SIMULATOR])
        if self.stepped and not self.fc.stepped:
            raise RuntimeError("SITL did not take the step: this build has no stepped clock")
        self.motors = _MOTORS.unpack(replies[MSP_MOTOR])
        if self.vehicle.tailsitter:
            # MSP2_INAV_STATUS ends with the active mixer profile; sensors follow it from the next tick
            self.mixer_profile = replies[MSP2_INAV_STATUS][-1]
            p.hover_frame = self.mixer_profile == self.vehicle.hover_mixer_profile
        if self.vehicle.servos:
            self.servos = struct.unpack(f"<{len(replies[MSP_SERVO]) // 2}H", replies[MSP_SERVO])
        if fc_log_due:
            self.fc_telemetry = fc_telemetry(replies, self.fc.stabilized, self._box_names)
        p.set_outputs(self.motors, self.servos)
        self._log_row(now)

        if self._period and not self.stepped:
            # hold the tick rate: sleep off what is left of this tick
            self._next_tick = max(self._next_tick + self._period, now)
            delay = self._next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def _requests(self, frame, fc_log):
        requests = [(hitl.MSP_SIMULATOR, frame), (MSP_MOTOR, b"")]
        if self.vehicle.servos:
            requests.append((MSP_SERVO, b""))
        if fc_log:
            requests += [(cmd, b"") for cmd in FC_LOG_REQUESTS]
        elif self.vehicle.tailsitter:
            requests.append((MSP2_INAV_STATUS, b""))
        return requests

    def launch(self, throttle, modes=(), wait_s=1.0, arm_timeout=10.0, rail_timeout=10.0):
        """The INAV side of leaving the launcher the vehicle started on (Bench(launcher=...)): switch on
        `modes`, arm while it is held, set `throttle` and run wait_s with the vehicle still held; then a
        catapult fires, and on a launch rail its own thrust takes it once that beats weight and drag. Returns
        once the vehicle is off the rail's end; what it flies after that is the caller's.

        modes=() launches in ACRO: nothing self-levels, so no controller winds up against the attitude the
        rail holds, and the motor runs up while the vehicle is held. modes=("NAV LAUNCH",) uses INAV's
        fixed-wing launch, which keeps the motor at idle until it detects the throw: that needs a catapult,
        and with nav_fw_launch_idle_thr set it spins up to idle over 1.5 s first, so wait_s must be longer.
        Every mode needs a switch (aux range) in the vehicle's profile."""
        rail = self.plant.rail
        if not (rail and rail.held):
            raise RuntimeError("not held on a launcher: start with Bench(launcher=...)")
        for name in modes:
            self.mode(name)
        self.arm(arm_timeout)
        self.sticks(throttle=throttle)
        self.run(wait_s)
        if isinstance(rail.launcher, Catapult):
            self.fire()
        deadline = self._now() + rail_timeout
        while self.plant.on_rail:
            if self._now() > deadline:
                raise RuntimeError(f"still on the launch rail after {rail_timeout:.0f} s: its thrust along the rail "
                                   f"does not beat weight and drag")
            self.run(self._period or 0.001)

    def fire(self):
        """Fire the catapult the vehicle started on, nothing else: launch() does the INAV side around it."""
        self.plant.fire()

    def release(self):
        """Let physics take over after a held (airborne) start."""
        self.hold = False
        self._t_last = self._now()

    @property
    def elapsed(self):
        return self._now() - self._t0

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
                                    if not any(channel == aux and low <= v < high for channel, low, high in self._ranges[name]))

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
        self._extra_fields = list(self.log_extra(self)) if self.log_extra else []
        self._fc_fields = [*FC_LOG_FIELDS, *map(mode_column, self._box_names)] if self.log_fc else []
        self._log.writerow(["t", "armed", "fc_roll", "fc_pitch", "fc_yaw", *self._log_fields,
                            *(f"rc{i + 1}" for i in range(8)),
                            *(f"motor{i + 1}" for i in range(len(self.vehicle.motors))),
                            *(f"servo{i}" for i in range(len(self.vehicle.servos))),
                            *self._fc_fields, *self._extra_fields])
        self._next_log = 0.0

    def _log_row(self, now):
        if not getattr(self, "_log_file", None) or now < self._next_log:
            return
        self._next_log = now + 1.0 / LOG_HZ
        t = self.plant.truth()
        self._log.writerow([f"{now - self._t0:.3f}", int(self.fc.armed), self.fc.roll, self.fc.pitch, self.fc.yaw,
                            *(f"{t[k]:.7f}" if k in ("lat", "lon") else f"{t[k]:.3f}" for k in self._log_fields),
                            *self.rc, *self.motors[:len(self.vehicle.motors)],
                            *self.servos[:len(self.vehicle.servos)],
                            *(self.fc_telemetry.get(key, "") for key in self._fc_fields),
                            *(f"{value:.4f}" for value in self._extra_values())])

    def _extra_values(self):
        if not self._extra_fields:
            return []
        extra = self.log_extra(self)
        return [extra[key] for key in self._extra_fields]
