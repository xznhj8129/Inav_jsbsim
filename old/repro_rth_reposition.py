#!/usr/bin/env python3
"""Reproduce: does a stick nudge during an RTH landing move the position target?

The firmware is real; only the world is simulated. Sensors and RC go in over
MSP_SIMULATOR, so we can place the aircraft wherever we like and then read the
nav target straight out of the FC with MSP2_INAV_NAV_TARGET, which reports
posControl.desiredState.pos.

nav_user_control_mode is ATTI, so a held stick tilts the aircraft without moving
the target; the target is only meant to be updated on release, by
setMulticopterStopPosition(). We move the aircraft ourselves to stand in for that
tilt, release the stick, and see where the target ends up.
"""
import os, struct, sys, time
BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "inav-sitl-bench")
sys.path.insert(0, BENCH)
from msp import MspClient
import hitl

MSP_NAV_STATUS          = 121
MSP2_INAV_STATUS        = 0x2000
ARM_BLOCK = {6:"GEOZONE",7:"FAILSAFE_SYSTEM",8:"NOT_LEVEL",9:"SENSORS_CALIBRATING",
             10:"SYSTEM_OVERLOADED",11:"NAVIGATION_UNSAFE",12:"COMPASS_NOT_CALIBRATED",
             13:"ACC_NOT_CALIBRATED",14:"ARM_SWITCH",15:"HARDWARE_FAILURE",16:"BOXFAILSAFE",
             18:"RC_LINK",19:"THROTTLE",23:"ROLLPITCH_NOT_CENTERED",26:"INVALID_SETTING",
             28:"NO_PREARM",30:"LANDING_DETECTED"}
MSP2_INAV_NAV_TARGET    = 0x221B
MSP2_INAV_ACTIVATE_RTH  = 0x2225
MSP2_INAV_ACTIVATE_LANDING = 0x2224
MW_NAV_STATE = {0:"NONE",1:"RTH_START",2:"RTH_ENROUTE",3:"HOLD_INFINIT",4:"HOLD_TIMED",
                5:"WP_ENROUTE",6:"PROCESS_NEXT",7:"DO_JUMP",8:"LAND_START",
                9:"LAND_IN_PROGRESS",10:"LANDED",11:"LAND_SETTLE",12:"LAND_START_DESCENT",
                13:"HOVER_ABOVE_HOME",14:"EMERGENCY_LANDING",15:"RTH_CLIMB"}

HOME_LAT, HOME_LON = 365304400, -832163830      # Sneedville, TN, 1e7 degrees
M_PER_DEG = 1.113195e5 / 1e7                     # metres per 1e-7 degree of latitude

RC_MID, RC_LOW, RC_HIGH = 1500, 1000, 1900
CH = dict(roll=0, pitch=1, throttle=2, yaw=3, arm=4, angle=5, poshold=6, rth=7)

def rc(**kw):
    ch = [RC_MID, RC_MID, RC_LOW, RC_MID, RC_LOW, RC_LOW, RC_LOW, RC_LOW]
    for k, v in kw.items():
        ch[CH[k]] = v
    return ch

def arming_blockers(msp):
    af = struct.unpack_from("<I", msp.request(MSP2_INAV_STATUS), 9)[0]  # u16 cycle,i2c,sensors,load + u8 profile
    return af, [n for b, n in ARM_BLOCK.items() if af & (1 << b)]


def nav_state(msp):
    p = msp.request(MSP_NAV_STATUS)
    return p[1]

def nav_target(msp):
    """Returns the FC's desired position as (lat_e7, lon_e7, alt_cm)."""
    p = msp.request(MSP2_INAV_NAV_TARGET)
    lat, lon, alt = struct.unpack_from("<iii", p, 0)
    return lat, lon, alt

class World:
    """Where we are telling the flight controller it is."""
    def __init__(self):
        self.lat, self.lon = HOME_LAT, HOME_LON
        self.alt_cm = 0
        self.vel_ned = (0, 0, 0)
    def gps(self):
        return dict(lat_e7=self.lat, lon_e7=self.lon, alt_cm=self.alt_cm,
                    speed_cms=0, course_dd=0, vel_ned_cms=list(self.vel_ned))
    def baro_pa(self):
        return int(101325 * (1 - 2.25577e-5 * (self.alt_cm / 100.0)) ** 5.25588)

# A multirotor will not engage nav modes without a heading reference, so unlike the
# existing fixed-wing bench script (a plane can infer heading from GPS course) this
# needs a magnetometer. mag_hardware is set to FAKE and fed over the HITL packet.
SIM_FLAGS = hitl.BENCH_FLAGS          # includes HITL_USE_IMU: real AHRS from injected acc/gyro
MAG_NORTH = (4000, 0, 0)              # field pointing north, so the AHRS has a heading


def step(msp, w, rc_us, n=1, dt=0.01):
    r = None
    for _ in range(n):
        r = hitl.sim_step(msp, (0, 0, 1000), (0, 0, 0), rc_us,
                          baro_pa=w.baro_pa(), gps=w.gps(), mag=MAG_NORTH, flags=SIM_FLAGS)
        time.sleep(dt)
    return r

def main():
    msp = MspClient()
    w = World()
    print("API", msp.api_version())

    # compassUpdate() returns early once SIMULATOR_MODE_HITL is set, before the
    # SITL_BUILD line that marks the compass calibrated. So let the FC run on its own
    # first: once we start injecting, that flag can never be set and arming is blocked
    # by COMPASS_NOT_CALIBRATED for as long as a mag is configured.
    print("-- letting the FC boot before HITL injection starts --")
    time.sleep(3)

    print("\n-- settling sensors / GPS fix --")
    step(msp, w, rc(), 800)

    print("-- arming --")
    step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH), 50)
    r = step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, throttle=1200), 50)
    print(f"   armed={r.armed}")
    if not r.armed:
        af, blk = arming_blockers(msp)
        print(f"   !! did not arm (flags=0x{af:08x}): {', '.join(blk) or 'none'}")
        return

    print("-- climbing to 30 m --")
    for i in range(1, 31):
        w.alt_cm = i * 100
        step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, poshold=RC_HIGH, throttle=1600), 10)
    print(f"   alt={w.alt_cm/100:.0f} m  nav={MW_NAV_STATE.get(nav_state(msp))}")

    print("-- flying 40 m north of home --")
    for i in range(1, 41):
        w.lat = HOME_LAT + int(i / M_PER_DEG)
        step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, poshold=RC_HIGH, throttle=1500), 5)
    print(f"   nav={MW_NAV_STATE.get(nav_state(msp))}")

    via = sys.argv[1] if len(sys.argv) > 1 else "switch"
    phase = sys.argv[2] if len(sys.argv) > 2 else "land"
    print(f"-- commanding RTH via {via} --")
    rth_sw = RC_HIGH if via == "switch" else RC_LOW
    if via == "landhere":
        # QGC "Land Here" -> MAV_CMD_NAV_LAND -> activateForcedLanding(), which is
        # what MSP2_INAV_ACTIVATE_LANDING calls too. Lands on the spot, not at home.
        try:
            msp.request(MSP2_INAV_ACTIVATE_LANDING)
        except OSError as e:
            print(f"   !! {e}"); return
    if via == "msp":
        # MSP2_INAV_ACTIVATE_RTH is exactly what the MAVLink DO_SET_MODE handler calls
        # (activateRTHMode), which is how QGC's Return button reaches the FC.
        try:
            msp.request(MSP2_INAV_ACTIVATE_RTH)
        except OSError as e:
            print(f"   !! {e}"); return
    deadline = time.time() + 60
    seen = set()
    while time.time() < deadline:
        step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, poshold=RC_HIGH, rth=rth_sw, throttle=1500), 5)
        st = nav_state(msp)
        if st not in seen:
            seen.add(st); print(f"   nav -> {MW_NAV_STATE.get(st, st)}")
        # let it converge on home as RTH would
        if phase != "enroute" and abs(w.lat - HOME_LAT) > 0:
            w.lat += int((HOME_LAT - w.lat) * 0.05) or (1 if HOME_LAT > w.lat else -1)
        if phase == "enroute" and st == 2:    # RTH_ENROUTE
            break
        if phase == "land" and st == 9:       # LAND_IN_PROGRESS
            break
        if via == "landhere" and st in (9, 10):
            break
    want = 2 if phase == "enroute" else 9
    if via == "landhere" and nav_state(msp) in (9, 10):
        want = nav_state(msp)
    if nav_state(msp) != want:
        print(f"   !! never reached {MW_NAV_STATE[want]} (state={MW_NAV_STATE.get(nav_state(msp))})"); return

    print(f"\n-- in RTH {phase}; nudging with the roll stick (nav={MW_NAV_STATE.get(nav_state(msp))}) --")
    before = nav_target(msp)
    print(f"   target before      lat={before[0]} lon={before[1]}")

    # Hold roll for 3 s, moving the aircraft 15 m east at a steady 5 m/s and
    # reporting that velocity, standing in for the tilt the stick would produce.
    N = 300
    descend = (phase == "land")          # actually come down while nudging
    for i in range(N):
        w.lon = HOME_LON + int((i / N * 15) / M_PER_DEG)
        if descend:
            w.alt_cm = max(100, w.alt_cm - 5)        # ~1.7 m/s down
            w.vel_ned = (0, 500, 150)                # east + down, cm/s
        else:
            w.vel_ned = (0, 500, 0)
        step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, poshold=RC_HIGH, rth=rth_sw, throttle=1500, roll=1800), 1)
    w.vel_ned = (0, 0, 150 if descend else 0)
    during = nav_target(msp)
    print(f"   target while held  lat={during[0]} lon={during[1]}")

    # Release: centre the stick and let it settle.
    for _ in range(300):
        if descend:
            w.alt_cm = max(100, w.alt_cm - 5)
        step(msp, w, rc(arm=RC_HIGH, angle=RC_HIGH, poshold=RC_HIGH, rth=rth_sw, throttle=1500), 1)
    after = nav_target(msp)
    print(f"   altitude now {w.alt_cm/100:.1f} m, nav={MW_NAV_STATE.get(nav_state(msp))}")
    print(f"   target on release  lat={after[0]} lon={after[1]}")

    moved_m = (w.lon - HOME_LON) * M_PER_DEG
    tgt_m   = (after[1] - HOME_LON) * M_PER_DEG
    print(f"\n   aircraft moved {moved_m:.1f} m east of home")
    print(f"   target ended   {tgt_m:.1f} m east of home")
    if abs(tgt_m) < 3.0:
        print("   RESULT: target still at home - REPRODUCED, it snaps back")
    else:
        print(f"   RESULT: target moved off home with the aircraft - reposition HELD")
        print(f"           (offset from aircraft {tgt_m - moved_m:+.1f} m is world-model noise:")
        print(f"            estimator lag and the velocity extrapolation in")
        print(f"            calculateMulticopterInitialHoldPosition)")

main()
