"""MSP_SIMULATOR v3 frames, as parsed by readMspSimulatorValues() in INAV's fc_msp.c."""
import struct
from dataclasses import dataclass

MSP_SIMULATOR = 0x201F

HITL_ENABLE = 1 << 0
HITL_SIMULATE_BATTERY = 1 << 1
HITL_MUTE_BEEPER = 1 << 2
HITL_USE_IMU = 1 << 3
HITL_HAS_NEW_GPS_DATA = 1 << 4
HITL_EXT_BATTERY_VOLTAGE = 1 << 5
HITL_AIRSPEED = 1 << 6
HITL_CURRENT_SENSOR = 1 << 10
HITL_SIM_RC_INPUT = 1 << 11
HITL_RANGEFINDER = 1 << 12
HITL_FAILSAFE_TRIGGERED = 1 << 13

BASE_FLAGS = HITL_ENABLE | HITL_MUTE_BEEPER | HITL_USE_IMU | HITL_SIM_RC_INPUT


MAX_BATTERY_V = 25.5  # INAV carries the simulated battery voltage as one byte of 0.1 V


@dataclass
class Reply:
    armed: bool
    roll: float     # the FC's own attitude estimate, degrees, same signs as the truth:
    pitch: float    # roll right, pitch nose up, yaw clockwise from north all positive
    yaw: float
    stabilized: tuple  # the FC's stabilised roll, pitch, yaw, throttle commands (-500..500; throttle -500 disarmed)


# version, flags | GPS block | euler attitude (unused with HITL_USE_IMU) | acc | gyro | baro | mag
_SENSORS = struct.Struct("<BH" "BBiiihhhhh" "6x" "hhh" "hhh" "I" "hhh")
# rangefinder, current (unused), 8 RC channels, rssi
_TAIL = struct.Struct("<HH8HH")
_AIRSPEED = struct.Struct("<H")
_BATTERY = struct.Struct("<B")
_NO_GPS = (0,) * 10
# field ranges, used only when a value is out of range (a diverged plant)
_U8, _U16, _I16, _I32, _U32 = (0, 255), (0, 65535), (-32768, 32767), (-2**31, 2**31 - 1), (0, 2**32 - 1)
_SENSOR_LIMITS = ([_U8, _U16]                           # version, flags
                  + [_U8, _U8] + [_I32] * 3 + [_I16] * 5  # GPS: fix, sats, lat, lon, alt, speed, course, velNED
                  + [_I16] * 6 + [_U32] + [_I16] * 3)     # acc, gyro, baro, mag


def _pack_clamped(st, values, limits):
    try:
        return st.pack(*values)
    except struct.error:
        return st.pack(*(max(lo, min(hi, v)) for v, (lo, hi) in zip(values, limits)))


def pack(flags, acc_mg, gyro_dps16, baro_pa, mag, rc_us, gps=None, airspeed_cms=None, rangefinder_cm=None, battery=None):
    """gps is a dict (fix, sats, lat_e7, lon_e7, alt_cm, speed_cms, course_dd, vel_ned_cms) or None.
    battery is (volts, amps) for INAV's battery and current meters, or None (INAV then reads a full 3S pack, no current)."""
    if gps is None:
        gps_fields = _NO_GPS
    else:
        flags |= HITL_HAS_NEW_GPS_DATA
        gps_fields = (gps["fix"], gps["sats"], gps["lat_e7"], gps["lon_e7"], gps["alt_cm"],
                      round(gps["speed_cms"]), round(gps["course_dd"]), *map(round, gps["vel_ned_cms"]))
    if airspeed_cms is not None:
        flags |= HITL_AIRSPEED
    if rangefinder_cm is not None:
        flags |= HITL_RANGEFINDER
    if battery is not None:
        flags |= HITL_SIMULATE_BATTERY | HITL_EXT_BATTERY_VOLTAGE | HITL_CURRENT_SENSOR
        if not 0.0 <= battery[0] <= MAX_BATTERY_V:
            raise ValueError(f"battery {battery[0]:.2f} V: INAV's simulator frame carries 0-{MAX_BATTERY_V} V")

    sensors = (3, flags, *gps_fields, *map(round, acc_mg), *map(round, gyro_dps16), round(baro_pa), *map(round, mag))
    p = _pack_clamped(_SENSORS, sensors, _SENSOR_LIMITS)
    if battery is not None:
        p += _BATTERY.pack(round(battery[0] * 10.0))
    if airspeed_cms is not None:
        p += _AIRSPEED.pack(max(0, min(65535, round(airspeed_cms))))
    rangefinder = 0xFFFF if rangefinder_cm is None or rangefinder_cm < 0 else min(65534, round(rangefinder_cm))
    current_da = 0 if battery is None else max(0, min(65535, round(battery[1] * 10.0)))
    return p + _TAIL.pack(rangefinder, current_da, *rc_us, 1000)


def unpack(payload):
    # stabilized roll/pitch/yaw/throttle (4 x i16), debug index u8, debug u32, then attitude
    stabilized = struct.unpack_from("<hhhh", payload, 0)
    roll, pitch, yaw = struct.unpack_from("<hhh", payload, 13)
    # INAV's own pitch is positive nose down; flip it so it compares directly with the truth
    return Reply(armed=bool(payload[8] & 64), roll=roll / 10.0, pitch=-pitch / 10.0, yaw=yaw / 10.0, stabilized=stabilized)
