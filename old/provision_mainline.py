#!/usr/bin/env python3
"""Provision a mainline INAV SITL for JSBSim/HITL testing.

Mainline-safe subset of inav-sitl-bench's bench.py provision() — see the
jsbsim-sitl-testing skill. Run once against a fresh SITL, then restart the
SITL so the saved eeprom takes effect.

Usage: python3 provision_mainline.py
"""
import os
import struct
import sys

BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "inav-sitl-bench")
sys.path.insert(0, BENCH)

from msp import MspClient


def provision_mainline(extra_mode_ranges=()):
    msp = MspClient()
    print("API", msp.api_version())

    msp.set_setting("receiver_type", struct.pack("<B", 3))     # SIM (SITL)
    msp.set_setting("platform_type", struct.pack("<B", 1))     # AIRPLANE
    msp.set_setting("small_angle", struct.pack("<B", 180))
    msp.set_setting("baro_hardware", struct.pack("<B", 12))    # FAKE
    msp.set_setting("mag_hardware", struct.pack("<B", 0))      # NONE
    msp.set_setting("init_gyro_cal", struct.pack("<B", 0))     # no real sensors behind HITL
    msp.set_setting("pitot_hardware", struct.pack("<B", 0))    # NONE
    msp.enable_feature(1 << 7)                                 # FEATURE_GPS
    msp.set_setting("gps_provider", struct.pack("<B", 1))      # MSP-driven

    # Standard airplane servo mixer (S1 aileron, S2 elevator, S3 rudder) —
    # without this isMixerUsingServos() is false and the MSP_SIMULATOR
    # reply's stabilized outputs stay 0.
    msp.set_servo_mixer_rule(0, 0, 0)   # servo 0 <- stabilized roll
    msp.set_servo_mixer_rule(1, 1, 1)   # servo 1 <- stabilized pitch
    msp.set_servo_mixer_rule(2, 2, 2)   # servo 2 <- stabilized yaw

    PERM_ARM, PERM_ANGLE = 0, 1
    CH_ARM, CH_ANGLE = 4, 5              # AUX1/AUX2 in AETR+AUX1..4 layout
    msp.set_mode_range(0, PERM_ARM, CH_ARM - 4, 1700, 2100)
    msp.set_mode_range(1, PERM_ANGLE, CH_ANGLE - 4, 1700, 2100)

    for i, (perm_id, aux_ch, start_pwm, end_pwm) in enumerate(extra_mode_ranges, start=2):
        msp.set_mode_range(i, perm_id, aux_ch - 4, start_pwm, end_pwm)

    msp.save_eeprom()
    print("provisioned + saved — restart the SITL now (eeprom needs a reboot)")


if __name__ == "__main__":
    provision_mainline()
