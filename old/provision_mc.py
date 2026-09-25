#!/usr/bin/env python3
"""Provision an INAV SITL as a multirotor for HITL nav testing.

Run once against a fresh SITL, then restart it - the eeprom write only takes
effect on reboot.
"""
import os, struct, sys
BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "inav-sitl-bench")
sys.path.insert(0, BENCH)
from msp import MspClient

PERM_ARM, PERM_ANGLE, PERM_NAV_POSHOLD, PERM_NAV_RTH = 0, 1, 11, 10
CH_ARM, CH_ANGLE, CH_POSHOLD, CH_RTH = 5, 6, 7, 8

def main():
    msp = MspClient()
    print("API", msp.api_version())

    msp.set_setting("receiver_type",   struct.pack("<B", 3))   # SIM
    msp.set_setting("platform_type",   struct.pack("<B", 0))   # MULTIROTOR
    msp.set_setting("small_angle",     struct.pack("<B", 180))
    msp.set_setting("baro_hardware",   struct.pack("<B", 12))  # FAKE
    msp.set_setting("mag_hardware",    struct.pack("<B", 16))  # FAKE (fed over HITL)
    msp.set_setting("init_gyro_cal",   struct.pack("<B", 0))
    msp.set_setting("pitot_hardware",  struct.pack("<B", 0))
    msp.enable_feature(1 << 7)                                 # FEATURE_GPS
    msp.set_setting("gps_provider",    struct.pack("<B", 1))   # MSP-driven

    # The behaviour under test: ATTI means a stick tilts the craft without moving
    # the position target, which is only updated when the stick is released.
    msp.set_setting("nav_user_control_mode",   struct.pack("<B", 0))  # ATTI
    msp.set_setting("nav_rth_allow_landing",   struct.pack("<B", 1))  # ALWAYS
    msp.set_setting("nav_extra_arming_safety", struct.pack("<B", 1))  # ALLOW_BYPASS

    # STATE(COMPASS_CALIBRATED) is false while magZero is all zero with default gains,
    # which is the factory-uncalibrated signature and blocks arming. A non-zero offset
    # is enough to mark it calibrated; the field itself is injected over HITL.
    msp.set_setting("magzero_x", struct.pack("<h", 1))
    msp.set_setting("nav_rth_climb_first",     struct.pack("<B", 0))  # go straight home

    msp.set_mode_range(0, PERM_ARM,          CH_ARM     - 5, 1700, 2100)
    msp.set_mode_range(1, PERM_ANGLE,        CH_ANGLE   - 5, 1700, 2100)
    msp.set_mode_range(2, PERM_NAV_POSHOLD,  CH_POSHOLD - 5, 1700, 2100)
    msp.set_mode_range(3, PERM_NAV_RTH,      CH_RTH     - 5, 1700, 2100)

    msp.save_eeprom()
    print("provisioned + saved - restart SITL for the eeprom to take effect")

main()
