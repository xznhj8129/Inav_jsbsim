"""Flying INAV from outside, as a GCS does: GCS NAV moves NAV POSHOLD's target (waypoint #255)."""
import math
import struct

from .vehicles import HOME

GCSNAV_AUX = "aux 7 31 2 1700 2100"     # GCS NAV on AUX3 high, the same position as NAV POSHOLD in the profiles
MSP_SET_WP = 209
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LON = 111320.0 * math.cos(math.radians(HOME[0]))


def set_target(b, east, north, alt, heading):
    """Send a vehicle in NAV POSHOLD + GCS NAV to east/north metres from HOME, alt metres above
    the FC's origin (where it armed), facing heading degrees. INAV flies it there itself."""
    lat = HOME[0] + north / M_PER_DEG_LAT
    lon = HOME[1] + east / M_PER_DEG_LON
    payload = struct.pack("<BBiiihhhB", 255, 1, round(lat * 1e7), round(lon * 1e7),
                          max(500, round(alt * 100)), round(heading) % 360 or 1, 0, 0, 0)   # INAV takes 1..359
    b.msp.request(MSP_SET_WP, payload)


def gcs_addresses(specs):
    """--gcs HOST[:PORT] arguments as (host, port), port 14550 if omitted."""
    return [(h.rsplit(":", 1)[0], int(h.rsplit(":", 1)[1])) if ":" in h else (h, 14550) for h in specs]
