"""Live wind for the sims, from a real METAR."""
import json
import math
import urllib.request

STATION = "KMOR"                        # Morristown TN, the reporting station nearest Sneedville


def fetch_wind(station=STATION):
    """Live METAR wind as (north, east) m/s the air moves toward, plus the raw report."""
    url = f"https://aviationweather.gov/api/data/metar?ids={station}&format=json"
    with urllib.request.urlopen(url, timeout=10) as r:
        m = json.load(r)[0]
    speed = (m.get("wspd") or 0) * 0.514444
    wdir = m.get("wdir")
    if not isinstance(wdir, (int, float)):          # "VRB": no steady direction to use
        speed = 0.0
        wdir = 0
    to = math.radians(wdir + 180.0)
    return (speed * math.cos(to), speed * math.sin(to)), m["rawOb"]
