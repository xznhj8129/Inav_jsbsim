#!/usr/bin/env python3
"""Turn swarm_replay.json (from swarm_flock.py) into one self-contained 3D replay page.

    python3 examples/swarm_replay.py swarm_replay.json swarm_replay.html
"""
import json
import math
import os
import sys

TEMPLATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "swarm_replay_template.html")


def compact(replay):
    """Relative times, per-quad columns: small enough to embed."""
    t0 = replay["t0"]
    quads = {}
    for sysid, q in sorted(replay["quads"].items(), key=lambda kv: int(kv[0])):
        log = q["log"]
        quads[sysid] = {
            "landed": q["landed"],
            "t": [round(s["t"] - t0, 2) for s in log],
            "p": [s["truth"] for s in log],
            "tgt": [s["target"] for s in log],
            "seg": [s["seg"] for s in log],
            "n": [s["n"] for s in log],
            "em": [s["emergency"] for s in log],
        }
    links = [[round(t - t0, 2), a, b] for t, a, b in replay["links"]]
    errors = [math.dist(s["truth"][:2], s["fc"][:2]) for q in replay["quads"].values() for s in q["log"]]
    return {"metar": replay["metar"], "wind": replay["wind"], "route": replay["route"],
            "radio_range_m": replay["radio_range_m"], "radio_loss": replay["radio_loss"],
            "quads": quads, "links": links, "fc_error_m": sum(errors) / len(errors)}


def main():
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as f:
        data = compact(json.load(f))
    with open(TEMPLATE) as f:
        page = f.read()
    page = page.replace("/*REPLAY_DATA*/null", json.dumps(data, separators=(",", ":")))
    with open(dst, "w") as f:
        f.write(page)
    print(f"wrote {dst} ({os.path.getsize(dst) / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
