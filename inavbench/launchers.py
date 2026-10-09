"""Launchers for any JSBSim vehicle: a catapult, and an unpowered launch rail.

Both are a perfect rail along the vehicle's nose. On it the vehicle's attitude is locked and it can only move
along the rail; it flies free at the rail's end.

- Catapult: latched until fired, then pushed along the rail at constant acceleration to a set speed.
- LaunchRail: nothing pushes. The vehicle sits at the rail's base and only its own thrust (against weight and
  drag) moves it along, like a model rocket on its launch rod.

JSBSim runs throughout. While the vehicle is not moving along the rail (a latched catapult, or a vehicle
resting at a launch rail's base) JSBSim's forces/hold-down holds it exactly. Once it moves, the bench drives
a `launch_rail` external force, added to a copy of the vehicle's model (launcher_model), so the total
acceleration is what the rail allows: nothing across it, and along it the catapult's push or the vehicle's
own forces. JSBSim applies an external force a step late, so it is only used while the forces change
smoothly; a held vehicle is never balanced by it.

Use it through the bench: Bench(vehicle, launcher=Catapult(...) or LaunchRail(...)), then Bench.launch(...)
for the INAV side of a launch.
"""
import math
import os
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import jsbsim

from .units import FT


@dataclass(frozen=True)
class Catapult:
    """Held still at pitch_deg, height_m above the ground, until fired; then pushed along the nose at constant
    acceleration to speed_ms over length_m. Heading is the bench's heading_deg."""
    pitch_deg: float = 10.0
    length_m: float = 2.0
    speed_ms: float = 15.0
    height_m: float = 1.5

    @property
    def stroke_s(self):
        return 2.0 * self.length_m / self.speed_ms


@dataclass(frozen=True)
class LaunchRail:
    """A rail of length_m along the nose at pitch_deg, its base height_m above the ground. Nothing pushes:
    the vehicle rests at the base until its thrust along the rail beats weight and drag, then runs off the
    end. Heading is the bench's heading_deg. INAV only arms within small_angle (25 deg) of level: a steeper
    rail needs it raised in the vehicle's config (Bench(cli=["set small_angle = 90"]))."""
    pitch_deg: float = 85.0
    length_m: float = 1.0
    height_m: float = 1.0


def launcher_model(model, model_dir, into):
    """Copy JSBSim model `model` (from model_dir, or JSBSim's bundled set when empty) into `into` and add the
    launch_rail external force to the copy. Returns (aircraft path, engine path) for JSBSim."""
    root = model_dir or jsbsim.get_default_root_dir()
    aircraft = os.path.join(into, "aircraft")
    shutil.copytree(os.path.join(root, "aircraft", model), os.path.join(aircraft, model))
    path = os.path.join(aircraft, model, model + ".xml")
    tree = ET.parse(path)
    reactions = tree.getroot().find("external_reactions")
    if reactions is None:
        reactions = ET.SubElement(tree.getroot(), "external_reactions")
    # direction and magnitude are set every step (Rail.before_step); it acts through the CG (Rail.__init__)
    force = ET.SubElement(reactions, "force", name="launch_rail", frame="BODY")
    location = ET.SubElement(force, "location", unit="IN")
    for axis in "xyz":
        ET.SubElement(location, axis).text = "0"
    tree.write(path)
    return aircraft, os.path.join(root, "engine")


class Rail:
    """A launcher acting on a JSBSim model built by launcher_model, from creation until the vehicle leaves the
    rail's end. The owner calls before_step()/after_step() around every fdm.run()."""

    GAIN = 50.0     # 1/s: how hard the rail pulls the velocity back onto what it allows
    _SOURCES = ("aero", "prop", "gear", "weight")
    _ROTATION_INTEGRATORS = ("simulation/integrator/rate/rotational", "simulation/integrator/position/rotational")

    def __init__(self, fdm, launcher, dt):
        self.fdm, self.launcher, self.dt = fdm, launcher, dt
        self.on_rail = True
        self._latched = isinstance(launcher, Catapult)     # a catapult holds the vehicle until fire()
        self._steps = 0
        self._t = 0.0           # time since the catapult fired, s
        self._run_ft = 0.0      # distance along the rail from its base
        for axis in "xyz":
            fdm[f"external_reactions/launch_rail/location-{axis}-in"] = fdm[f"inertia/cg-{axis}-in"]
        # attitude locked: rotation is not integrated while on the rail
        self._saved_integrators = {k: fdm[k] for k in self._ROTATION_INTEGRATORS}
        for k in self._ROTATION_INTEGRATORS:
            fdm[k] = 0
        fdm["forces/hold-down"] = 1

    @property
    def held(self):
        """Held still: a latched catapult, or resting at a launch rail's base."""
        return self.fdm["forces/hold-down"] == 1

    def fire(self):
        if not self._latched:
            raise RuntimeError("nothing to fire: no catapult latched")
        self._latched = False
        self.fdm["forces/hold-down"] = 0

    def _force(self, axis):
        f = self.fdm
        return sum(f[f"forces/fb{axis}-{source}-lbs"] for source in self._SOURCES)

    def before_step(self):
        if not self.on_rail:
            return
        f = self.fdm
        if self.held:
            # a launch rail lets go once thrust along it beats weight and drag; the first step's forces are
            # not yet computed
            if not self._latched and self._steps > 0 and self._force("x") > 0.0:
                f["forces/hold-down"] = 0
            else:
                return
        mass = f["inertia/mass-slugs"]
        u, v, w = (f[f"velocities/{axis}-fps"] for axis in "uvw")
        if isinstance(self.launcher, Catapult):
            c = self.launcher
            accel = (c.speed_ms / FT) ** 2 / (2.0 * c.length_m / FT)       # ft/s^2
            along = mass * (accel + self.GAIN * (accel * (self._t + self.dt) - u)) - self._force("x")
        else:
            along = 0.0         # nothing pushes: the vehicle's own forces move it along
        force = (along, mass * -self.GAIN * v - self._force("y"), mass * -self.GAIN * w - self._force("z"))
        magnitude = math.sqrt(sum(component * component for component in force))
        if magnitude > 0.0:
            for axis, component in zip("xyz", force):
                f[f"external_reactions/launch_rail/{axis}"] = component / magnitude
        f["external_reactions/launch_rail/magnitude"] = magnitude

    def after_step(self):
        self._steps += 1
        if not self.on_rail or self.held:
            return
        f = self.fdm
        self._t += self.dt
        self._run_ft += f["velocities/u-fps"] * self.dt
        if self._run_ft <= 0.0 and not isinstance(self.launcher, Catapult):
            # slid back to the base: it rests there again
            self._run_ft = 0.0
            f["external_reactions/launch_rail/magnitude"] = 0.0
            f["forces/hold-down"] = 1
        elif self._run_ft >= self.launcher.length_m / FT:
            f["external_reactions/launch_rail/magnitude"] = 0.0
            for k, value in self._saved_integrators.items():
                f[k] = value
            self.on_rail = False
