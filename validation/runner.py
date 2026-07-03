"""runner.py — drive the real physics bike over a test road and record signals.

In-process, headless, fast (no ROS/Gazebo/WS). Builds a Motorcycle with the given
test road, holds a target speed straight-line, and records the engineering signals a
validation engineer needs:

  vertical accel of the sprung mass (frame)  → ISO 2631 ride comfort/health
  fork / shock travel                        → suspension bottoming check
  front / rear tyre vertical load  Fz        → peak impact load (DAF) → fatigue
  roll, lateral pos                          → cross-slope stability response

Straight-line (no steering) — exactly the regime the model is stable in, and exactly
how these durability/ride road tests are run.
"""

from __future__ import annotations

import math

from core.chassis.motorcycle import Motorcycle, BikeParams
from core.rider.rider_input import RiderInput, RiderInputSource
from core.tire.tire_interface import TireModel

DT = 0.001

# Suspension travel available (m) — bottoming limit. CBR-class defaults.
FORK_TRAVEL_MAX = 0.120
SHOCK_TRAVEL_MAX = 0.130

_BIKE = {
    "wheelbase": 1.37, "rake": math.radians(24.0), "trail": 0.097,
    "front_wheel_radius": 0.299, "rear_wheel_radius": 0.318,
    "total_mass": 189.0, "cg_height": 0.58, "cg_x": 0.63,
    "front_spring_k": 18500.0, "front_damper_c": 1400.0,
    "rear_spring_k": 24000.0, "rear_damper_c": 2200.0,
}


class _Hold(RiderInputSource):
    """Hold target speed, ride straight (no steer/lean)."""
    def __init__(self):
        self.thr = 0.0
    def get(self, t):
        return RiderInput(throttle=self.thr).clamp()


def _tire():
    return TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")


def run_test(road, target_speed=8.0, distance=40.0, settle=2.0, bike=None):
    """Run one road test. Returns a dict of recorded signals + summary.

    distance: metres to travel past the start before stopping.
    settle:   seconds of straight cruise before metrics window opens (reach speed).
    """
    rider = _Hold()
    p = dict(_BIKE); p.update(bike or {})
    moto = Motorcycle(BikeParams(p), _tire(), _tire(), road, rider, dt=DT)
    moto.profile_ground = road       # build a bumpy collision ground from this profile
    moto.build()
    moto.reset(v0=target_speed)

    Wf0 = p["total_mass"] * 9.81 * (1 - p["cg_x"] / p["wheelbase"])   # static front load (approx)
    Wr0 = p["total_mass"] * 9.81 * (p["cg_x"] / p["wheelbase"])

    t = 0.0
    vz_prev = 0.0
    rec = {k: [] for k in ("t", "x", "az", "fork", "shock", "Fz_f", "Fz_r", "roll", "vx", "y")}
    diverged = False
    max_steps = int((settle + distance / max(target_speed, 1.0) + 4.0) / DT)

    for k in range(max_steps):
        t = k * DT
        telem = moto.step(t)
        c = telem["chassis"]
        if not (math.isfinite(c["roll"]) and math.isfinite(c["vx"])):
            diverged = True
            break
        # hold speed
        rider.thr = max(0.0, min(0.6, 0.12 + 0.18 * (target_speed - c["vx"])))

        # sprung-mass vertical accel (finite-diff of frame vertical velocity)
        vz = moto.main_frame.GetPosDt().z
        az = (vz - vz_prev) / DT
        vz_prev = vz
        fx = moto.main_frame.GetPos().x

        if t > settle:
            rec["t"].append(t - settle)
            rec["x"].append(fx)
            rec["az"].append(az)
            rec["fork"].append(telem["suspension"]["fork_travel"])
            rec["shock"].append(telem["suspension"]["rear_travel"])
            rec["Fz_f"].append(telem["front"]["Fz"])
            rec["Fz_r"].append(telem["rear"]["Fz"])
            rec["roll"].append(c["roll"])
            rec["vx"].append(c["vx"])
            rec["y"].append(moto.main_frame.GetPos().y)
            if fx > rec["x"][0] + distance:
                break

    return {
        "diverged": diverged,
        "signals": rec,
        "static": {"Fz_f": Wf0, "Fz_r": Wr0,
                   "fork_max": FORK_TRAVEL_MAX, "shock_max": SHOCK_TRAVEL_MAX},
        "target_speed": target_speed,
    }
