"""
Shared simulation harness for the validation probes.

One place to build a configured motorcycle and extract the standard response
metrics, so every probe in this directory reports comparable numbers.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pychrono as chrono  # noqa: E402

from core.chassis.motorcycle import Motorcycle  # noqa: E402
from core.rider.rider_input import ConstantInputSource  # noqa: E402
from core.road.flat_plane import FlatPlane  # noqa: E402

BIKE_DIR = ROOT / "core/chassis/bike_params"
TIRE = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DATA = ROOT / "validation/probes/data"
FIGS = ROOT / "validation/probes/figs"
DT_DEFAULT = 0.001


def build(bike: str = "cbr600rr", v0: float = 6.0, dt: float = DT_DEFAULT,
          *, steer_kin_sign: float | None = None,
          K_Mz_missing: float | None = None,
          centripetal_factor: float | None = None,
          C_roll: float | None = None,
          K_yaw_damp: float | None = None, C_steer: float | None = None,
          perturb_deg: float = 0.0) -> Motorcycle:
    """Build, reset and optionally roll-perturb a motorcycle.

    Every tuning argument defaults to None, meaning "leave the model's own
    default alone". Probes that compare pre-fix against post-fix behaviour must
    pass the values they want EXPLICITLY — otherwise this harness would silently
    pin the model to whatever was current when the harness was written, which is
    exactly the trap that made a verification run report the wrong configuration.
    """
    yaml_path = bike if str(bike).endswith(".yaml") else BIKE_DIR / f"{bike}.yaml"
    rider = ConstantInputSource(steer_torque=0.0, throttle=0.0,
                               brake_front=0.0, brake_rear=0.0)
    m = Motorcycle.from_yaml(yaml_path, TIRE, TIRE, FlatPlane(), rider, dt=dt)
    m.build()
    m.reset(v0=v0)

    if steer_kin_sign is not None:
        m.steer_kin_sign = steer_kin_sign
    if K_Mz_missing is not None:
        m._whipple.K_Mz_missing = K_Mz_missing
    if centripetal_factor is not None:
        m._whipple.centripetal_factor = centripetal_factor
    if C_roll is not None:
        m._whipple.C_roll = C_roll
    if K_yaw_damp is not None:
        m.K_yaw_damp = K_yaw_damp
    if C_steer is not None:
        m.C_steer = C_steer

    if perturb_deg:
        m.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(perturb_deg)))
    return m


def true_steer(m: Motorcycle) -> float:
    """Geometric steer angle [rad], independent of the motor sign convention.

    Positive = front wheel points LEFT of the chassis.
    """
    ax = m.front_wheel.GetRot().Rotate(chrono.ChVector3d(0.0, 1.0, 0.0))
    wheel_yaw = math.atan2(-ax.x, ax.y)
    q = m.main_frame.GetRot()
    frame_yaw = math.atan2(2 * (q.e0 * q.e3 + q.e1 * q.e2),
                           1 - 2 * (q.e2 * q.e2 + q.e3 * q.e3))
    d = wheel_yaw - frame_yaw
    return math.atan2(math.sin(d), math.cos(d))


def run(m: Motorcycle, secs: float, dt: float = DT_DEFAULT) -> dict:
    """Run and return time histories of the standard channels."""
    n = round(secs / dt)
    t = np.arange(n) * dt
    roll = np.empty(n)
    steer = np.empty(n)
    yawrate = np.empty(n)
    speed = np.empty(n)
    for k in range(n):
        tel = m.step(k * dt)
        roll[k] = tel["chassis"]["roll"]
        steer[k] = true_steer(m)
        yawrate[k] = m.main_frame.GetAngVelParent().z
        speed[k] = tel["chassis"].get("speed", float("nan"))
    return {"t": t, "roll": roll, "steer": steer,
            "yawrate": yawrate, "speed": speed}


def sign_reversals(x: np.ndarray, deadband: float = 0.0) -> int:
    """Count sign changes of x, ignoring excursions inside +/- deadband."""
    s = x.copy()
    if deadband > 0:
        s = s[np.abs(s) > deadband]
    if s.size < 2:
        return 0
    return int(np.count_nonzero(np.diff(np.sign(s)) != 0))


def metrics(tr: dict) -> dict:
    """Standard scalar summary of a run."""
    roll_deg = np.degrees(tr["roll"])
    steer_deg = np.degrees(tr["steer"])
    finite = np.isfinite(roll_deg).all() and np.abs(roll_deg).max() < 89.0
    return {
        "stable": bool(finite),
        "max_abs_roll_deg": float(np.abs(roll_deg).max()),
        "final_abs_roll_deg": float(abs(roll_deg[-1])),
        "max_abs_steer_deg": float(np.abs(steer_deg).max()),
        "final_abs_steer_deg": float(abs(steer_deg[-1])),
        "roll_reversals": sign_reversals(roll_deg, deadband=0.05),
        "mean_abs_yawrate_dps": float(np.degrees(np.abs(tr["yawrate"])).mean()),
    }
