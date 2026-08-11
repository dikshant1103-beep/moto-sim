#!/usr/bin/env python3
"""
S1c — Direct audit of the front-wheel heading used for tyre slip.

motorcycle.py computes the heading handed to the front tyre model as

    _psi_front = yaw + _steer_ang_raw * cos(rake)          (line ~805)

where `_steer_ang_raw = self._steer_motor.GetMotorAngle()`.

S1b showed GetMotorAngle() carries the opposite sign to the true geometric
steer angle. This script logs, step by step in an ordinary unmodified run,

    psi_front_code  = yaw + GetMotorAngle() * cos(rake)     (what the tyre sees)
    psi_front_true  = actual world heading of the front wheel (from its
                      spin-axis orientation)

and reports their correlation. If psi_front_code tracks the NEGATIVE of
psi_front_true, the front tyre is being evaluated at a slip angle derived from
a steer angle of the wrong sign, and its lateral force will act to amplify,
rather than oppose, a steer perturbation.

Usage:  python validation/probes/s1c_psi_front_audit.py
Output: validation/probes/data/s1c_psi_front_audit.json
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import pychrono as chrono  # noqa: E402

from core.chassis.motorcycle import Motorcycle  # noqa: E402
from core.rider.rider_input import ConstantInputSource  # noqa: E402
from core.road.flat_plane import FlatPlane  # noqa: E402

BIKE = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
TIRE = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001
OUT = ROOT / "validation/probes/data/s1c_psi_front_audit.json"


def _yaw(q) -> float:
    return math.atan2(2 * (q.e0 * q.e3 + q.e1 * q.e2),
                      1 - 2 * (q.e2 * q.e2 + q.e3 * q.e3))


def _true_wheel_yaw(m: Motorcycle) -> float:
    ax = m.front_wheel.GetRot().Rotate(chrono.ChVector3d(0.0, 1.0, 0.0))
    return math.atan2(-ax.x, ax.y)   # heading = spin_axis x zhat


def main() -> None:
    rider = ConstantInputSource(steer_torque=0.0, throttle=0.0,
                               brake_front=0.0, brake_rear=0.0)
    m = Motorcycle.from_yaml(BIKE, TIRE, TIRE, FlatPlane(), rider, dt=DT)
    m.build()
    m.reset(v0=6.0)
    cos_rake = math.cos(m.params.rake)

    # 5 deg roll perturbation -> the standard weave excitation
    m.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))

    n = 3000
    code_steer, true_steer = [], []
    for k in range(n):
        m.step(k * DT)
        motor = m._steer_motor.GetMotorAngle()
        frame_yaw = _yaw(m.main_frame.GetRot())

        psi_code = frame_yaw + motor * cos_rake         # exactly line ~805
        psi_true = _true_wheel_yaw(m)

        d_code = psi_code - frame_yaw
        d_true = math.atan2(math.sin(psi_true - frame_yaw),
                            math.cos(psi_true - frame_yaw))
        code_steer.append(d_code)
        true_steer.append(d_true)

    a = np.array(code_steer)
    b = np.array(true_steer)
    mask = np.abs(b) > math.radians(0.01)      # ignore numerical zero
    ratio = float(np.median(a[mask] / b[mask])) if mask.any() else float("nan")
    corr = float(np.corrcoef(a, b)[0, 1])

    print("Front-wheel heading audit (v=6 m/s, 5 deg roll perturbation, 3 s)")
    print(f"  samples with non-zero steer : {int(mask.sum())} / {n}")
    print(f"  median  psi_code / psi_true : {ratio:+.4f}")
    print(f"  Pearson correlation         : {corr:+.4f}")
    print(f"  max |steer_code| [deg]      : {math.degrees(np.abs(a).max()):.3f}")
    print(f"  max |steer_true| [deg]      : {math.degrees(np.abs(b).max()):.3f}")
    verdict = ("INVERTED — the tyre sees the opposite steer sign"
               if ratio < -0.5 else
               "consistent" if ratio > 0.5 else "inconclusive")
    print(f"  VERDICT: {verdict}")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({
        "n_steps": n, "dt": DT, "v": 6.0, "perturb_deg": 5.0,
        "median_ratio_code_over_true": ratio,
        "pearson_corr": corr,
        "max_steer_code_deg": math.degrees(float(np.abs(a).max())),
        "max_steer_true_deg": math.degrees(float(np.abs(b).max())),
        "verdict": verdict,
        "steer_code_rad": a.tolist(),
        "steer_true_rad": b.tolist(),
    }, indent=2))
    print(f"\nWrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
