#!/usr/bin/env python3
"""
S1b — Sign audit of the steer axis and the applied aligning moment.

S1 found that a tyre aligning moment Mz injected at the front contact patch
reaches the steering degree of freedom at |eta| = 0.96, but with the OPPOSITE
sign to an equal torque applied directly at the steer motor.

That is either (a) a harmless bookkeeping difference between the motor's angle
convention and the world-Z convention, or (b) a genuine sign inversion which
would turn the tyre's SELF-aligning moment into an ANTI-aligning moment.

This script settles the question geometrically, without trusting any stated
convention:

  1. Measure the steer axis direction in world coordinates directly from the
     motor frame.
  2. Apply a known positive steer-motor torque and measure BOTH
     GetMotorAngle() and the true geometric heading of the front wheel
     (from the wheel body's spin-axis orientation), to fix the motor sign.
  3. Compute the true steer-axis component of the applied Mz wrench.
  4. Compare against the physically correct self-aligning direction: for a
     tyre rolling at slip angle alpha, Mz must rotate the wheel TOWARD zero
     slip angle.

Usage:  python validation/probes/s1b_sign_check.py
Output: validation/probes/data/s1b_sign_check.json
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
OUT = ROOT / "validation/probes/data/s1b_sign_check.json"
SETTLE_S = 0.60
V = 10.0


class _FixedTire:
    def __init__(self, inner, Fx=0.0, Fy=0.0, Mz=0.0):
        self._inner = inner
        self.Fx, self.Fy, self.Mz = Fx, Fy, Mz

    def compute_forces(self, *a, **k):
        return self.Fx, self.Fy, self.Mz

    def reset(self):
        return self._inner.reset()

    @property
    def alpha_prime(self):
        return 0.0

    @property
    def state(self):
        return self._inner.state


def _build_settled(v: float) -> Motorcycle:
    rider = ConstantInputSource(steer_torque=0.0, throttle=0.0,
                               brake_front=0.0, brake_rear=0.0)
    m = Motorcycle.from_yaml(BIKE, TIRE, TIRE, FlatPlane(), rider, dt=DT)
    m.build()
    m.reset(v0=v)
    for k in range(round(SETTLE_S / DT)):
        m.step(k * DT)
    return m


def _geometric_steer(m: Motorcycle) -> float:
    """True steer angle from body orientations, independent of motor convention.

    Front-wheel spin axis is the wheel body's local +Y. Its projection on the
    ground plane is perpendicular to the wheel's heading, so the wheel heading
    yaw is atan2 of the spin axis rotated by -90 deg. We compare that heading
    against the main frame's yaw; the difference is the geometric steer angle
    (positive = wheel points LEFT of the chassis).
    """
    ax = m.front_wheel.GetRot().Rotate(chrono.ChVector3d(0.0, 1.0, 0.0))
    # heading = spin axis rotated -90 deg about world Z
    hx, hy = ax.y, -ax.x
    wheel_yaw = math.atan2(hy, hx)

    q = m.main_frame.GetRot()
    frame_yaw = math.atan2(2 * (q.e0 * q.e3 + q.e1 * q.e2),
                           1 - 2 * (q.e2 * q.e2 + q.e3 * q.e3))
    d = wheel_yaw - frame_yaw
    return math.atan2(math.sin(d), math.cos(d))


def _run(m: Motorcycle, secs: float, t0: float,
         direct_steer_tau=0.0, tire_Mz=0.0):
    m._whipple.torques = lambda **kw: (direct_steer_tau, 0.0)
    m.C_steer = 0.0
    m.K_yaw_damp = 0.0
    m.K_pitch_damp = 0.0
    m.front_tire = _FixedTire(m.front_tire, 0.0, 0.0, tire_Mz)
    m.rear_tire = _FixedTire(m.rear_tire)
    n = round(secs / DT)
    motor, geo = np.empty(n), np.empty(n)
    for k in range(n):
        m.step(t0 + k * DT)
        motor[k] = m._steer_motor.GetMotorAngle()
        geo[k] = _geometric_steer(m)
    return motor, geo


def main() -> None:
    out = {}

    # ── 1. Steer axis direction in world coordinates ─────────────────────
    m = _build_settled(V)
    p = m.params
    lf = m._steer_motor.GetFrame2Abs() if hasattr(m._steer_motor, "GetFrame2Abs") \
        else m._steer_motor.GetVisualModelFrame()
    axis = lf.GetRotMat() * chrono.ChVector3d(0.0, 0.0, 1.0)
    out["steer_axis_world"] = [axis.x, axis.y, axis.z]
    out["steer_axis_dot_worldZ"] = axis.z
    print(f"steer axis (world) = ({axis.x:+.4f}, {axis.y:+.4f}, {axis.z:+.4f})")
    print(f"  axis . worldZ = {axis.z:+.4f}   (cos rake = {math.cos(p.rake):.4f})")

    # ── 2. Motor-angle sign vs true geometric steer ──────────────────────
    m = _build_settled(V)
    motor, geo = _run(m, 0.20, SETTLE_S, direct_steer_tau=+10.0)
    out["pos_motor_torque"] = {
        "motor_angle_end_deg": math.degrees(motor[-1]),
        "geometric_steer_end_deg": math.degrees(geo[-1]),
    }
    print(f"\n+10 N.m direct steer-motor torque for 0.2 s:")
    print(f"  GetMotorAngle()      = {math.degrees(motor[-1]):+.4f} deg")
    print(f"  geometric steer      = {math.degrees(geo[-1]):+.4f} deg "
          f"(+ = wheel points LEFT)")
    same = (motor[-1] * geo[-1]) > 0
    out["motor_angle_matches_geometry"] = bool(same)
    print(f"  motor sign matches geometry: {same}")

    # ── 3. Injected world-Z Mz -> which way does the wheel actually turn? ─
    m = _build_settled(V)
    motor2, geo2 = _run(m, 0.20, SETTLE_S, tire_Mz=+10.0)
    out["pos_tire_Mz"] = {
        "motor_angle_end_deg": math.degrees(motor2[-1]),
        "geometric_steer_end_deg": math.degrees(geo2[-1]),
    }
    print(f"\n+10 N.m tyre Mz about world +Z for 0.2 s:")
    print(f"  GetMotorAngle()      = {math.degrees(motor2[-1]):+.4f} deg")
    print(f"  geometric steer      = {math.degrees(geo2[-1]):+.4f} deg")

    # A positive moment about world +Z is a left (counter-clockwise, viewed
    # from above) yawing moment, so it MUST turn the wheel to the left.
    correct = geo2[-1] > 0
    out["worldZ_Mz_turns_wheel_left"] = bool(correct)
    print(f"  +Z moment turns wheel LEFT (physically required): {correct}")

    # ── 4. Pacejka Mz sign convention check at a known slip angle ────────
    # Self-aligning: at positive slip angle alpha, Mz must act to REDUCE alpha.
    from core.tire.tire_interface import TireModel
    tm = TireModel.from_yaml(TIRE)
    tm.reset()
    Fz = 900.0
    rows = []
    for alpha_deg in (1.0, 2.0, 4.0, 6.0):
        tm.reset()
        a = math.radians(alpha_deg)
        for _ in range(400):          # let relaxation settle
            Fx, Fy, Mz = tm.compute_forces(a, 0.0, Fz, 0.0, 10.0, DT)
        rows.append({"alpha_deg": alpha_deg, "Fy": Fy, "Mz": Mz})
        print(f"  alpha={alpha_deg:+.1f} deg -> Fy={Fy:+9.2f} N   Mz={Mz:+8.3f} N.m")
    out["pacejka_sweep"] = rows
    out["Mz_opposes_alpha"] = bool(rows[0]["Mz"] * rows[0]["alpha_deg"] < 0)
    print(f"\n  Pacejka Mz opposes slip angle (self-aligning): "
          f"{out['Mz_opposes_alpha']}")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
