#!/usr/bin/env python3
"""
S1 — Force-routing probe (audits the two claimed force-routing failures).

It has been claimed that PyChrono's constraint chain fails to route two force paths:

  Failure A — the Pacejka aligning moment Mz does not reach the steering axis.
  Failure B — the front-tyre lateral force Fy does not produce a roll moment
              on the main frame.

Both claims were previously supported only by aggregate simulation behaviour.
This script tests each claim directly by *transmission ratio*: inject a known
generalised force at the tyre contact patch, measure the response of the
target degree of freedom, and divide by the response obtained when the SAME
generalised force is applied directly to that degree of freedom.

    eta = (q_injected - q_null) / (q_reference - q_null)

    eta ~ 1  ->  the path IS routed by the constraint chain
    eta ~ 0  ->  the path is NOT routed (the claimed failure)

Every case shares the identical settled initial state, timestepper, and
supplement configuration; only the injected generalised force differs, so the
ratio is free of any tuning parameter.

Usage:  python validation/probes/s1_routing_probe.py
Output: validation/probes/data/s1_routing_probe.json
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
OUT = ROOT / "validation/probes/data/s1_routing_probe.json"

SETTLE_S = 0.60      # let contact/suspension transients die out
PROBE_S = 0.050      # short window: measures the *instantaneous* transmission
V_PROBE = 10.0       # m/s


def _build(v0: float) -> Motorcycle:
    rider = ConstantInputSource(steer_torque=0.0, throttle=0.0,
                               brake_front=0.0, brake_rear=0.0)
    m = Motorcycle.from_yaml(BIKE, TIRE, TIRE, FlatPlane(), rider, dt=DT)
    m.build()
    m.reset(v0=v0)
    return m


class _FixedTire:
    """Replaces a TireModel with a constant (Fx, Fy, Mz) source.

    Keeps the exact call signature and the `alpha_prime`/`state` attributes the
    telemetry block reads, so the injected wrench travels through exactly the
    same `_apply_tire_force` code path as a genuine Pacejka output.
    """

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


def _run_case(v: float, *, tire_Fx=0.0, tire_Fy=0.0, tire_Mz=0.0,
              direct_steer_tau=0.0, direct_roll_tau=0.0) -> dict:
    """Settle, then inject for PROBE_S with the analytical supplement OFF."""
    m = _build(v)

    n_settle = round(SETTLE_S / DT)
    for k in range(n_settle):
        m.step(k * DT)

    # ── Freeze every discretionary torque so only the injected one acts ──
    #    tau_steer_w and tau_roll_w are added straight onto the steer motor and
    #    the main-frame body respectively, so returning them here IS the
    #    "applied directly to the DOF" reference case.
    m._whipple.torques = lambda **kw: (direct_steer_tau, direct_roll_tau)
    m.C_steer = 0.0
    m.K_yaw_damp = 0.0
    m.K_pitch_damp = 0.0
    m.front_tire = _FixedTire(m.front_tire, tire_Fx, tire_Fy, tire_Mz)
    m.rear_tire = _FixedTire(m.rear_tire, 0.0, 0.0, 0.0)

    n_probe = round(PROBE_S / DT)
    t = np.arange(n_probe) * DT
    steer = np.empty(n_probe)
    roll = np.empty(n_probe)
    for k in range(n_probe):
        tel = m.step((n_settle + k) * DT)
        steer[k] = m._steer_motor.GetMotorAngle()
        roll[k] = tel["chassis"]["roll"]
    return {"t": t, "steer": steer, "roll": roll}


def _quad_accel(t: np.ndarray, q: np.ndarray) -> float:
    """Second derivative from a quadratic least-squares fit q = a/2 t^2 + b t + c."""
    c2, _c1, _c0 = np.polyfit(t, q, 2)
    return 2.0 * c2


def main() -> None:
    p = _build(0.0).params
    cos_rake = math.cos(p.rake)
    h = p.cg_height

    M0 = 50.0    # N·m aligning moment injected at the front contact patch
    F0 = 500.0   # N  lateral force injected at the front contact patch

    print(f"Force-routing probe  v={V_PROBE} m/s  dt={DT*1e3:.0f} ms  "
          f"window={PROBE_S*1e3:.0f} ms")
    print(f"  rake={math.degrees(p.rake):.1f} deg  cos(rake)={cos_rake:.4f}  "
          f"h_cg={h:.3f} m")

    # ── Failure A: does tyre Mz reach the steering axis? ──────────────────
    print("\n[A] aligning moment Mz -> steer axis")
    null = _run_case(V_PROBE)
    inj_a = _run_case(V_PROBE, tire_Mz=M0)
    # Reference: the same moment resolved onto the steer axis, applied
    # directly as a steer-motor torque.
    ref_a = _run_case(V_PROBE, direct_steer_tau=M0 * cos_rake)

    acc_null_s = _quad_accel(null["t"], null["steer"])
    acc_inj_s = _quad_accel(inj_a["t"], inj_a["steer"])
    acc_ref_s = _quad_accel(ref_a["t"], ref_a["steer"])
    eta_A = (acc_inj_s - acc_null_s) / (acc_ref_s - acc_null_s)
    print(f"  steer ang. accel [rad/s^2]  null={acc_null_s:+.4f}  "
          f"injected={acc_inj_s:+.4f}  reference={acc_ref_s:+.4f}")
    print(f"  --> transmission ratio eta_A = {eta_A:.4f}")

    # ── Failure B: does front tyre Fy produce main-frame roll? ────────────
    print("\n[B] lateral force Fy -> main-frame roll")
    inj_b = _run_case(V_PROBE, tire_Fy=F0)
    # Reference: the same force times its CG moment arm, applied directly as
    # a roll torque on the main frame.
    ref_b = _run_case(V_PROBE, direct_roll_tau=F0 * h)

    acc_null_r = _quad_accel(null["t"], null["roll"])
    acc_inj_r = _quad_accel(inj_b["t"], inj_b["roll"])
    acc_ref_r = _quad_accel(ref_b["t"], ref_b["roll"])
    eta_B = (acc_inj_r - acc_null_r) / (acc_ref_r - acc_null_r)
    print(f"  roll ang. accel [rad/s^2]   null={acc_null_r:+.4f}  "
          f"injected={acc_inj_r:+.4f}  reference={acc_ref_r:+.4f}")
    print(f"  --> transmission ratio eta_B = {eta_B:.4f}")

    result = {
        "config": {
            "v": V_PROBE, "dt": DT, "settle_s": SETTLE_S, "probe_s": PROBE_S,
            "M0_Nm": M0, "F0_N": F0,
            "rake_deg": math.degrees(p.rake), "cos_rake": cos_rake,
            "cg_height_m": h, "trail_m": p.trail,
        },
        "failure_A": {
            "steer_accel_null": acc_null_s,
            "steer_accel_injected": acc_inj_s,
            "steer_accel_reference": acc_ref_s,
            "eta": eta_A,
        },
        "failure_B": {
            "roll_accel_null": acc_null_r,
            "roll_accel_injected": acc_inj_r,
            "roll_accel_reference": acc_ref_r,
            "eta": eta_B,
        },
        "traces": {
            k: {"t": v["t"].tolist(), "steer": v["steer"].tolist(),
                "roll": v["roll"].tolist()}
            for k, v in {"null": null, "injA": inj_a, "refA": ref_a,
                         "injB": inj_b, "refB": ref_b}.items()
        },
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"\nWrote {OUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
