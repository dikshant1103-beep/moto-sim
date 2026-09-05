#!/usr/bin/env python3
"""
S4 — Can the model corner at all?

Steady cornering, slalom and counter-steering manoeuvres are only meaningful if the model can develop a sustained yaw
rate. Two features of the current implementation put that in doubt:

  1. K_yaw_damp = 2000 N.m.s/rad is applied to the main frame about world Z.
     For comparison, the frame yaw inertia is Izz = 50 kg.m^2, so this is a
     yaw time constant of 40:1 against the vehicle's own dynamics.

  2. The roll supplement is

         tau_roll = -K_roll_total * sin(roll - lean_delta) - C_roll * roll_rate

     which contains NO steer-angle term. K_centripetal_eff is folded into
     K_roll_total, i.e. onto the DIAGONAL of the roll equation, so steer angle
     does not drive roll in the supplement at all.

This script measures the steady yaw rate and turn radius produced by a
commanded steer angle, across K_yaw_damp values, to quantify how much turning
authority exists.

Usage:  python validation/probes/s4_cornering_capability.py
Output: validation/probes/data/s4_cornering.json
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness import DATA, build, true_steer  # noqa: E402

DT = 0.001
V = 10.0
SETTLE_S = 0.5
HOLD_S = 4.0


def steady_turn(steer_target_deg: float, K_yaw_damp: float,
                steer_kin_sign: float, K_Mz: float) -> dict:
    m = build(v0=V, steer_kin_sign=steer_kin_sign, K_Mz_missing=K_Mz,
              K_yaw_damp=K_yaw_damp)
    for k in range(round(SETTLE_S / DT)):
        m.step(k * DT)

    # Command a steady steer angle through the model's own rate-limited
    # steer-target path (RiderInput.steer_target -> _steer_tgt_smooth).
    m.rider_source._input.steer_target = math.radians(steer_target_deg)

    n = round(HOLD_S / DT)
    yaw_rate = np.empty(n)
    roll = np.empty(n)
    steer = np.empty(n)
    for k in range(n):
        tel = m.step((round(SETTLE_S / DT) + k) * DT)
        yaw_rate[k] = m.main_frame.GetAngVelParent().z
        roll[k] = tel["chassis"]["roll"]
        steer[k] = true_steer(m)

    tail = slice(-1000, None)         # last 1 s
    yr = float(np.mean(yaw_rate[tail]))
    radius = V / yr if abs(yr) > 1e-4 else float("inf")
    # Kinematic (Ackermann-like) reference for a single-track vehicle
    d_true = float(np.mean(steer[tail]))
    yr_kin = V * math.tan(d_true) * math.cos(m.params.rake) / m.params.wheelbase
    return {
        "yaw_rate_dps": math.degrees(yr),
        "yaw_rate_kinematic_dps": math.degrees(yr_kin),
        "turn_radius_m": radius,
        "roll_deg": math.degrees(float(np.mean(roll[tail]))),
        "steer_true_deg": math.degrees(d_true),
    }


def main() -> None:
    out = {"config": {"v": V, "hold_s": HOLD_S, "dt": DT}, "cases": []}
    print(f"Steady-turn authority  v={V} m/s, commanded steer target = 3 deg\n")
    print(f"{'kin':<10} {'K_Mz':>5} {'K_yaw':>6} {'steer':>7} {'yaw_sim':>8} "
          f"{'yaw_kin':>8} {'radius':>9} {'roll':>7}")
    print("-" * 68)
    for sign, label in [(+1.0, "legacy"), (-1.0, "corrected")]:
        for K_Mz in (2000.0, 0.0):
            for kyd in (2000.0, 200.0, 0.0):
                try:
                    r = steady_turn(3.0, kyd, sign, K_Mz)
                except Exception as e:  # divergence
                    print(f"{label:<10} {K_Mz:5.0f} {kyd:6.0f}   diverged: {e}")
                    continue
                out["cases"].append({"kinematics": label, "K_Mz_missing": K_Mz,
                                     "K_yaw_damp": kyd, **r})
                rad = r["turn_radius_m"]
                rad_s = f"{rad:9.1f}" if math.isfinite(rad) else "      inf"
                print(f"{label:<10} {K_Mz:5.0f} {kyd:6.0f} "
                      f"{r['steer_true_deg']:+7.3f} {r['yaw_rate_dps']:+8.3f} "
                      f"{r['yaw_rate_kinematic_dps']:+8.3f} {rad_s} "
                      f"{r['roll_deg']:+7.3f}")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s4_cornering.json"
    p.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
