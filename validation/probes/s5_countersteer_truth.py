#!/usr/bin/env python3
"""
S5 — Which sign convention actually countersteers?

The G6 gate (tests/motorcycle/test_m1_validation.py) applies +2 N.m of rider
steer torque at 10 m/s and asserts yaw_rate < -0.05 deg/s, on the stated
reasoning:

    "+torque steers wheel right -> gyroscopic reaction rolls frame left
     -> left cornering (yaw_rate < 0 in SAE/PyChrono Z-up convention where
     +Z is up and left turn = negative yaw rate when X is forward and Y is
     left)."

Two things in that sentence need checking, because the frame is right-handed
with X forward, Y LEFT and Z up:

  (i)  In such a frame a positive rotation about +Z carries X toward Y, i.e.
       toward the left. So a LEFT turn is a POSITIVE yaw rate, not negative.
  (ii) The claim that rider +2 N.m steers the wheel right depends on the
       negation in motorcycle.py (`_tau = -inp.steer_torque + ...`).

This probe measures, geometrically and without trusting any convention:

  * the direction the front wheel physically points during the torque pulse,
  * the direction the chassis rolls,
  * the direction the chassis ends up yawing,

for both steer_kin_sign settings. Genuine countersteering requires the initial
steer direction and the final yaw direction to have OPPOSITE signs.

Usage:  python validation/probes/s5_countersteer_truth.py
Output: validation/probes/data/s5_countersteer_truth.json
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
TAU = 2.0        # N.m rider steer torque, as in the G6 gate
APPLY_S = 0.5    # gate's application window
TOTAL_S = 3.0    # keep watching after the pulse


def run(steer_kin_sign: float, K_Mz: float, K_yaw_damp: float) -> dict:
    m = build(v0=V, steer_kin_sign=steer_kin_sign, K_Mz_missing=K_Mz,
              K_yaw_damp=K_yaw_damp)
    n = round(TOTAL_S / DT)
    n_apply = round(APPLY_S / DT)
    steer = np.empty(n)
    roll = np.empty(n)
    yaw_rate = np.empty(n)
    for k in range(n):
        m.rider_source._input.steer_torque = TAU if k < n_apply else 0.0
        tel = m.step(k * DT)
        steer[k] = true_steer(m)
        roll[k] = tel["chassis"]["roll"]
        yaw_rate[k] = tel["chassis"]["yaw_rate"]

    i_early = round(0.05 / DT)          # while the wheel is first deflecting
    return {
        "steer_early_deg": math.degrees(float(steer[i_early])),
        "steer_at_gate_deg": math.degrees(float(steer[n_apply - 1])),
        "roll_at_gate_deg": math.degrees(float(roll[n_apply - 1])),
        "yaw_rate_at_gate_dps": math.degrees(float(yaw_rate[n_apply - 1])),
        "yaw_rate_final_dps": math.degrees(float(yaw_rate[-500:].mean())),
        "roll_final_deg": math.degrees(float(roll[-1])),
    }


def main() -> None:
    print(f"G6 countersteer audit: rider +{TAU} N.m for {APPLY_S}s at {V} m/s")
    print("sign convention: +steer = wheel points LEFT, "
          "+yaw_rate = turning LEFT (right-handed, X fwd / Y left / Z up)\n")
    print(f"{'kin':<10} {'K_Mz':>5} {'K_yaw':>6} {'steer_0':>8} {'steer_g':>8} "
          f"{'roll_g':>8} {'yaw_g':>8} {'yaw_f':>8}  countersteer?")
    print("-" * 88)

    rows = []
    for sign, label in [(+1.0, "pre-fix"), (-1.0, "fixed")]:
        for K_Mz in (2000.0, 0.0):
            for kyd in (2000.0, 0.0):
                r = run(sign, K_Mz, kyd)
                s0 = r["steer_early_deg"]
                yf = r["yaw_rate_final_dps"]
                if abs(s0) < 1e-3 or abs(yf) < 1e-3:
                    cs = "no response"
                else:
                    cs = "YES" if s0 * yf < 0 else "no (turns as steered)"
                rows.append({"kinematics": label, "steer_kin_sign": sign,
                             "K_Mz_missing": K_Mz, "K_yaw_damp": kyd,
                             "countersteer": cs, **r})
                print(f"{label:<10} {K_Mz:5.0f} {kyd:6.0f} {s0:+8.3f} "
                      f"{r['steer_at_gate_deg']:+8.3f} "
                      f"{r['roll_at_gate_deg']:+8.3f} "
                      f"{r['yaw_rate_at_gate_dps']:+8.3f} "
                      f"{yf:+8.3f}  {cs}")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s5_countersteer_truth.json"
    p.write_text(json.dumps({"tau_Nm": TAU, "apply_s": APPLY_S, "v": V,
                             "rows": rows}, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
