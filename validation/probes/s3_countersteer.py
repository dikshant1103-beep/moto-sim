#!/usr/bin/env python3
"""
S3 — Countersteering: an independent physical check of the steer sign.

Countersteering is the defining, non-negotiable signature of single-track
vehicle dynamics: above a few m/s, a steer torque applied to the RIGHT makes
the machine first steer right, then roll LEFT and turn LEFT (Cossalter ch. 6;
Sharp 1971). Any model that turns the same way it is steered at speed is
kinematically wrong.

This is a sign check that does not depend on any convention internal to the
simulator, so it independently arbitrates between

    steer_kin_sign = +1  (legacy front-tyre kinematics)
    steer_kin_sign = -1  (corrected, per studies S1b/S1c)

A short steer-torque pulse is applied from steady straight running and the
initial steer direction, the resulting roll direction, and the resulting yaw
direction are recorded.

Usage:  python validation/probes/s3_countersteer.py
Output: validation/probes/data/s3_countersteer.json
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
V = 12.0
PULSE_NM = 25.0      # steer torque amplitude
PULSE_S = 0.30       # pulse duration
SETTLE_S = 0.50
TOTAL_S = 4.0


def run_pulse(steer_kin_sign: float, K_Mz: float, torque: float) -> dict:
    m = build(v0=V, steer_kin_sign=steer_kin_sign, K_Mz_missing=K_Mz,
              K_yaw_damp=2000.0)   # pinned: pre-fix value
    n_settle = round(SETTLE_S / DT)
    for k in range(n_settle):
        m.step(k * DT)

    n = round(TOTAL_S / DT)
    n_pulse = round(PULSE_S / DT)
    t = np.arange(n) * DT
    roll = np.empty(n)
    steer = np.empty(n)
    yaw_rate = np.empty(n)
    for k in range(n):
        m.rider_source._input.steer_torque = torque if k < n_pulse else 0.0
        tel = m.step((n_settle + k) * DT)
        roll[k] = tel["chassis"]["roll"]
        steer[k] = true_steer(m)
        yaw_rate[k] = m.main_frame.GetAngVelParent().z

    i_pulse_end = n_pulse
    return {
        "t": t, "roll": roll, "steer": steer, "yaw_rate": yaw_rate,
        # initial steer response, sampled mid-pulse
        "steer_initial_deg": math.degrees(float(steer[n_pulse // 2])),
        "roll_at_pulse_end_deg": math.degrees(float(roll[i_pulse_end])),
        "roll_final_deg": math.degrees(float(roll[-1])),
        "yaw_rate_final_dps": math.degrees(float(yaw_rate[-500:].mean())),
    }


def classify(r: dict) -> str:
    s0 = r["steer_initial_deg"]
    yr = r["yaw_rate_final_dps"]
    if abs(s0) < 0.05 or abs(yr) < 0.5:
        return "no clear response"
    return "COUNTERSTEER (correct)" if s0 * yr < 0 else "direct steer (wrong at speed)"


def main() -> None:
    out = {"config": {"v": V, "pulse_Nm": PULSE_NM, "pulse_s": PULSE_S,
                      "total_s": TOTAL_S, "dt": DT}, "cases": []}
    print(f"Countersteer test  v={V} m/s  pulse={PULSE_NM} N.m for {PULSE_S}s\n")
    print(f"{'kinematics':<12} {'K_Mz':>6} {'tau':>6} {'steer0':>8} "
          f"{'roll_end':>9} {'yaw_rate':>9}  verdict")
    print("-" * 74)

    for sign, label in [(+1.0, "legacy"), (-1.0, "corrected")]:
        for K_Mz in (2000.0, 0.0):
            for tau in (+PULSE_NM, -PULSE_NM):
                r = run_pulse(sign, K_Mz, tau)
                v = classify(r)
                rec = {"kinematics": label, "steer_kin_sign": sign,
                       "K_Mz_missing": K_Mz, "steer_torque_Nm": tau,
                       "steer_initial_deg": r["steer_initial_deg"],
                       "roll_at_pulse_end_deg": r["roll_at_pulse_end_deg"],
                       "roll_final_deg": r["roll_final_deg"],
                       "yaw_rate_final_dps": r["yaw_rate_final_dps"],
                       "verdict": v}
                out["cases"].append(rec)
                print(f"{label:<12} {K_Mz:6.0f} {tau:+6.0f} "
                      f"{r['steer_initial_deg']:+8.3f} "
                      f"{r['roll_at_pulse_end_deg']:+9.3f} "
                      f"{r['yaw_rate_final_dps']:+9.3f}  {v}")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s3_countersteer.json"
    p.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
