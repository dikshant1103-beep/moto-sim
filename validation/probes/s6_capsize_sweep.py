#!/usr/bin/env python3
"""
S6 — Capsize-mode speed sweep before and after the steer-sign fix.

The G8 gate (tests/motorcycle/test_m2_validation.py) applies a 5 deg roll
perturbation at v = 0.5 m/s and requires the roll to grow past 5.8 deg within
1.5 s, i.e. a positive capsize eigenvalue at that speed.

Removing the K_Mz_missing spring changes the low-speed roll behaviour, so this
probe re-measures the whole capsize transition rather than the single gate
point: is the capsize mode still present (roll diverging at low speed, decaying
at higher speed), and where does the transition now sit?

The physical reference is that a single-track vehicle is unstable in capsize
below its capsize speed and stable above it. For the Meijaard benchmark bicycle
that speed is 6.024 m/s; a motorcycle with a rider-balance model stabilises far
lower, so the absolute value is not the point — the presence and ordering of the
transition is.

Usage:  python validation/probes/s6_capsize_sweep.py
Output: validation/probes/data/s6_capsize_sweep.json
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness import DATA, build  # noqa: E402

DT = 0.001
PERTURB = 5.0
SIM_S = 1.5
SPEEDS = [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.5, 2.0, 3.0]


def delta_roll(v: float, steer_kin_sign: float, K_Mz: float,
               K_yaw_damp: float) -> float:
    m = build(v0=v, steer_kin_sign=steer_kin_sign, K_Mz_missing=K_Mz,
              K_yaw_damp=K_yaw_damp, perturb_deg=PERTURB)
    n = round(SIM_S / DT)
    roll = 0.0
    for k in range(n):
        tel = m.step(k * DT)
        roll = tel["chassis"]["roll"]
    return math.degrees(abs(roll)) - PERTURB


def main() -> None:
    configs = [(+1.0, 2000.0, 2000.0,
                "pre-fix (sign=+1, K_Mz=2000, K_yaw=2000)"),
               (-1.0, 0.0, 0.0,
                "shipped (sign=-1, K_Mz=0,    K_yaw=0)")]
    out = {"perturb_deg": PERTURB, "sim_s": SIM_S, "gate_margin_deg": 0.8,
           "series": []}

    for sign, kmz, kyd, label in configs:
        deltas = []
        print(f"\n{label}")
        print(f"  {'v [m/s]':>8} {'d|roll| [deg]':>14}  verdict")
        for v in SPEEDS:
            d = delta_roll(v, sign, kmz, kyd)
            deltas.append(d)
            print(f"  {v:8.1f} {d:+14.3f}  "
                  f"{'CAPSIZE' if d > 0 else 'stable'}")
        # transition = first speed at which the roll stops growing
        trans = None
        for v, d in zip(SPEEDS, deltas):
            if d <= 0:
                trans = v
                break
        out["series"].append({"label": label, "steer_kin_sign": sign,
                              "K_Mz_missing": kmz, "K_yaw_damp": kyd,
                              "speeds": SPEEDS,
                              "delta_roll_deg": deltas,
                              "transition_speed_mps": trans})
        print(f"  -> capsize/stable transition at {trans} m/s")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s6_capsize_sweep.json"
    p.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
