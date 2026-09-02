#!/usr/bin/env python3
"""
S7 — How much artificial yaw damping does the model actually need?

`Motorcycle.K_yaw_damp` is a torque -K*omega_z applied to the main frame about
world Z. At 2000 N.m.s/rad against a frame yaw inertia of Izz = 50 kg.m^2 it
dominates the yaw equation, and s4_cornering_capability.py showed it degrades
steady-turn agreement with the single-track kinematic law from +0.5 % to -65 %.

Its stated justification was:

    "the model lacks inherent yaw stability (no physical trail self-alignment
     in the constraint model). Numerical noise grows into a yaw instability
     with eigenvalue ~14 rad/s; need K > I_yaw*14 ~ 800."

That premise is the same one disproved in STEER_SIGN_AUDIT.md: trail
self-alignment IS routed to the steer axis (|eta| = 0.96). The observed yaw
instability is far more likely to have been the front-tyre steer-sign inversion
acting as positive feedback, in which case the damper is a second symptom of the
same bug and can now be reduced or removed.

This probe tests both halves of that:

  A. STABILITY — long straight-line runs (30 s), no rider input, measuring
     whether yaw rate grows. Runs an exponential fit on |omega_z| to detect a
     positive yaw eigenvalue, at several speeds, for both sign conventions.
  B. FIDELITY — steady-turn yaw rate versus v*tan(delta)*cos(rake)/L.

A correct model should be yaw-stable with K_yaw_damp = 0 and should match the
kinematic law.

Usage:  python validation/probes/s7_yaw_damping.py
Output: validation/probes/data/s7_yaw_damping.json
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
LONG_S = 30.0            # long enough for a 14 rad/s eigenvalue to be obvious
SPEEDS_STAB = [5.0, 10.0, 20.0]
KYD = [0.0, 50.0, 200.0, 500.0, 2000.0]


def yaw_stability(v: float, steer_kin_sign: float, K_yaw_damp: float) -> dict:
    """Straight running, no input. Does yaw rate grow?"""
    m = build(v0=v, steer_kin_sign=steer_kin_sign, K_Mz_missing=0.0,
              K_yaw_damp=K_yaw_damp)
    n = round(LONG_S / DT)
    yr = np.empty(n)
    for k in range(n):
        m.step(k * DT)
        yr[k] = m.main_frame.GetAngVelParent().z
    if not np.isfinite(yr).all():
        return {"diverged": True, "growth_per_s": float("inf"),
                "max_abs_yawrate_dps": float("inf"),
                "final_abs_yawrate_dps": float("inf")}

    # Growth rate from a log-linear fit over the second half, on the envelope.
    half = yr[n // 2:]
    env = np.abs(half)
    env = np.maximum(env, 1e-12)
    t = np.arange(env.size) * DT
    # coarse envelope: max within 0.5 s blocks, to fit the trend not the ripple
    blk = round(0.5 / DT)
    nb = env.size // blk
    peaks = np.array([env[i * blk:(i + 1) * blk].max() for i in range(nb)])
    tb = np.arange(nb) * 0.5
    good = peaks > 1e-11
    slope = float(np.polyfit(tb[good], np.log(peaks[good]), 1)[0]) if good.sum() > 3 else 0.0
    return {
        "diverged": False,
        "growth_per_s": slope,          # >0 means yaw rate is growing
        "max_abs_yawrate_dps": float(np.degrees(np.abs(yr).max())),
        "final_abs_yawrate_dps": float(np.degrees(np.abs(yr[-1000:]).mean())),
    }


def turn_fidelity(steer_kin_sign: float, K_yaw_damp: float,
                  v: float = 10.0, steer_deg: float = 3.0) -> dict:
    m = build(v0=v, steer_kin_sign=steer_kin_sign, K_Mz_missing=0.0,
              K_yaw_damp=K_yaw_damp)
    for k in range(500):
        m.step(k * DT)
    m.rider_source._input.steer_target = math.radians(steer_deg)
    n = round(4.0 / DT)
    yr = np.empty(n)
    st = np.empty(n)
    for k in range(n):
        m.step((500 + k) * DT)
        yr[k] = m.main_frame.GetAngVelParent().z
        st[k] = true_steer(m)
    tail = slice(-1000, None)
    yr_s = float(np.mean(yr[tail]))
    d = float(np.mean(st[tail]))
    yr_k = v * math.tan(d) * math.cos(m.params.rake) / m.params.wheelbase
    err = (yr_s - yr_k) / yr_k * 100 if abs(yr_k) > 1e-9 else float("nan")
    return {"yaw_sim_dps": math.degrees(yr_s),
            "yaw_kin_dps": math.degrees(yr_k),
            "steer_true_deg": math.degrees(d),
            "error_pct": err}


def main() -> None:
    out = {"long_s": LONG_S, "dt": DT, "stability": [], "fidelity": []}

    print("A. YAW STABILITY — 30 s straight running, no input, K_Mz_missing=0")
    print("   growth_per_s > 0 means the yaw rate is diverging\n")
    print(f"{'kin':<10} {'K_yaw':>6} {'v':>5} {'growth/s':>10} "
          f"{'max|yaw|':>10} {'final|yaw|':>11}")
    print("-" * 58)
    for sign, label in [(+1.0, "pre-fix"), (-1.0, "fixed")]:
        for kyd in KYD:
            for v in SPEEDS_STAB:
                r = yaw_stability(v, sign, kyd)
                out["stability"].append({"kinematics": label, "K_yaw_damp": kyd,
                                         "v": v, **r})
                if r["diverged"]:
                    print(f"{label:<10} {kyd:6.0f} {v:5.1f}   DIVERGED")
                else:
                    print(f"{label:<10} {kyd:6.0f} {v:5.1f} "
                          f"{r['growth_per_s']:+10.4f} "
                          f"{r['max_abs_yawrate_dps']:10.4f} "
                          f"{r['final_abs_yawrate_dps']:11.4f}")

    print("\n\nB. TURN FIDELITY — 3 deg steer at 10 m/s vs v*tan(d)*cos(e)/L\n")
    print(f"{'kin':<10} {'K_yaw':>6} {'steer':>8} {'yaw_sim':>9} "
          f"{'yaw_kin':>9} {'err %':>9}")
    print("-" * 54)
    for sign, label in [(+1.0, "pre-fix"), (-1.0, "fixed")]:
        for kyd in KYD:
            r = turn_fidelity(sign, kyd)
            out["fidelity"].append({"kinematics": label, "K_yaw_damp": kyd, **r})
            print(f"{label:<10} {kyd:6.0f} {r['steer_true_deg']:+8.3f} "
                  f"{r['yaw_sim_dps']:+9.3f} {r['yaw_kin_dps']:+9.3f} "
                  f"{r['error_pct']:+9.1f}")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s7_yaw_damping.json"
    p.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
