#!/usr/bin/env python3
"""
S8 — What roll coupling does PyChrono already provide, and what holds the bike up?

Groundwork before touching tau_roll. The supplement currently computes

    tau_roll = -K_roll_total * sin(roll - lean_delta) - C_roll * roll_rate
    K_roll_total = K_rider + K_centripetal_eff
    K_centripetal_eff = centripetal_factor * K_centripetal * K_lean / K_ctr_phys

`K_centripetal_eff` multiplies sin(roll), so it sits on the DIAGONAL of the roll
equation. It is presented as restoring Meijaard's off-diagonal K2 term, but an
off-diagonal term must couple STEER into roll, and no steer angle appears here.

Before replacing it we need two numbers that only measurement can give:

  A. SIGN AND STRENGTH of the steer->roll coupling PyChrono already supplies
     through the constraint chain. s1_routing_probe.py measured eta_B = 0.69 for
     a lateral force injected at the contact patch, so most of this path is
     already physical. Adding a full analytical m*v^2*h/L*delta term on top
     would double-count it. Here we hold a steer angle with the supplement's
     roll torque switched OFF and watch which way, and how hard, the chassis
     rolls — that is the genuine coupling, sign included.

  B. WHETHER THE BIKE STANDS UP without the diagonal fudge. K_rider is only
     m_rider*g*arm = 404 N.m/rad while gravity destabilisation is
     m*g*h = 1075 N.m/rad, so if K_centripetal_eff is simply deleted the roll
     spring is weaker than gravity and the machine may fall over. This sweeps
     centripetal_factor to find out.

Usage:  python validation/probes/s8_roll_coupling.py
Output: validation/probes/data/s8_roll_coupling.json
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


def natural_coupling(v: float, steer_deg: float) -> dict:
    """Hold a steer angle with the DIAGONAL centripetal fudge removed.

    centripetal_factor = 0 strips K_centripetal_eff but keeps the rider balance
    terms (K_rider, C_roll), so the bike stays upright. Whatever steady roll
    then develops is PyChrono's own steer->roll coupling working against a
    rider spring of known stiffness, and at equilibrium

        M_natural(delta) = K_rider * sin(phi_ss)

    which recovers the natural coupling moment directly, sign included.
    """
    m = build(v0=v, centripetal_factor=0.0)
    for k in range(500):
        m.step(k * DT)
    m.rider_source._input.steer_target = math.radians(steer_deg)

    n = 2500
    roll = np.empty(n)
    steer = np.empty(n)
    yaw = np.empty(n)
    for k in range(n):
        tel = m.step((500 + k) * DT)
        roll[k] = tel["chassis"]["roll"]
        steer[k] = true_steer(m)
        yaw[k] = m.main_frame.GetAngVelParent().z

    d = float(np.mean(steer[-500:]))
    r = float(np.mean(roll[-500:]))

    p = m.params
    K_rider = p.rider_mass * 9.81 * p.rider_lean_arm
    M_nat = K_rider * math.sin(r)              # equilibrium moment [N·m]
    # analytical full centripetal roll moment for this steer angle
    K_cent = p.total_mass * v * v * p.cg_height / p.wheelbase
    M_full = K_cent * math.cos(p.rake) * math.sin(d)
    return {
        "v": v, "steer_cmd_deg": steer_deg,
        "steer_true_deg": math.degrees(d),
        "roll_deg": math.degrees(r),
        "yaw_rate_dps": math.degrees(float(np.mean(yaw[-500:]))),
        "M_natural_Nm": M_nat,
        "M_full_analytical_Nm": M_full,
        "fraction_of_full": (M_nat / M_full) if abs(M_full) > 1e-9 else float("nan"),
    }


def balance(v: float, cf: float, secs: float = 5.0,
            perturb_deg: float = 5.0) -> dict:
    """Does the bike stay upright with this centripetal_factor?"""
    m = build(v0=v, centripetal_factor=cf, perturb_deg=perturb_deg)
    n = round(secs / DT)
    roll = np.empty(n)
    for k in range(n):
        tel = m.step(k * DT)
        roll[k] = tel["chassis"]["roll"]
    rd = np.degrees(roll)
    ok = bool(np.isfinite(rd).all() and np.abs(rd).max() < 45.0)
    return {"v": v, "centripetal_factor": cf, "upright": ok,
            "max_abs_roll_deg": float(np.abs(rd).max()) if np.isfinite(rd).all() else float("inf"),
            "final_abs_roll_deg": float(abs(rd[-1])) if np.isfinite(rd).all() else float("inf")}


def main() -> None:
    out = {"natural_coupling": [], "balance": []}

    print("A. NATURAL steer->roll coupling (supplement roll torque OFF)")
    print("   sign convention: +steer = wheel LEFT, +yaw = turning LEFT\n")
    print(f"{'v':>5} {'steer_cmd':>10} {'steer_true':>11} {'roll':>9} "
          f"{'yaw_rate':>9} {'M_nat':>9} {'M_full':>9} {'frac':>7}")
    print("-" * 74)
    for v in (5.0, 10.0, 15.0):
        for sd in (+3.0, -3.0):
            r = natural_coupling(v, sd)
            out["natural_coupling"].append(r)
            print(f"{v:5.1f} {sd:+10.1f} {r['steer_true_deg']:+11.3f} "
                  f"{r['roll_deg']:+9.3f} {r['yaw_rate_dps']:+9.3f} "
                  f"{r['M_natural_Nm']:+9.1f} {r['M_full_analytical_Nm']:+9.1f} "
                  f"{r['fraction_of_full']:+7.2f}")

    print("\n\nB. BALANCE vs centripetal_factor (5 deg roll perturbation, 5 s)\n")
    print(f"{'v':>5} {'cf':>6} {'upright':>8} {'max|roll|':>11} {'final|roll|':>12}")
    print("-" * 47)
    for v in (5.0, 10.0, 20.0):
        for cf in (0.0, 0.5, 1.0, 2.5):
            r = balance(v, cf)
            out["balance"].append(r)
            print(f"{v:5.1f} {cf:6.2f} {str(r['upright']):>8} "
                  f"{r['max_abs_roll_deg']:11.3f} {r['final_abs_roll_deg']:12.3f}")

    DATA.mkdir(parents=True, exist_ok=True)
    p = DATA / "s8_roll_coupling.json"
    p.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {p}")


if __name__ == "__main__":
    main()
