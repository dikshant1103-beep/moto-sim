#!/usr/bin/env python3
"""
tools/validate_capsize_sharp1971.py
────────────────────────────────────
Capsize-mode threshold validation against the Sharp / Whipple linearised model.

THEORY
──────
The Whipple motorcycle model has two principal instability modes:

  1. Capsize mode  (real eigenvalue λ_c):
       λ_c > 0  ↔  bike topples — roll diverges monotonically
       λ_c < 0  ↔  bike upright — marginally stable (slowly returns to zero)
     Condition for stability:  K_net > 0  (supplement stiffness > gravitational
     destabilisation), where
       K_net = K_rider + K_centripetal_eff − K_grav
     The speed at which K_net = 0 is the analytical capsize threshold v_cap.

  2. Weave mode  (complex conjugate pair):
       Real part > 0  ↔  oscillation grows (weave-speed instability)
       Real part < 0  ↔  oscillation decays (stable weave)
     Already validated in validate_weave_sharp1971.py.

For the CBR600RR at typical parameters the analytical capsize threshold is
≈ 1.8 m/s.  This script sweeps speeds from 0.5 to 4.0 m/s, simulates a
5° roll perturbation for 1 s, and determines from the sign of Δ|roll|
whether the simulation is in the capsize or stable regime.

MEASUREMENT
───────────
Step 1: Apply 5° roll perturbation at each test speed.
Step 2: Run 1.0 s headless simulation (no throttle / brakes / steer).
Step 3: Compare |roll(t=1s)| against initial 5°.
         Δ > 0  →  CAPSIZE  (roll grew)
         Δ < 0  →  STABLE   (roll decayed or oscillated to smaller amplitude)
Step 4: Find the simulation capsize speed v_cap_sim = lowest speed where
        the mode flips from CAPSIZE to STABLE.
Step 5: Compare v_cap_sim against analytical v_cap; gate = |error| ≤ 50 %.

REFERENCE
─────────
Sharp (1971) J. Mech. Eng. Sci. 13(5) 316-329.
Meijaard et al. (2007) Proc. R. Soc. A 463 1955-1982.
Cossalter (2006) "Motorcycle Dynamics" 2nd ed. Ch. 7.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path

import yaml
import numpy as np
import pychrono as chrono

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from core.chassis.motorcycle import Motorcycle, BikeParams
from core.rider.rider_input  import ConstantInputSource
from core.road.flat_plane    import FlatPlane

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"

DT          = 0.001         # physics step [s]
SIM_SECS    = 1.0           # simulation window per speed
PERTURB_DEG = 5.0           # initial roll perturbation [deg]
TEST_SPEEDS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]  # m/s


# ── Analytical K_net and capsize speed ───────────────────────────────────────

def _k_net(params: BikeParams, vx: float) -> float:
    """Net roll stiffness K_net [N·m/rad].  Positive = stable, negative = capsize."""
    p = params
    g = 9.81
    Fz_f       = p.total_mass * g * (p.cg_x / p.wheelbase)
    K_lean     = Fz_f * p.trail * math.cos(p.rake)
    K_gyro     = p.front_wheel_Iyy * vx * vx / (p.front_wheel_radius * p.wheelbase)
    K_ctr      = K_lean + K_gyro
    K_cent     = p.total_mass * vx * vx * p.cg_height / p.wheelbase
    K_cent_eff = 2.5 * K_cent * K_lean / max(K_ctr, 1.0)
    K_rider    = p.rider_mass * g * p.rider_lean_arm
    K_grav     = p.total_mass * g * p.cg_height
    return K_rider + K_cent_eff - K_grav


def _capsize_speed(params: BikeParams) -> float:
    """Analytical capsize speed [m/s]: first v (step 0.01) where K_net > 0."""
    for vi in range(1, 1001):
        v = vi * 0.01
        if _k_net(params, v) > 0:
            return v
    return float('nan')


# ── Per-speed simulation ──────────────────────────────────────────────────────

def _simulate(vx: float) -> tuple[float, float, str]:
    """
    Run simulation at vx with PERTURB_DEG roll perturbation.
    Returns (delta_roll_deg, final_roll_deg, mode_label).
      delta_roll > 0 → capsize; delta_roll < 0 → stable.
    """
    rider = ConstantInputSource(steer_torque=0.0, throttle=0.0,
                                brake_front=0.0, brake_rear=0.0)
    moto = Motorcycle.from_yaml(BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
                                FlatPlane(), rider, dt=DT)
    moto.build()
    moto.reset(v0=vx)
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(PERTURB_DEG)))

    n = round(SIM_SECS / DT)
    roll_final = PERTURB_DEG  # fallback
    for k in range(n):
        telem = moto.step(k * DT)
        roll_final = math.degrees(abs(telem["chassis"]["roll"]))

    delta = roll_final - PERTURB_DEG
    label = "CAPSIZE" if delta > 0 else "STABLE "
    return delta, roll_final, label


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    with open(BIKE_YAML) as f:
        raw = yaml.safe_load(f)
    params = BikeParams(raw)

    v_cap_analytical = _capsize_speed(params)

    print()
    print("CBR600RR — Capsize mode threshold validation")
    print(f"  Analytical capsize speed (K_net = 0 crossing): {v_cap_analytical:.2f} m/s")
    print()
    print(f"  Perturbation: {PERTURB_DEG}°  |  Window: {SIM_SECS} s  |  Step: {DT*1000:.0f} ms")
    print(f"  Mode: CAPSIZE if |roll(1s)| > {PERTURB_DEG}°;  STABLE if |roll(1s)| < {PERTURB_DEG}°")
    print()

    # Print K_net summary across speeds
    print("Analytical K_net [N·m/rad] at each test speed:")
    for vx in TEST_SPEEDS:
        kn = _k_net(params, vx)
        sign = "+" if kn >= 0 else ""
        print(f"  v = {vx:4.1f} m/s  →  K_net = {sign}{kn:.0f}  "
              f"({'stable' if kn > 0 else 'CAPSIZE'})")
    print()

    hdr = (f"{'v [m/s]':>8}  {'K_net':>8}  {'Δroll [°]':>10}  "
           f"{'|roll| final':>13}  {'Mode':>8}  {'Gate':>6}")
    print(hdr)
    print("─" * 68)

    results: list[tuple[float, str]] = []
    all_pass = True

    for vx in TEST_SPEEDS:
        kn = _k_net(params, vx)
        delta, roll_f, mode = _simulate(vx)
        expected_capsize = kn < 0

        # Gate: simulation mode must agree with analytical prediction
        sim_capsize = delta > 0
        gate = (sim_capsize == expected_capsize)
        if not gate:
            all_pass = False
        tag = "PASS" if gate else "FAIL"

        sign_delta = "+" if delta >= 0 else ""
        print(f"{vx:>8.1f}  {kn:>+8.0f}  {sign_delta}{delta:>9.2f}°  "
              f"{roll_f:>12.2f}°  {mode:>8}  {tag:>6}")
        results.append((vx, mode.strip()))

    print("─" * 68)
    print()

    # Find simulation capsize threshold
    v_cap_sim = float('nan')
    for i in range(len(results) - 1):
        v_lo, m_lo = results[i]
        v_hi, m_hi = results[i + 1]
        if m_lo == "CAPSIZE" and m_hi == "STABLE":
            v_cap_sim = (v_lo + v_hi) / 2.0
            break

    if not math.isnan(v_cap_sim) and not math.isnan(v_cap_analytical):
        err = abs(v_cap_sim - v_cap_analytical) / v_cap_analytical * 100
        gate_threshold = err <= 50.0
        if not gate_threshold:
            all_pass = False
        print(f"  Analytical capsize speed: {v_cap_analytical:.2f} m/s")
        print(f"  Simulation capsize speed: {v_cap_sim:.2f} m/s  "
              f"(error {err:.1f}%,  gate ≤ 50%  →  {'PASS' if gate_threshold else 'FAIL'})")
    elif math.isnan(v_cap_sim):
        print("  WARNING: No CAPSIZE→STABLE transition detected in test speed range.")
        print("  Extend TEST_SPEEDS below 0.5 m/s or above 4.0 m/s if needed.")
        all_pass = False

    print()
    print(f"  Result: {'ALL SPEEDS PASS' if all_pass else 'SOME SPEEDS FAIL'}")
    print()


if __name__ == "__main__":
    main()
