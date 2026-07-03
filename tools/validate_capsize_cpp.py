#!/usr/bin/env python3
"""
tools/validate_capsize_cpp.py
──────────────────────────────
Capsize threshold validation of the C++ 13-DOF engine
against the Sharp (1971) analytical prediction.

THEORY
──────
The Whipple/Sharp model has a capsize mode (real eigenvalue):
    λ_c > 0  ↔  bike capsizes — roll diverges monotonically
    λ_c < 0  ↔  bike stable  — roll returns toward zero

Capsize threshold: speed v_cap where K_net (net roll stiffness) = 0.

MEASUREMENT
───────────
1. Run: moto_engine --headless --v0 <vx> --phi0 5.0 --duration 1.0
2. Compare |roll_final| vs 5.0°:
     |roll_final| > 5° → CAPSIZE
     |roll_final| < 5° → STABLE
3. Find CAPSIZE→STABLE transition speed in simulation.
4. Gate: |v_cap_sim - v_cap_ref| / v_cap_ref ≤ 50% → PASS

REFERENCE
─────────
Sharp (1971) J. Mech. Eng. Sci. 13(5) 316-329.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT      = Path(__file__).resolve().parent.parent
ENGINE    = ROOT / "engine/build/moto_engine"
BIKE_YAML = ROOT / "core/chassis/bike_params/cbr600rr.yaml"

TEST_SPEEDS   = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]   # m/s
PERTURB_DEG   = 5.0
DURATION      = 1.0
ERR_GATE_REL  = 0.50   # 50 %


# ── Analytical K_net ──────────────────────────────────────────────────────────

def _k_net(params: dict, vx: float) -> float:
    """Net roll stiffness [N·m/rad]. Positive = stable, negative = capsize."""
    g    = 9.81
    L    = params["wheelbase"]
    cg_x = params["cg_x"]   # distance from rear axle in YAML
    Fz_f = params["total_mass"] * g * cg_x / L

    K_lean   = Fz_f * params["trail"] * math.cos(params["rake"])
    K_gyro   = (params["front_wheel_Iyy"] * vx**2
                / (params["front_wheel_radius"] * L))
    K_ctr    = K_lean + K_gyro
    K_cent   = params["total_mass"] * vx**2 * params["cg_height"] / L
    K_cent_e = 2.5 * K_cent * K_lean / max(K_ctr, 1.0)
    K_rider  = params["rider_mass"] * g * params["rider_lean_arm"]
    K_grav   = params["total_mass"] * g * params["cg_height"]
    return K_rider + K_cent_e - K_grav


def _capsize_speed_ref(params: dict) -> float:
    """Analytical capsize speed: sweep v until K_net > 0."""
    for vi in range(1, 1001):
        v = vi * 0.01
        if _k_net(params, v) > 0:
            return v
    return float("nan")


# ── Run engine and get final roll ─────────────────────────────────────────────

def _run_and_get_final_roll(vx: float) -> float | None:
    """
    Run engine, return |roll| of the LAST telemetry frame [degrees].
    Returns None on error.
    """
    cmd = [
        str(ENGINE),
        "--headless",
        "--v0",       str(vx),
        "--phi0",     str(PERTURB_DEG),
        "--duration", str(DURATION),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired:
        print(f"  [ERROR] Engine timed out at v={vx} m/s", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [ERROR] {e}", file=sys.stderr)
        return None

    last_roll_deg = None
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            frame = json.loads(line)
            last_roll_deg = math.degrees(abs(float(frame["roll"])))
        except (json.JSONDecodeError, KeyError):
            continue

    return last_roll_deg


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    if not ENGINE.exists():
        print(f"ERROR: Engine binary not found at {ENGINE}")
        print("Build it first:  cd engine/build && make -j$(nproc)")
        sys.exit(1)

    with open(BIKE_YAML) as f:
        params = yaml.safe_load(f)

    v_cap_ref = _capsize_speed_ref(params)

    print()
    print("C++ 13-DOF Engine — Capsize threshold validation against Sharp (1971)")
    print(f"  Engine : {ENGINE}")
    print(f"  Bike   : {BIKE_YAML.name}")
    print(f"  phi0   : {PERTURB_DEG}°   duration: {DURATION} s")
    print(f"  Gate   : |v_cap_sim - v_cap_ref| / v_cap_ref ≤ {ERR_GATE_REL*100:.0f}%")
    print()
    print(f"  Analytical capsize speed (K_net = 0): {v_cap_ref:.2f} m/s")
    print()

    # Print K_net summary
    print("  Analytical K_net [N·m/rad] at each test speed:")
    for vx in TEST_SPEEDS:
        kn   = _k_net(params, vx)
        sign = "+" if kn >= 0 else ""
        mode = "stable" if kn >= 0 else "CAPSIZE"
        print(f"    v = {vx:.1f} m/s  →  K_net = {sign}{kn:.0f}  ({mode})")
    print()

    hdr = (f"{'v [m/s]':>8}  {'K_net':>8}  "
           f"{'|roll_final|':>13}  {'Mode':>9}  {'Gate':>6}")
    print(hdr)
    print("─" * 60)

    results: list[tuple[float, str]] = []
    all_pass = True

    for vx in TEST_SPEEDS:
        kn = _k_net(params, vx)
        roll_final = _run_and_get_final_roll(vx)

        if roll_final is None:
            print(f"{vx:>8.1f}  {kn:>+8.0f}  {'ERROR':>13}  {'?':>9}  {'FAIL':>6}")
            all_pass = False
            results.append((vx, "ERROR"))
            continue

        sim_capsize = roll_final > PERTURB_DEG
        mode_label  = "CAPSIZE" if sim_capsize else "STABLE "
        results.append((vx, "CAPSIZE" if sim_capsize else "STABLE"))

        # Gate: sim mode must agree with analytical prediction
        ana_capsize = kn < 0
        gate = (sim_capsize == ana_capsize)
        if not gate:
            all_pass = False
        tag = "PASS" if gate else "FAIL"

        print(f"{vx:>8.1f}  {kn:>+8.0f}  {roll_final:>12.2f}°  {mode_label:>9}  {tag:>6}")

    print("─" * 60)
    print()

    # Find simulation capsize→stable transition
    v_cap_sim = float("nan")
    for i in range(len(results) - 1):
        v_lo, m_lo = results[i]
        v_hi, m_hi = results[i + 1]
        if m_lo == "CAPSIZE" and m_hi == "STABLE":
            v_cap_sim = (v_lo + v_hi) / 2.0
            break

    if not math.isnan(v_cap_sim) and not math.isnan(v_cap_ref):
        err_pct = abs(v_cap_sim - v_cap_ref) / v_cap_ref * 100.0
        gate_thresh = err_pct <= ERR_GATE_REL * 100.0
        if not gate_thresh:
            all_pass = False
        tag = "PASS" if gate_thresh else "FAIL"
        print(f"  Analytical capsize speed : {v_cap_ref:.2f} m/s")
        print(f"  Simulation capsize speed : {v_cap_sim:.2f} m/s  "
              f"(error {err_pct:.1f}%,  gate ≤ {ERR_GATE_REL*100:.0f}%  →  {tag})")
    elif math.isnan(v_cap_sim):
        print("  WARNING: No CAPSIZE→STABLE transition detected in test speed range.")
        print("  Extend TEST_SPEEDS or check engine output.")
        all_pass = False

    print()
    print(f"  Overall result: {'PASS' if all_pass else 'FAIL'}")
    print()
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
