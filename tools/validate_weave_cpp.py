#!/usr/bin/env python3
"""
tools/validate_weave_cpp.py
────────────────────────────
Weave-mode frequency validation of the C++ 13-DOF engine
against the Sharp (1971) analytical prediction.

THEORY
──────
Sharp (1971) linearised roll equation:
    I_eff · φ̈  +  C_roll · φ̇  +  K_net · φ  =  0
    ω_ref  =  sqrt(K_net / I_eff)   [rad/s]

MEASUREMENT
───────────
1. Run C++ engine:  moto_engine --headless --v0 <vx> --phi0 0.5 --duration 2.0
2. Parse JSON stdout → collect roll field [rad].
3. Low-pass filter at 5 Hz (zero-phase IIR), skip first 120 ms.
4. Fit damped cosine:  A·exp(-σ·t)·cos(ω_d·t + ψ) + φ∞
5. Recover  ω_n = sqrt(ω_d² + σ²)
6. Gate:  |ω_sim - ω_ref| / ω_ref  ≤  25 %  → PASS

REFERENCE
─────────
Sharp (1971) J. Mech. Eng. Sci. 13(5) 316-329.
Meijaard et al. (2007) Proc. R. Soc. A 463 1955-1982.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import curve_fit
import yaml

ROOT   = Path(__file__).resolve().parent.parent
ENGINE = ROOT / "engine/build/moto_engine"
BIKE_YAML = ROOT / "core/chassis/bike_params/cbr600rr.yaml"

TEST_SPEEDS   = [5.0, 8.0, 10.0, 15.0, 20.0]   # m/s
PERTURB_DEG   = 0.5                              # initial roll [deg]
DURATION      = 2.0                              # sim window [s]
LP_CUTOFF     = 5.0                              # low-pass [Hz]
LP_ORDER      = 4
T_SKIP        = 0.12                             # skip first [s]
ERR_GATE      = 0.25                             # 25 %


# ── Low-pass filter ───────────────────────────────────────────────────────────

def _lowpass(data: np.ndarray, fc: float, dt: float, order: int = 4) -> np.ndarray:
    """Zero-phase first-order IIR cascade (each forward+backward pass = 2nd-order)."""
    alpha = math.exp(-2.0 * math.pi * fc * dt)
    out = data.astype(float).copy()
    for _ in range(order // 2):
        for k in range(1, len(out)):
            out[k] = alpha * out[k - 1] + (1.0 - alpha) * out[k]
        for k in range(len(out) - 2, -1, -1):
            out[k] = alpha * out[k + 1] + (1.0 - alpha) * out[k]
    return out


# ── Analytical reference ──────────────────────────────────────────────────────

def _omega_ref(params: dict, vx: float) -> tuple[float, float, float]:
    """Return (ω_ref, K_net, I_eff). Returns 0 if statically unstable."""
    g        = 9.81
    L        = params["wheelbase"]
    # cg_x in YAML is distance from rear axle
    cg_x     = params["cg_x"]
    Fz_f     = params["total_mass"] * g * cg_x / L

    K_lean   = Fz_f * params["trail"] * math.cos(params["rake"])
    K_gyro   = (params["front_wheel_Iyy"] * vx**2
                / (params["front_wheel_radius"] * L))
    K_ctr    = K_lean + K_gyro
    K_cent   = params["total_mass"] * vx**2 * params["cg_height"] / L
    K_supp   = (params["rider_mass"] * g * params["rider_lean_arm"]
                + 2.5 * K_cent * K_lean / max(K_ctr, 1.0))
    K_grav   = params["total_mass"] * g * params["cg_height"]
    K_net    = K_supp - K_grav

    I_eff    = (params["frame_Ixx"]
                + (params["main_frame_mass"]
                   + params["front_wheel_mass"]
                   + params["rear_wheel_mass"]
                   + params["front_fork_mass"]) * params["cg_height"]**2
                + params["swingarm_mass"] * 0.33**2)

    if K_net <= 0.0:
        return 0.0, K_net, I_eff
    return math.sqrt(K_net / I_eff), K_net, I_eff


# ── Damped cosine model ───────────────────────────────────────────────────────

def _damped_cos(t, A, sigma, omega_d, psi, phi_inf):
    return A * np.exp(-sigma * t) * np.cos(omega_d * t + psi) + phi_inf


# ── Run C++ engine and collect roll trace ─────────────────────────────────────

def _run_engine(vx: float, phi0_deg: float, duration: float) -> np.ndarray | None:
    """
    Run moto_engine --headless, collect JSON stdout, return roll array [rad].
    Returns None on subprocess error.
    """
    cmd = [
        str(ENGINE),
        "--headless",
        "--v0",       str(vx),
        "--phi0",     str(phi0_deg),
        "--duration", str(duration),
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        print(f"  [ERROR] Engine timed out at v={vx} m/s", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  [ERROR] Subprocess failed: {e}", file=sys.stderr)
        return None

    rolls = []
    for line in result.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            frame = json.loads(line)
            rolls.append(float(frame["roll"]))
        except (json.JSONDecodeError, KeyError):
            continue

    if len(rolls) < 20:
        return None
    return np.array(rolls)


# ── Fit damped cosine to roll trace ──────────────────────────────────────────

def _fit_roll(rolls: np.ndarray, omega_ref_guess: float,
              duration: float) -> tuple[float, float, float]:
    """
    Low-pass filter, skip T_SKIP, fit damped cosine.
    Returns (omega_n, sigma, rms_residual) or (0, 0, inf) on failure.
    """
    n  = len(rolls)
    dt = duration / n
    t_arr = np.arange(n) * dt

    filtered = _lowpass(rolls, LP_CUTOFF, dt, LP_ORDER)

    skip = round(T_SKIP / dt)
    if skip >= n - 10:
        return 0.0, 0.0, float("inf")

    t_fit = t_arr[skip:] - t_arr[skip]
    y_fit = filtered[skip:]

    zeta_guess = 0.30
    sigma_g    = zeta_guess * max(omega_ref_guess, 1.0)
    omega_d_g  = max(omega_ref_guess, 1.0) * math.sqrt(max(1 - zeta_guess**2, 0.01))
    A_guess    = max(abs(y_fit[0]), math.radians(0.02))

    try:
        popt, _ = curve_fit(
            _damped_cos,
            t_fit,
            y_fit,
            p0     = [A_guess, sigma_g, omega_d_g, 0.0, 0.0],
            bounds = (
                [1e-6,   0.0,  0.5, -math.pi, -0.02],
                [0.20,  50.0, 50.0,  math.pi,  0.02],
            ),
            maxfev = 20000,
        )
        A, sigma, omega_d, psi, phi_inf = popt

        if A < math.radians(0.01) or sigma < 0.05:
            return 0.0, 0.0, float("inf")

        omega_n  = math.sqrt(omega_d**2 + sigma**2)
        residual = float(np.sqrt(np.mean((y_fit - _damped_cos(t_fit, *popt))**2)))
        return omega_n, sigma, residual

    except Exception:
        return 0.0, 0.0, float("inf")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    if not ENGINE.exists():
        print(f"ERROR: Engine binary not found at {ENGINE}")
        print("Build it first:  cd engine/build && make -j$(nproc)")
        sys.exit(1)

    with open(BIKE_YAML) as f:
        params = yaml.safe_load(f)

    print()
    print("C++ 13-DOF Engine — Weave-mode validation against Sharp (1971)")
    print(f"  Engine   : {ENGINE}")
    print(f"  Bike     : {BIKE_YAML.name}")
    print(f"  phi0     : {PERTURB_DEG}°   duration: {DURATION} s")
    print(f"  LP filter: {LP_CUTOFF} Hz   skip: {T_SKIP*1000:.0f} ms")
    print(f"  Gate     : |err| ≤ {ERR_GATE*100:.0f}%")
    print()

    hdr = (f"{'v [m/s]':>8}  {'ω_ref':>8}  {'f_ref [Hz]':>10}  "
           f"{'ω_sim':>8}  {'f_sim [Hz]':>10}  {'err%':>7}  "
           f"{'ζ_sim':>7}  {'PASS/FAIL':>9}")
    print(hdr)
    print("─" * 85)

    all_pass = True
    ref_list, sim_list = [], []

    for vx in TEST_SPEEDS:
        omega_r, K_net, I_eff = _omega_ref(params, vx)
        f_ref = omega_r / (2 * math.pi)

        if omega_r <= 0:
            print(f"{vx:>8.1f}  {omega_r:>8.3f}  {f_ref:>10.3f}  "
                  f"{'—':>8}  {'—':>10}  {'—':>7}  {'—':>7}  {'SKIP':>9}")
            continue

        rolls = _run_engine(vx, PERTURB_DEG, DURATION)
        if rolls is None:
            print(f"{vx:>8.1f}  {omega_r:>8.3f}  {f_ref:>10.3f}  "
                  f"{'ERR':>8}  {'—':>10}  {'—':>7}  {'—':>7}  {'FAIL':>9}")
            all_pass = False
            continue

        omega_n, sigma, rms = _fit_roll(rolls, omega_r, DURATION)

        if omega_n <= 0:
            print(f"{vx:>8.1f}  {omega_r:>8.3f}  {f_ref:>10.3f}  "
                  f"{'NoFit':>8}  {'—':>10}  {'—':>7}  {'—':>7}  {'FAIL':>9}")
            all_pass = False
            continue

        f_sim  = omega_n / (2 * math.pi)
        zeta   = sigma / omega_n
        err_pct = abs(omega_n - omega_r) / omega_r * 100.0
        passed  = err_pct <= ERR_GATE * 100.0
        if not passed:
            all_pass = False
        tag = "PASS" if passed else "FAIL"

        print(f"{vx:>8.1f}  {omega_r:>8.3f}  {f_ref:>10.3f}  "
              f"{omega_n:>8.3f}  {f_sim:>10.3f}  "
              f"{err_pct:>6.2f}%  {zeta:>7.3f}  {tag:>9}")

        ref_list.append((vx, omega_r))
        sim_list.append((vx, omega_n))

    print("─" * 85)
    print()

    if ref_list and sim_list:
        errs = [abs(ws - wr) / wr * 100
                for (_, wr), (_, ws) in zip(ref_list, sim_list)]
        print(f"  Error — best: {min(errs):.2f}%   worst: {max(errs):.2f}%   "
              f"mean: {sum(errs)/len(errs):.2f}%")
        print()

    print(f"  Overall result: {'PASS — all speeds within 25%' if all_pass else 'FAIL — one or more speeds exceeded gate'}")
    print()
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
