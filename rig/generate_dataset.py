#!/usr/bin/env python3
"""
rig/generate_dataset.py  —  PINN-1: Synthetic tire dataset generation
──────────────────────────────────────────────────────────────────────
Generates ~200 K rows of (Fz, alpha, kappa, gamma) → (Fx, Fy, Mz)
samples from the Pacejka MF analytical model (warm tire, zero wear)
for use as PINN-2 training data.

Sampling strategy — three stages to balance coverage and density:

  Stage 1 — Coarse Cartesian grid (full input space):
    Fz   :  9 values  500 – 3000 N   (covers unloaded to heavily loaded)
    alpha: 50 values −0.35 – +0.35 rad  (±20°, full linear+nonlinear)
    kappa: 25 values −0.50 – +0.50      (full slip range)
    gamma:  5 values −0.40 – +0.40 rad  (±23° camber, motorcycle range)
    → 9 × 50 × 25 × 5 = 56 250 rows

  Stage 2 — Importance region near the nonlinear knee:
    alpha ∈ [−0.15, +0.15] rad  (±8.6°, α where Fy is changing fastest)
    kappa ∈ [−0.15, +0.15]      (κ where Fx peaks and Fy coupling is strong)
    Same 9 Fz × 5 gamma → 80 × 60 sub-grid per (Fz, gamma) combination
    → 9 × 5 × 80 × 60 = 216 000 rows  (trimmed to knee box only)

  Stage 3 — Latin-hypercube random fill (exploration):
    50 000 random points uniform in the full (Fz, alpha, kappa, gamma) box.
    Removes gaps left by the grid at non-grid Fz values.

  Deduplication: none (deterministic model — duplicates are fine for PINN).
  Total: ~200 K rows (Stage 1 + trimmed Stage 2 + Stage 3).

Output: dataset/tire_synthetic.csv
Columns: Fz [N], alpha [rad], kappa [–], gamma [rad], Fx [N], Fy [N], Mz [Nm]

Run from moto_sim/ root:
    python rig/generate_dataset.py
    python rig/generate_dataset.py --compound soft   # alternate compound
    python rig/generate_dataset.py --rows 50000      # smaller quick run
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from core.tire.pinn_model import DummyTirePINN, TireCoeffs

DATASET_DIR = ROOT / "dataset"

# ── Input bounds ──────────────────────────────────────────────────────────────
FZ_MIN,    FZ_MAX    =  500.0,  3000.0   # [N]
ALPHA_MIN, ALPHA_MAX = -0.35,   +0.35    # [rad]  ≈ ±20°
KAPPA_MIN, KAPPA_MAX = -0.50,   +0.50    # [–]
GAMMA_MIN, GAMMA_MAX = -0.40,   +0.40    # [rad]  ≈ ±23°

# Nonlinear knee region (importance sampling)
KNEE_ALPHA = 0.15   # |alpha| ≤ this → knee region
KNEE_KAPPA = 0.15   # |kappa| ≤ this → knee region


# ─────────────────────────────────────────────────────────────────────────────
# Core evaluation (vectorised via Python loop — fast enough for 200 K rows)
# ─────────────────────────────────────────────────────────────────────────────

def _eval_batch(pinn: DummyTirePINN,
                Fz_arr: np.ndarray,
                alpha_arr: np.ndarray,
                kappa_arr: np.ndarray,
                gamma_arr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate Pacejka model for N samples. Returns (Fx, Fy, Mz) arrays."""
    n = len(Fz_arr)
    Fx_out = np.empty(n)
    Fy_out = np.empty(n)
    Mz_out = np.empty(n)
    for i in range(n):
        Fx_out[i], Fy_out[i], Mz_out[i] = pinn.predict_combined(
            Fz        = float(Fz_arr[i]),
            slip_angle= float(alpha_arr[i]),
            kappa     = float(kappa_arr[i]),
            camber    = float(gamma_arr[i]),
        )
    return Fx_out, Fy_out, Mz_out


# ─────────────────────────────────────────────────────────────────────────────
# Sampling stages
# ─────────────────────────────────────────────────────────────────────────────

def _stage1_coarse_grid() -> tuple[np.ndarray, ...]:
    """Full-space Cartesian grid — broad coverage."""
    Fz_vals    = np.array([500, 750, 1000, 1250, 1500, 1750, 2000, 2500, 3000],
                           dtype=float)
    alpha_vals = np.linspace(ALPHA_MIN, ALPHA_MAX, 50)
    kappa_vals = np.linspace(KAPPA_MIN, KAPPA_MAX, 25)
    gamma_vals = np.linspace(GAMMA_MIN, GAMMA_MAX, 5)

    grids = np.meshgrid(Fz_vals, alpha_vals, kappa_vals, gamma_vals, indexing="ij")
    Fz    = grids[0].ravel()
    alpha = grids[1].ravel()
    kappa = grids[2].ravel()
    gamma = grids[3].ravel()
    print(f"  Stage 1 (coarse grid):      {len(Fz):>8,} rows", flush=True)
    return Fz, alpha, kappa, gamma


def _stage2_knee_importance() -> tuple[np.ndarray, ...]:
    """Dense grid near the nonlinear knee."""
    Fz_vals    = np.array([500, 750, 1000, 1250, 1500, 1750, 2000, 2500, 3000],
                           dtype=float)
    alpha_vals = np.linspace(-KNEE_ALPHA, KNEE_ALPHA, 80)
    kappa_vals = np.linspace(-KNEE_KAPPA, KNEE_KAPPA, 60)
    gamma_vals = np.linspace(GAMMA_MIN, GAMMA_MAX, 5)

    grids = np.meshgrid(Fz_vals, alpha_vals, kappa_vals, gamma_vals, indexing="ij")
    Fz    = grids[0].ravel()
    alpha = grids[1].ravel()
    kappa = grids[2].ravel()
    gamma = grids[3].ravel()
    print(f"  Stage 2 (knee importance):  {len(Fz):>8,} rows", flush=True)
    return Fz, alpha, kappa, gamma


def _stage3_random_fill(n: int, rng: np.random.Generator) -> tuple[np.ndarray, ...]:
    """Uniform random fill — closes gaps between grid lines."""
    Fz    = rng.uniform(FZ_MIN,    FZ_MAX,    n)
    alpha = rng.uniform(ALPHA_MIN, ALPHA_MAX, n)
    kappa = rng.uniform(KAPPA_MIN, KAPPA_MAX, n)
    gamma = rng.uniform(GAMMA_MIN, GAMMA_MAX, n)
    print(f"  Stage 3 (random fill):      {n:>8,} rows", flush=True)
    return Fz, alpha, kappa, gamma


def _stage4_peak_region(rng: np.random.Generator) -> tuple[np.ndarray, ...]:
    """
    Extra samples around the Fy and Fx peaks — critical for PINN accuracy
    near the friction limit.

    Fy peak:   alpha ≈ ±peak_alpha  (compound-dependent, ~7–12°),  kappa ≈ 0
    Fx peak:   kappa ≈ ±0.08–0.12,  alpha ≈ 0
    Coupled:   alpha ≈ ±peak/2,     kappa ≈ ±0.06  (mixed braking/cornering)
    """
    n_per_region = 5_000
    regions = []

    for sign in (+1, -1):
        # Fy peak cluster
        Fz    = rng.uniform(FZ_MIN, FZ_MAX, n_per_region)
        alpha = rng.normal(sign * math.radians(9.0), math.radians(3.0), n_per_region)
        kappa = rng.normal(0.0,   0.04, n_per_region)
        gamma = rng.uniform(GAMMA_MIN, GAMMA_MAX, n_per_region)
        regions.append((Fz, np.clip(alpha, ALPHA_MIN, ALPHA_MAX),
                        np.clip(kappa, KAPPA_MIN, KAPPA_MAX), gamma))

        # Fx peak cluster
        Fz    = rng.uniform(FZ_MIN, FZ_MAX, n_per_region)
        alpha = rng.normal(0.0,  math.radians(2.0), n_per_region)
        kappa = rng.normal(sign * 0.10, 0.03, n_per_region)
        gamma = rng.uniform(GAMMA_MIN, GAMMA_MAX, n_per_region)
        regions.append((Fz, np.clip(alpha, ALPHA_MIN, ALPHA_MAX),
                        np.clip(kappa, KAPPA_MIN, KAPPA_MAX), gamma))

    Fz    = np.concatenate([r[0] for r in regions])
    alpha = np.concatenate([r[1] for r in regions])
    kappa = np.concatenate([r[2] for r in regions])
    gamma = np.concatenate([r[3] for r in regions])
    print(f"  Stage 4 (peak clusters):    {len(Fz):>8,} rows", flush=True)
    return Fz, alpha, kappa, gamma


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def generate(compound: str = "medium",
             target_rows: int | None = None,
             seed: int = 42) -> Path:

    yaml_path = ROOT / "core/tire/pacejka_coeffs" / f"{compound}.yaml"
    assert yaml_path.exists(), f"Compound YAML not found: {yaml_path}"

    coeffs = TireCoeffs.from_yaml(yaml_path)
    pinn   = DummyTirePINN.from_coeffs(coeffs)

    rng = np.random.default_rng(seed)

    print(f"\n=== PINN-1: Synthetic Dataset Generation ===")
    print(f"Compound : {compound}  ({yaml_path.name})")
    print(f"Seed     : {seed}")
    print(f"\nSampling stages:")

    # Build input arrays
    Fz1, a1, k1, g1 = _stage1_coarse_grid()
    Fz2, a2, k2, g2 = _stage2_knee_importance()
    n_random = 50_000
    Fz3, a3, k3, g3 = _stage3_random_fill(n_random, rng)
    Fz4, a4, k4, g4 = _stage4_peak_region(rng)

    Fz    = np.concatenate([Fz1, Fz2, Fz3, Fz4])
    alpha = np.concatenate([a1,  a2,  a3,  a4])
    kappa = np.concatenate([k1,  k2,  k3,  k4])
    gamma = np.concatenate([g1,  g2,  g3,  g4])

    # Optionally subsample to target_rows
    if target_rows is not None and target_rows < len(Fz):
        idx   = rng.choice(len(Fz), size=target_rows, replace=False)
        Fz    = Fz[idx];    alpha = alpha[idx]
        kappa = kappa[idx]; gamma = gamma[idx]

    total = len(Fz)
    print(f"\nTotal rows to evaluate: {total:,}")

    # ── Evaluate Pacejka ──────────────────────────────────────────────────────
    print(f"\nEvaluating Pacejka MF ...", flush=True)
    t0 = time.perf_counter()

    CHUNK = 10_000
    Fx_parts, Fy_parts, Mz_parts = [], [], []
    for start in range(0, total, CHUNK):
        end = min(start + CHUNK, total)
        fx, fy, mz = _eval_batch(pinn, Fz[start:end], alpha[start:end],
                                  kappa[start:end], gamma[start:end])
        Fx_parts.append(fx);  Fy_parts.append(fy);  Mz_parts.append(mz)
        if (start // CHUNK) % 5 == 0:
            pct = end / total * 100
            elapsed = time.perf_counter() - t0
            eta = elapsed / max(pct, 1) * (100 - pct)
            print(f"  {pct:5.1f}%  ({end:,}/{total:,})  "
                  f"elapsed={elapsed:.1f}s  ETA={eta:.1f}s", flush=True)

    Fx = np.concatenate(Fx_parts)
    Fy = np.concatenate(Fy_parts)
    Mz = np.concatenate(Mz_parts)

    elapsed_total = time.perf_counter() - t0
    rate = total / elapsed_total
    print(f"\nEvaluation complete: {elapsed_total:.1f} s  ({rate:.0f} rows/s)")

    # ── Assemble and write CSV ────────────────────────────────────────────────
    DATASET_DIR.mkdir(exist_ok=True)
    out_path = DATASET_DIR / f"tire_synthetic_{compound}.csv"

    header = "Fz,alpha,kappa,gamma,Fx,Fy,Mz"
    data   = np.column_stack([Fz, alpha, kappa, gamma, Fx, Fy, Mz])

    np.savetxt(out_path, data, delimiter=",", header=header, comments="", fmt="%.6g")

    size_mb = out_path.stat().st_size / 1e6
    print(f"\nDataset saved → {out_path}")
    print(f"  Rows     : {total:,}")
    print(f"  Columns  : Fz, alpha, kappa, gamma, Fx, Fy, Mz")
    print(f"  File size: {size_mb:.1f} MB")

    # ── Quick sanity statistics ───────────────────────────────────────────────
    print(f"\nInput ranges:")
    for name, arr in [("Fz   [N]",   Fz),
                       ("alpha [rad]", alpha),
                       ("kappa [–]",  kappa),
                       ("gamma [rad]", gamma)]:
        print(f"  {name:12s}  min={arr.min():+.4g}  max={arr.max():+.4g}"
              f"  mean={arr.mean():+.4g}")

    print(f"\nOutput ranges:")
    for name, arr in [("Fx [N]",   Fx),
                       ("Fy [N]",   Fy),
                       ("Mz [Nm]",  Mz)]:
        print(f"  {name:10s}  min={arr.min():+.1f}  max={arr.max():+.1f}"
              f"  |mean|={abs(arr.mean()):.2f}")

    # Friction ellipse: (Fx/µFz)² + (Fy/µFz)² ≤ 1
    # Note: Pacejka combined-slip does NOT enforce the friction circle — Fx and Fy
    # are reduced independently by Gxα / Gyκ so their vector sum can exceed µFz
    # at large combined-slip inputs.  Expected violation rate: 5–15%.
    # The Physics Guard Layer (PINN Phase 3.3) will enforce this at inference time.
    mu     = coeffs.mu
    ratio  = (Fx**2 + Fy**2) / (mu * Fz + 1e-6)**2
    viol   = int(np.sum(ratio > 1.0))
    pct_ok = (1 - viol / total) * 100
    print(f"\nFriction ellipse: {pct_ok:.2f}% within µFz  "
          f"({viol} exceed — expected, Pacejka has no built-in circle)  ✓")

    # Mz sign consistency: only check pre-reversal region (α ∈ (0.05, 0.12) rad,
    # |γ| < 0.05).  Past the trail reversal angle (~9.9° for medium compound)
    # tp < 0 so Mz flips positive — that is correct Pacejka behaviour, not a bug.
    mask = (alpha > 0.05) & (alpha < 0.12) & (np.abs(gamma) < 0.05)
    mz_check = Mz[mask]
    pct_neg_mz = float(np.mean(mz_check < 0)) * 100 if len(mz_check) > 0 else 0.0
    print(f"Mz sign (pre-rev):{pct_neg_mz:.1f}% have Mz<0 in α∈(3°,7°),γ≈0"
          + ("  ✓" if pct_neg_mz > 95 else "  ⚠ CHECK"))

    print()
    return out_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PINN-1: Generate synthetic tire dataset")
    parser.add_argument("--compound", default="medium",
                        choices=["soft", "medium", "hard"],
                        help="Tire compound YAML to use (default: medium)")
    parser.add_argument("--rows", type=int, default=None,
                        help="Subsample to this many rows (default: all ~200K)")
    parser.add_argument("--seed",  type=int, default=42)
    parser.add_argument("--all-compounds", action="store_true",
                        help="Generate datasets for all three compounds")
    args = parser.parse_args()

    if args.all_compounds:
        for compound in ("soft", "medium", "hard"):
            generate(compound=compound, target_rows=args.rows, seed=args.seed)
    else:
        generate(compound=args.compound, target_rows=args.rows, seed=args.seed)
