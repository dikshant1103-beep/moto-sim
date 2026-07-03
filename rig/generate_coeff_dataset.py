#!/usr/bin/env python3
"""
rig/generate_coeff_dataset.py  —  Phase 3.1 dataset generation
───────────────────────────────────────────────────────────────
Generates training data for the PINN coefficient fitter.

Each example consists of:
  - A window of N_OBS = 64 on-vehicle observations at random operating points
    (Fz, α, κ, γ, T_tread) drawn from the physical riding envelope
  - Forces (Fx, Fy, Mz) computed from the FullMF formula with randomly-sampled
    coefficient scales applied to the nominal CBR600RR baseline
  - Target labels: the 6 scaling factors used to generate the data

Network input representation (permutation-invariant):
  For each of 8 normalised features [Fz, α, κ, γ, T, Fx, Fy, Mz]:
    → mean, std, min, max over the N_OBS window   = 32 statistics
  + current tire state [Fz_mean, T_mean]           =  2 scalars
  Total: 34 float features

Output (6 scale factors, all ≈ 1.0 at nominal):
  [mu_y_scale, p_Ky1_scale, E_Fy_scale, mu_x_scale, K_x_scale, tp0_scale]

Run:
    cd moto_sim
    python rig/generate_coeff_dataset.py              # ~100K examples
    python rig/generate_coeff_dataset.py --n 50000
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

ROOT   = Path(__file__).resolve().parent.parent
OUTDIR = ROOT / "dataset"

# ── Nominal FullMFCoeffs (CBR600RR baseline) ─────────────────────────────────
NOM = dict(
    Fz0    = 1470.0,   # N
    mu_y0  = 0.90,
    p_Ky1  = 13.5,
    p_Ky2  = 2.00,
    C_Fy   = 1.35,
    E_Fy   = -0.80,
    S_Vy0  = 0.0,
    mu_x0  = 0.95,
    K_x    = 11.0,
    tp0    = 0.030,
    Bt     = 10.0,
    Ct     = 1.50,
)

# ── Scale-factor sampling ranges ──────────────────────────────────────────────
SCALE_RANGES = {
    "mu_y_scale":   (0.45, 1.15),   # grip degradation: cold/worn → 0.45, fresh → 1.15
    "p_Ky1_scale":  (0.60, 1.40),   # lateral stiffness: worn tire looser
    "E_Fy_scale":   (0.70, 1.30),   # curvature: changes cornering character
    "mu_x_scale":   (0.45, 1.15),   # longitudinal grip
    "K_x_scale":    (0.70, 1.30),   # longitudinal stiffness
    "tp0_scale":    (0.60, 1.50),   # pneumatic trail: affects aligning torque
}

SCALE_KEYS = list(SCALE_RANGES.keys())
N_SCALES   = len(SCALE_KEYS)          # 6

# ── Operating-point sampling bounds ──────────────────────────────────────────
OP_BOUNDS = dict(
    Fz     = (400.0,  3000.0),   # N
    alpha  = (-0.32,  0.32),     # rad  (±18°)
    kappa  = (-0.40,  0.40),     # –
    gamma  = (-0.50,  0.50),     # rad  (±29° lean)
    T_tread= (20.0,   120.0),    # °C
)

N_OBS = 64    # observations per example
N_FEAT = 8    # features per observation before normalisation

# ── Normalisation constants ────────────────────────────────────────────────────
FEAT_CENTER = np.array([1470.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0], dtype=np.float32)
FEAT_SCALE  = np.array([ 750.0, 0.2, 0.25, 0.30, 50.0, 1500.0, 1500.0, 30.0], dtype=np.float32)


# ── Magic Formula functions ────────────────────────────────────────────────────

def fy_pure(alpha, Fz, mu_y, p_Ky1, C_Fy, E_Fy,
            Fz0=1470.0, p_Ky2=2.0):
    D = mu_y * Fz
    K_Fy = p_Ky1 * Fz0 * np.sin(2.0 * np.arctan(Fz / (p_Ky2 * Fz0 + 1e-9)))
    B = K_Fy / (C_Fy * D + 1e-8)
    phi = B * alpha - E_Fy * (B * alpha - np.arctan(B * alpha))
    return D * np.sin(C_Fy * np.arctan(phi))


def fx_pure(kappa, Fz, mu_x, K_x):
    return mu_x * Fz * np.tanh(K_x * kappa)


def mz_pneum(alpha, Fy, tp0, Bt=10.0, Ct=1.50):
    tp = tp0 * np.cos(Ct * np.arctan(Bt * alpha))
    return -Fy * tp


def combined_slip(alpha, kappa, Fz, mu_y, p_Ky1, C_Fy, E_Fy,
                  mu_x, K_x, tp0,
                  Bxa=10.0, Cxa=1.0, Byk=8.0, Cyk=1.0):
    Fy0 = fy_pure(alpha, Fz, mu_y, p_Ky1, C_Fy, E_Fy)
    Fx0 = fx_pure(kappa, Fz, mu_x, K_x)
    Gxa = np.cos(Cxa * np.arctan(Bxa * alpha))
    Gyk = np.cos(Cyk * np.arctan(Byk * kappa))
    Fx  = Gxa * Fx0
    Fy  = Gyk * Fy0
    Mz  = mz_pneum(alpha, Fy, tp0)
    return Fx, Fy, Mz


# ── Per-example generation ─────────────────────────────────────────────────────

def make_example(rng: np.random.Generator):
    # ── Sample scale factors ─────────────────────────────────────────────────
    scales = np.array([
        rng.uniform(*SCALE_RANGES[k]) for k in SCALE_KEYS
    ], dtype=np.float32)

    mu_y_s, pKy1_s, E_s, mu_x_s, Kx_s, tp0_s = scales

    # Effective coefficients
    mu_y  = NOM["mu_y0"]  * mu_y_s
    p_Ky1 = NOM["p_Ky1"]  * pKy1_s
    E_Fy  = NOM["E_Fy"]   * E_s
    mu_x  = NOM["mu_x0"]  * mu_x_s
    K_x   = NOM["K_x"]    * Kx_s
    tp0   = NOM["tp0"]    * tp0_s

    # ── Sample operating points ──────────────────────────────────────────────
    Fz     = rng.uniform(*OP_BOUNDS["Fz"],     N_OBS).astype(np.float32)
    alpha  = rng.uniform(*OP_BOUNDS["alpha"],  N_OBS).astype(np.float32)
    kappa  = rng.uniform(*OP_BOUNDS["kappa"],  N_OBS).astype(np.float32)
    gamma  = rng.uniform(*OP_BOUNDS["gamma"],  N_OBS).astype(np.float32)
    T      = rng.uniform(*OP_BOUNDS["T_tread"],N_OBS).astype(np.float32)

    # ── Compute forces ────────────────────────────────────────────────────────
    Fx, Fy, Mz = combined_slip(alpha, kappa, Fz, mu_y, p_Ky1,
                                NOM["C_Fy"], E_Fy, mu_x, K_x, tp0)

    # ── Add sensor noise (SNR ≈ 40 dB) ───────────────────────────────────────
    Fx += rng.normal(0, 15.0, N_OBS)
    Fy += rng.normal(0, 15.0, N_OBS)
    Mz += rng.normal(0,  0.5, N_OBS)

    # ── Stack into observation matrix [N_OBS × N_FEAT] ───────────────────────
    obs = np.stack([Fz, alpha, kappa, gamma, T, Fx, Fy, Mz], axis=1)  # (64, 8)

    # ── Normalise ─────────────────────────────────────────────────────────────
    obs_n = (obs - FEAT_CENTER) / (FEAT_SCALE + 1e-9)

    # ── Permutation-invariant summary statistics ──────────────────────────────
    # For each feature: [mean, std, min, max]  →  4 × 8 = 32 scalars
    feat_mean = obs_n.mean(axis=0)                # (8,)
    feat_std  = obs_n.std(axis=0)                 # (8,)
    feat_min  = obs_n.min(axis=0)                 # (8,)
    feat_max  = obs_n.max(axis=0)                 # (8,)
    summary   = np.concatenate([feat_mean, feat_std, feat_min, feat_max])  # (32,)

    # Two extra scalars: mean load (captures Fz level) + mean temperature
    extra = np.array([(Fz.mean() - 1470.0) / 750.0,
                      (T.mean()  -    70.0) /  50.0], dtype=np.float32)

    x = np.concatenate([summary, extra]).astype(np.float32)   # (34,)

    return x, scales


# ── Batch generation ──────────────────────────────────────────────────────────

def generate(n: int, seed: int = 42) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    t0  = time.perf_counter()

    X = np.empty((n, 34),       dtype=np.float32)
    Y = np.empty((n, N_SCALES), dtype=np.float32)

    for i in range(n):
        X[i], Y[i] = make_example(rng)
        if (i + 1) % 10000 == 0:
            elapsed = time.perf_counter() - t0
            rate    = (i + 1) / elapsed
            print(f"  {i+1:>7}/{n}  {rate:.0f} ex/s  eta {(n-i-1)/rate:.0f}s")

    return X, Y


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Phase 3.1 coefficient-fitter dataset")
    parser.add_argument("--n",    type=int, default=120_000, help="Number of examples")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out",  type=str, default="")
    args = parser.parse_args()

    OUTDIR.mkdir(exist_ok=True)
    out = Path(args.out) if args.out else OUTDIR / "coeff_fitter_dataset.npz"

    print(f"Generating {args.n:,} examples  →  {out}")
    print(f"  Input size : 34  |  Output size: {N_SCALES}")
    print(f"  Scale keys : {SCALE_KEYS}")

    X, Y = generate(args.n, args.seed)

    np.savez_compressed(str(out), X=X, Y=Y,
                        scale_keys=np.array(SCALE_KEYS),
                        feat_center=FEAT_CENTER,
                        feat_scale=FEAT_SCALE)

    print(f"\nSaved  X:{X.shape}  Y:{Y.shape}  →  {out}")
    print(f"  Y ranges: " + " ".join(
        f"{SCALE_KEYS[i]}=[{Y[:,i].min():.2f},{Y[:,i].max():.2f}]"
        for i in range(N_SCALES)
    ))

if __name__ == "__main__":
    main()
