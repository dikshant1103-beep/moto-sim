#!/usr/bin/env python3
"""
rig/train_coeff_fitter.py  —  Phase 3.1: PINN Coefficient Fitter training
──────────────────────────────────────────────────────────────────────────
Trains a neural network that takes a window of on-vehicle telemetry and
outputs FullMFTire coefficient scale factors in real-time.

Architecture:  MLP  34 → 128 → 64 → 32 → 6
  Input  (34) : permutation-invariant statistics over a 64-frame window
                [mean, std, min, max] × 8 features + [Fz_mean, T_mean]
  Output (6)  : scale factors  [mu_y, p_Ky1, E_Fy, mu_x, K_x, tp0]
                each ≈ 1.0 at nominal; sigmoid-bounded to [0.35, 1.55]

Physics constraints in loss:
  1. Friction ellipse: (Fx/µFz)² + (Fy/µFz)² ≤ 1  (soft penalty)
  2. Monotonicity:     ∂Fy/∂α ≥ 0  for α ∈ [0, α_peak]  (soft penalty)

Export:  dataset/coeff_fitter.onnx  (opset 17)

Run:
    cd moto_sim
    python rig/generate_coeff_dataset.py      # generate data first
    python rig/train_coeff_fitter.py
    python rig/train_coeff_fitter.py --epochs 200 --lr 1e-3
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset, random_split

ROOT   = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

OUTDIR = ROOT / "dataset"

# ── Scale-factor bounds (must match generate_coeff_dataset.py) ───────────────
SCALE_MIN = torch.tensor([0.45, 0.60, 0.70, 0.45, 0.70, 0.60], dtype=torch.float32)
SCALE_MAX = torch.tensor([1.15, 1.40, 1.30, 1.15, 1.30, 1.50], dtype=torch.float32)
SCALE_KEYS= ["mu_y", "p_Ky1", "E_Fy", "mu_x", "K_x", "tp0"]

# Nominal coefficients for physics-penalty forward pass
NOM = dict(mu_y0=0.90, p_Ky1=13.5, p_Ky2=2.00, C_Fy=1.35,
           E_Fy=-0.80, mu_x0=0.95, K_x=11.0, tp0=0.030, Fz0=1470.0)


# ── Magic Formula (differentiable, PyTorch) ────────────────────────────────────

def mf_fy(alpha, Fz, mu_y, p_Ky1, C_Fy, E_Fy, Fz0=1470.0, p_Ky2=2.0):
    D    = mu_y * Fz
    K_Fy = p_Ky1 * Fz0 * torch.sin(2.0 * torch.atan(Fz / (p_Ky2 * Fz0 + 1e-9)))
    B    = K_Fy / (C_Fy * D + 1e-8)
    phi  = B * alpha - E_Fy * (B * alpha - torch.atan(B * alpha))
    return D * torch.sin(C_Fy * torch.atan(phi))


def mf_fx(kappa, Fz, mu_x, K_x):
    return mu_x * Fz * torch.tanh(K_x * kappa)


def physics_penalties(scales_pred: torch.Tensor, device) -> torch.Tensor:
    """
    Compute soft penalties for physics violations on random operating points.
    Returns scalar penalty (higher = worse).
    """
    B = scales_pred.shape[0]
    alpha  = torch.linspace(-0.25, 0.25, 20, device=device).unsqueeze(0).expand(B, -1)  # (B,20)
    Fz_ref = torch.full((B, 20), 1470.0, device=device)
    kappa  = torch.linspace(-0.3, 0.3, 10, device=device).unsqueeze(0).expand(B, -1)

    mu_y  = NOM["mu_y0"]  * scales_pred[:, 0].unsqueeze(1)
    p_Ky1 = NOM["p_Ky1"]  * scales_pred[:, 1].unsqueeze(1)
    E_Fy  = NOM["E_Fy"]   * scales_pred[:, 2].unsqueeze(1)
    mu_x  = NOM["mu_x0"]  * scales_pred[:, 3].unsqueeze(1)
    K_x   = NOM["K_x"]    * scales_pred[:, 4].unsqueeze(1)

    # 1. Friction-ellipse penalty at combined slip
    Fy = mf_fy(alpha[:, :10], Fz_ref[:, :10], mu_y, p_Ky1, NOM["C_Fy"], E_Fy)
    Fx = mf_fx(kappa,         Fz_ref[:, :10], mu_x, K_x)
    mu_Fz  = mu_y * Fz_ref[:, :10]
    ellipse_ratio = (Fx / (mu_Fz + 1e-9))**2 + (Fy / (mu_Fz + 1e-9))**2
    friction_pen  = torch.relu(ellipse_ratio - 1.0).mean()

    # 2. Monotonicity: dFy/dα ≥ 0 for α ∈ [0, 0.15 rad] (slope of MF before peak)
    alpha_mono = torch.linspace(0.001, 0.15, 15, device=device).unsqueeze(0).expand(B, -1)
    Fy_mono    = mf_fy(alpha_mono, Fz_ref[:, :15], mu_y, p_Ky1, NOM["C_Fy"], E_Fy)
    dFy        = Fy_mono[:, 1:] - Fy_mono[:, :-1]
    mono_pen   = torch.relu(-dFy).mean()

    return 0.1 * friction_pen + 0.05 * mono_pen


# ── Model ─────────────────────────────────────────────────────────────────────

class CoeffFitterMLP(nn.Module):
    """
    MLP coefficient fitter.
    Input:  34-dim statistics vector
    Output: 6 scale factors in (SCALE_MIN, SCALE_MAX) via sigmoid mapping
    """
    def __init__(self):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(34, 128), nn.LayerNorm(128), nn.GELU(),
            nn.Dropout(0.10),
            nn.Linear(128, 64), nn.LayerNorm(64), nn.GELU(),
            nn.Dropout(0.10),
            nn.Linear(64, 32),  nn.GELU(),
            nn.Linear(32, 6),
        )
        # Initialise last layer near zero → outputs near sigmoid(0) = 0.5 → midpoint of bounds
        nn.init.zeros_(self.encoder[-1].bias)
        nn.init.normal_(self.encoder[-1].weight, std=0.01)

    def forward(self, x):
        raw    = self.encoder(x)                          # (B, 6) unbounded
        sig    = torch.sigmoid(raw)                       # (B, 6) ∈ (0, 1)
        scales = SCALE_MIN.to(x.device) + sig * (SCALE_MAX - SCALE_MIN).to(x.device)
        return scales                                     # (B, 6) in physical range


# ── Training ──────────────────────────────────────────────────────────────────

def train(args):
    OUTDIR.mkdir(exist_ok=True)
    data_path = OUTDIR / "coeff_fitter_dataset.npz"

    if not data_path.exists():
        print(f"Dataset not found: {data_path}")
        print("Run:  python rig/generate_coeff_dataset.py")
        sys.exit(1)

    print(f"Loading dataset: {data_path}")
    npz = np.load(str(data_path))
    X   = torch.from_numpy(npz["X"])   # (N, 34)
    Y   = torch.from_numpy(npz["Y"])   # (N, 6)
    print(f"  X: {X.shape}  Y: {Y.shape}")

    # Train / val split (90 / 10)
    n_val   = max(1000, int(len(X) * 0.10))
    n_train = len(X) - n_val
    ds_full = TensorDataset(X, Y)
    ds_tr, ds_va = random_split(ds_full, [n_train, n_val],
                                generator=torch.Generator().manual_seed(42))

    dl_tr = DataLoader(ds_tr, batch_size=args.batch, shuffle=True,  num_workers=2, pin_memory=True)
    dl_va = DataLoader(ds_va, batch_size=args.batch, shuffle=False, num_workers=2, pin_memory=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"  Device: {device}  |  train={n_train}  val={n_val}")

    model = CoeffFitterMLP().to(device)
    SCALE_MIN_d = SCALE_MIN.to(device)
    SCALE_MAX_d = SCALE_MAX.to(device)

    opt = optim.Adam(model.parameters(), lr=args.lr, weight_decay=1e-5)
    sched = optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs, eta_min=args.lr * 0.05)

    best_val = float("inf")
    best_state = None

    print(f"\nEpoch  Train-MSE  Val-MSE   MAE-mu_y  MAE-Ky1  LR")
    print("─" * 62)

    for epoch in range(1, args.epochs + 1):
        # ── Train ────────────────────────────────────────────────────────────
        model.train()
        tr_loss = 0.0
        for xb, yb in dl_tr:
            xb, yb = xb.to(device), yb.to(device)
            pred   = model(xb)
            # Normalise targets and predictions to [0,1] for MSE
            pred_n = (pred - SCALE_MIN_d) / (SCALE_MAX_d - SCALE_MIN_d)
            yb_n   = (yb   - SCALE_MIN_d) / (SCALE_MAX_d - SCALE_MIN_d)
            mse    = nn.functional.mse_loss(pred_n, yb_n)
            pen    = physics_penalties(pred, device)
            loss   = mse + pen
            opt.zero_grad(); loss.backward(); opt.step()
            tr_loss += mse.item()
        tr_loss /= len(dl_tr)

        # ── Validate ─────────────────────────────────────────────────────────
        model.eval()
        val_loss = 0.0
        mae_buf  = torch.zeros(6, device=device)
        with torch.no_grad():
            for xb, yb in dl_va:
                xb, yb = xb.to(device), yb.to(device)
                pred   = model(xb)
                pred_n = (pred - SCALE_MIN_d) / (SCALE_MAX_d - SCALE_MIN_d)
                yb_n   = (yb   - SCALE_MIN_d) / (SCALE_MAX_d - SCALE_MIN_d)
                val_loss += nn.functional.mse_loss(pred_n, yb_n).item()
                mae_buf  += (pred - yb).abs().mean(dim=0)
        val_loss /= len(dl_va)
        mae_buf  /= len(dl_va)
        sched.step()

        if val_loss < best_val:
            best_val   = val_loss
            best_state = {k: v.cpu() for k, v in model.state_dict().items()}

        if epoch % 10 == 0 or epoch == 1:
            lr_now = opt.param_groups[0]["lr"]
            print(f"  {epoch:>4}   {tr_loss:.6f}   {val_loss:.6f}   "
                  f"{mae_buf[0]:.4f}    {mae_buf[1]:.4f}   {lr_now:.2e}")

    # ── Load best and export ─────────────────────────────────────────────────
    print(f"\nBest val MSE (normalised): {best_val:.6f}")
    model.load_state_dict(best_state)
    model.eval().to("cpu")

    # PyTorch save
    pt_path = OUTDIR / "coeff_fitter.pt"
    torch.save({
        "model_state":  model.state_dict(),
        "scale_min":    SCALE_MIN.numpy(),
        "scale_max":    SCALE_MAX.numpy(),
        "scale_keys":   SCALE_KEYS,
        "input_size":   34,
        "output_size":  6,
        "best_val_mse": best_val,
    }, str(pt_path))
    print(f"Saved PT  → {pt_path}")

    # ONNX export
    onnx_path = OUTDIR / "coeff_fitter.onnx"
    dummy = torch.zeros(1, 34)
    torch.onnx.export(
        model, dummy, str(onnx_path),
        input_names=["telemetry_stats"],
        output_names=["coeff_scales"],
        dynamic_axes={"telemetry_stats": {0: "batch"}, "coeff_scales": {0: "batch"}},
        opset_version=17,
    )
    print(f"Saved ONNX → {onnx_path}")

    # Verify ONNX
    try:
        import onnxruntime as ort
        sess  = ort.InferenceSession(str(onnx_path),
                                     providers=["CPUExecutionProvider"])
        out   = sess.run(None, {"telemetry_stats": np.zeros((1, 34), np.float32)})[0]
        print(f"ONNX verified: output={out.flatten()}")
        print(f"  (should be close to nominal scale = 1.0 everywhere)")
    except Exception as e:
        print(f"ONNX verify: {e}")

    # Final per-coefficient MAE summary
    model.eval()
    all_pred, all_true = [], []
    dl_va2 = DataLoader(ds_va, batch_size=2048, shuffle=False)
    with torch.no_grad():
        for xb, yb in dl_va2:
            all_pred.append(model(xb).numpy())
            all_true.append(yb.numpy())
    P = np.concatenate(all_pred); T = np.concatenate(all_true)
    mae = np.abs(P - T).mean(axis=0)
    print("\nFinal val MAE per coefficient:")
    for k, m in zip(SCALE_KEYS, mae):
        pct = m / (SCALE_MAX.numpy()[SCALE_KEYS.index(k)] -
                   SCALE_MIN.numpy()[SCALE_KEYS.index(k)]) * 100
        gate = "PASS" if pct < 8.0 else "FAIL"
        print(f"  {k:12s}  MAE={m:.4f}  ({pct:.1f}% of range)  [{gate}]")


def main():
    p = argparse.ArgumentParser(description="Phase 3.1 coefficient fitter training")
    p.add_argument("--epochs", type=int,   default=150)
    p.add_argument("--batch",  type=int,   default=512)
    p.add_argument("--lr",     type=float, default=1e-3)
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
