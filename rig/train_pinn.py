#!/usr/bin/env python3
"""
rig/train_pinn.py  —  PINN-2: Neural network training
───────────────────────────────────────────────────────
Trains a Physics-Informed Neural Network (PINN) on the synthetic Pacejka
dataset from PINN-1 (generate_dataset.py).

Architecture:   4 → 64 → 64 → 32 → 3   (Tanh activations)
Inputs:         [Fz [N], alpha [rad], kappa [–], gamma [rad]]
Outputs:        [Fx [N], Fy [N], Mz [Nm]]
Loss:           MSE + λ_fric · friction-ellipse penalty + λ_sym · symmetry loss
Export:         dataset/tire_pinn_<compound>.onnx

Physics constraints enforced during training:
  1. Friction ellipse: (Fx/µFz)² + (Fy/µFz)² ≤ 1  (soft penalty)
  2. Odd symmetry:     Fx(-κ,α,γ) = -Fx(κ,α,γ),
                       Fy(-α,-κ,-γ) = -Fy(α,κ,γ),
                       Mz(-α,-κ,-γ) = -Mz(α,κ,γ)  (soft penalty)

Run from moto_sim/ root:
    python rig/train_pinn.py                        # medium compound
    python rig/train_pinn.py --compound soft
    python rig/train_pinn.py --epochs 300 --lr 5e-4
    python rig/train_pinn.py --all-compounds
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
from torch.utils.data import DataLoader, TensorDataset, random_split

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

DATASET_DIR = ROOT / "dataset"
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ── Input / output normalisation constants (fixed, not data-dependent) ─────────
# Using known physical bounds → normalises to approx [-1, 1]
IN_CENTER  = torch.tensor([1750.0, 0.0,  0.0,  0.0],  dtype=torch.float32)
IN_SCALE   = torch.tensor([ 750.0, 0.175, 0.25, 0.20], dtype=torch.float32)
# Output scale (approximate peak values per compound)
# Set conservatively; actual peaks are compound-specific but close enough
OUT_SCALE  = torch.tensor([3000.0, 3000.0, 65.0], dtype=torch.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Model
# ─────────────────────────────────────────────────────────────────────────────

class TirePINN(nn.Module):
    """
    4 → 64 → 64 → 32 → 3  MLP with Tanh activations.

    Operates in NORMALISED space internally; norm / denorm helpers are
    stored as buffers so they travel with the model when saved/loaded.
    """

    def __init__(self,
                 in_center: torch.Tensor  = IN_CENTER,
                 in_scale:  torch.Tensor  = IN_SCALE,
                 out_scale: torch.Tensor  = OUT_SCALE,
                 mu:        float          = 0.90) -> None:
        super().__init__()
        self.register_buffer("in_center", in_center.clone())
        self.register_buffer("in_scale",  in_scale.clone())
        self.register_buffer("out_scale", out_scale.clone())
        self.mu = mu   # peak friction coefficient (used in physics penalty)

        self.net = nn.Sequential(
            nn.Linear(4,  64),  nn.Tanh(),
            nn.Linear(64, 64),  nn.Tanh(),
            nn.Linear(64, 32),  nn.Tanh(),
            nn.Linear(32,  3),
        )
        self._init_weights()

    def _init_weights(self) -> None:
        for m in self.net.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
                nn.init.zeros_(m.bias)

    # ── normalisation helpers ─────────────────────────────────────────────────
    def normalise_in(self, x: torch.Tensor) -> torch.Tensor:
        return (x - self.in_center) / self.in_scale

    def denormalise_out(self, y: torch.Tensor) -> torch.Tensor:
        return y * self.out_scale

    # ── forward  (input and output in physical units) ─────────────────────────
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: [..., 4] physical → returns [..., 3] physical (Fx, Fy, Mz)."""
        x_norm = self.normalise_in(x)
        y_norm = self.net(x_norm)
        return self.denormalise_out(y_norm)

    # ── inference convenience ─────────────────────────────────────────────────
    @torch.no_grad()
    def predict(self, Fz: float, alpha: float, kappa: float,
                gamma: float) -> tuple[float, float, float]:
        x = torch.tensor([[Fz, alpha, kappa, gamma]],
                          dtype=torch.float32, device=self.in_center.device)
        y = self.forward(x)[0]
        return float(y[0]), float(y[1]), float(y[2])


# ─────────────────────────────────────────────────────────────────────────────
# Loss
# ─────────────────────────────────────────────────────────────────────────────

def physics_loss(model: TirePINN,
                 x: torch.Tensor,
                 y_pred: torch.Tensor,
                 out_scale: torch.Tensor,
                 lambda_fric: float,
                 lambda_sym:  float) -> tuple[torch.Tensor, dict]:
    """
    Physics penalty terms (all dimensionless — same scale as normalised MSE):
      L_fric  — soft friction-ellipse hinge penalty
      L_sym   — odd-symmetry penalty in normalised output space
    """
    extras: dict[str, float] = {}
    loss = torch.zeros(1, device=x.device)

    Fz = x[:, 0:1]
    Fx = y_pred[:, 0:1]
    Fy = y_pred[:, 1:2]

    # ── Friction ellipse (dimensionless) ──────────────────────────────────────
    # (Fx/µFz)² + (Fy/µFz)² ≤ 1 — penalise hinge-squared violations only
    if lambda_fric > 0:
        denom = (model.mu * Fz).clamp(min=1.0)
        ellipse = (Fx / denom) ** 2 + (Fy / denom) ** 2
        L_fric  = torch.mean(torch.relu(ellipse - 1.0) ** 2)
        loss    = loss + lambda_fric * L_fric
        extras["L_fric"] = float(L_fric)

    # ── Odd-symmetry penalty in NORMALISED space ───────────────────────────────
    # Normalise first so L_sym is ~O(1), same magnitude as MSE.
    # Without normalisation, L_sym is in N² (≫1) and overwhelms MSE entirely.
    if lambda_sym > 0:
        y_norm = y_pred / out_scale          # [N, 3] in [-1, 1] approx

        # Fx odd in κ: Fx(Fz,α,-κ,γ) = -Fx(Fz,α,κ,γ)
        x_fk = x.clone(); x_fk[:, 2] = -x[:, 2]
        y_fk = model(x_fk) / out_scale
        L_sym_Fx = torch.mean((y_norm[:, 0] + y_fk[:, 0]) ** 2)

        # Fy and Mz odd in (α,κ,γ): f(Fz,-α,-κ,-γ) = -f(Fz,α,κ,γ)
        x_fa = x.clone()
        x_fa[:, 1] = -x[:, 1]; x_fa[:, 2] = -x[:, 2]; x_fa[:, 3] = -x[:, 3]
        y_fa = model(x_fa) / out_scale
        L_sym_Fy = torch.mean((y_norm[:, 1] + y_fa[:, 1]) ** 2)
        L_sym_Mz = torch.mean((y_norm[:, 2] + y_fa[:, 2]) ** 2)

        L_sym = (L_sym_Fx + L_sym_Fy + L_sym_Mz) / 3.0
        loss  = loss + lambda_sym * L_sym
        extras["L_sym"] = float(L_sym)

    return loss, extras


# ─────────────────────────────────────────────────────────────────────────────
# Training
# ─────────────────────────────────────────────────────────────────────────────

def train(compound:     str   = "medium",
          epochs:       int   = 250,
          batch_size:   int   = 4096,
          lr:           float = 1e-3,
          lambda_fric:  float = 0.01,
          lambda_sym:   float = 0.0,
          val_split:    float = 0.10,
          test_split:   float = 0.10,
          seed:         int   = 42) -> Path:

    torch.manual_seed(seed)
    np.random.seed(seed)

    print(f"\n=== PINN-2: Neural Network Training ===")
    print(f"Compound  : {compound}")
    print(f"Device    : {DEVICE}  ({'CUDA ' + torch.cuda.get_device_name(0) if DEVICE.type == 'cuda' else 'CPU'})")
    print(f"Arch      : 4 → 64 → 64 → 32 → 3  (Tanh)")
    print(f"Epochs    : {epochs}  |  Batch : {batch_size}  |  LR : {lr}")
    print(f"λ_fric    : {lambda_fric}  |  λ_sym : {lambda_sym}")

    # ── Load dataset ──────────────────────────────────────────────────────────
    csv_path = DATASET_DIR / f"tire_synthetic_{compound}.csv"
    assert csv_path.exists(), f"Dataset not found: {csv_path}  (run generate_dataset.py first)"

    print(f"\nLoading {csv_path.name} ...", flush=True)
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1, dtype=np.float32)
    X_np = data[:, :4]   # Fz, alpha, kappa, gamma
    Y_np = data[:, 4:]   # Fx, Fy, Mz
    print(f"  Loaded {len(X_np):,} rows")

    X = torch.from_numpy(X_np)
    Y = torch.from_numpy(Y_np)

    # ── Train / val / test split ──────────────────────────────────────────────
    n       = len(X)
    n_test  = int(n * test_split)
    n_val   = int(n * val_split)
    n_train = n - n_val - n_test
    ds      = TensorDataset(X, Y)
    ds_train, ds_val, ds_test = random_split(
        ds, [n_train, n_val, n_test],
        generator=torch.Generator().manual_seed(seed),
    )
    print(f"  Split   : train={n_train:,}  val={n_val:,}  test={n_test:,}")

    train_loader = DataLoader(ds_train, batch_size=batch_size, shuffle=True,
                              num_workers=0, pin_memory=(DEVICE.type == "cuda"))
    val_loader   = DataLoader(ds_val,   batch_size=batch_size * 4, shuffle=False,
                              num_workers=0, pin_memory=(DEVICE.type == "cuda"))
    test_loader  = DataLoader(ds_test,  batch_size=batch_size * 4, shuffle=False,
                              num_workers=0, pin_memory=(DEVICE.type == "cuda"))

    # ── Model ─────────────────────────────────────────────────────────────────
    # Read compound mu for physics penalty
    from core.tire.pinn_model import TireCoeffs
    coeffs = TireCoeffs.from_yaml(ROOT / "core/tire/pacejka_coeffs" / f"{compound}.yaml")

    model = TirePINN(mu=coeffs.mu).to(DEVICE)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"  Params  : {n_params:,}")

    # ── Optimiser + scheduler ─────────────────────────────────────────────────
    opt       = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs, eta_min=lr * 0.01)
    mse_fn    = nn.MSELoss()

    # Normalise Y for MSE so Fx/Fy/Mz contribute equally
    out_scale_dev = OUT_SCALE.to(DEVICE)

    # ── Training loop ─────────────────────────────────────────────────────────
    best_val_loss = float("inf")
    best_state    = None
    history       = {"train": [], "val": []}

    print(f"\n{'Epoch':>6}  {'Train MSE':>12}  {'Val MSE':>10}  "
          f"{'L_fric':>8}  {'L_sym':>7}  {'LR':>8}  {'Time':>6}")
    print("─" * 72)

    t0 = time.perf_counter()

    for epoch in range(1, epochs + 1):
        # ── Train ─────────────────────────────────────────────────────────────
        model.train()
        train_mse_accum = 0.0
        fric_accum = sym_accum = 0.0
        n_batches = 0

        for xb, yb in train_loader:
            xb, yb = xb.to(DEVICE), yb.to(DEVICE)
            opt.zero_grad()

            y_pred = model(xb)

            # Normalised MSE (equal weight per output channel)
            mse = mse_fn(y_pred / out_scale_dev, yb / out_scale_dev)

            # Physics penalties
            phy, extras = physics_loss(model, xb, y_pred,
                                       out_scale_dev, lambda_fric, lambda_sym)
            loss = mse + phy

            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()

            train_mse_accum += float(mse)
            fric_accum       += extras.get("L_fric", 0.0)
            sym_accum        += extras.get("L_sym",  0.0)
            n_batches        += 1

        scheduler.step()
        train_mse = train_mse_accum / n_batches
        fric_mean = fric_accum / n_batches
        sym_mean  = sym_accum  / n_batches

        # ── Validate ──────────────────────────────────────────────────────────
        model.eval()
        val_mse_accum = 0.0
        val_batches   = 0
        with torch.no_grad():
            for xb, yb in val_loader:
                xb, yb = xb.to(DEVICE), yb.to(DEVICE)
                y_pred = model(xb)
                val_mse_accum += float(mse_fn(y_pred / out_scale_dev,
                                               yb / out_scale_dev))
                val_batches += 1
        val_mse = val_mse_accum / val_batches

        history["train"].append(train_mse)
        history["val"].append(val_mse)

        if val_mse < best_val_loss:
            best_val_loss = val_mse
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}

        # ── Log every 10 epochs ───────────────────────────────────────────────
        if epoch % 10 == 0 or epoch == 1:
            elapsed = time.perf_counter() - t0
            current_lr = scheduler.get_last_lr()[0]
            print(f"{epoch:>6}  {train_mse:>12.6f}  {val_mse:>10.6f}  "
                  f"{fric_mean:>8.5f}  {sym_mean:>7.5f}  "
                  f"{current_lr:>8.2e}  {elapsed:>5.1f}s")

    total_time = time.perf_counter() - t0
    print(f"\nTraining complete in {total_time:.1f} s")
    print(f"Best val MSE (normalised): {best_val_loss:.6f}")

    # ── Restore best weights ──────────────────────────────────────────────────
    model.load_state_dict({k: v.to(DEVICE) for k, v in best_state.items()})

    # ── Test-set evaluation ───────────────────────────────────────────────────
    model.eval()
    Fx_err, Fy_err, Mz_err = [], [], []
    fric_viols = 0
    total_test = 0
    with torch.no_grad():
        for xb, yb in test_loader:
            xb, yb = xb.to(DEVICE), yb.to(DEVICE)
            yp = model(xb)
            err = (yp - yb).abs()
            Fx_err.append(err[:, 0].cpu())
            Fy_err.append(err[:, 1].cpu())
            Mz_err.append(err[:, 2].cpu())
            Fz_b = xb[:, 0]
            ellipse = ((yp[:, 0] / (model.mu * Fz_b)).pow(2)
                       + (yp[:, 1] / (model.mu * Fz_b)).pow(2))
            fric_viols += int((ellipse > 1.0).sum())
            total_test += len(xb)

    Fx_mae = float(torch.cat(Fx_err).mean())
    Fy_mae = float(torch.cat(Fy_err).mean())
    Mz_mae = float(torch.cat(Mz_err).mean())
    fric_pct = (1 - fric_viols / total_test) * 100

    print(f"\n── Test set ({total_test:,} rows) ──────────────────────────────")
    print(f"  MAE  Fx : {Fx_mae:7.2f} N")
    print(f"  MAE  Fy : {Fy_mae:7.2f} N")
    print(f"  MAE  Mz : {Mz_mae:7.3f} Nm")
    print(f"  Friction ellipse compliance: {fric_pct:.2f}%")

    # ── Mz reversal validation ────────────────────────────────────────────────
    print(f"\n── Physics validation ───────────────────────────────────────────")
    alpha_sweep = torch.linspace(0.0, math.radians(25), 500)
    Fz_test     = torch.full_like(alpha_sweep, 1470.0)
    kappa_test  = torch.zeros_like(alpha_sweep)
    gamma_test  = torch.zeros_like(alpha_sweep)
    x_sweep     = torch.stack([Fz_test, alpha_sweep, kappa_test, gamma_test], dim=1).to(DEVICE)

    with torch.no_grad():
        y_sweep = model(x_sweep).cpu()

    Mz_sweep = y_sweep[:, 2].numpy()
    from core.tire.pinn_model import DummyTirePINN
    pinn_ref     = DummyTirePINN.from_coeffs(coeffs)
    alpha_rev_ref = pinn_ref.trail_reversal_deg()
    # Skip first 2° (~40 pts) where Mz≈0 by symmetry to avoid false np.sign(0) triggers.
    # Look for the neg→pos transition only (physical trail reversal).
    skip = 40
    Mz_post    = Mz_sweep[skip:]
    alpha_post  = alpha_sweep[skip:].numpy() * 180 / math.pi
    sgn         = np.sign(Mz_post)
    sgn[sgn == 0] = -1          # treat exact zero as still-negative
    transitions = np.where(np.diff(sgn) > 0)[0]
    if len(transitions) > 0:
        alpha_rev_nn = float(alpha_post[transitions[0]])
        err_rev = abs(alpha_rev_nn - alpha_rev_ref)
        print(f"  Mz reversal (NN):  {alpha_rev_nn:.1f}°  |  ref: {alpha_rev_ref:.1f}°  "
              f"|  err: {err_rev:.2f}°" + ("  ✓" if err_rev < 2.0 else "  ⚠"))
    else:
        print(f"  Mz reversal: not detected in 2–25° sweep (ref: {alpha_rev_ref:.1f}°)  ⚠")

    # Find Fy peak slip angle
    Fy_sweep = y_sweep[:, 1].numpy()
    peak_idx = int(np.argmax(np.abs(Fy_sweep)))
    alpha_peak_nn = float(alpha_sweep[peak_idx]) * 180 / math.pi
    from core.tire.pinn_model import DummyTirePINN as _DPINN
    pinn_ref2 = _DPINN.from_coeffs(coeffs)
    alpha_peak_ref = pinn_ref2.peak_slip_angle_deg(Fz=1470.0)
    err_peak = abs(alpha_peak_nn - alpha_peak_ref)
    print(f"  Fy peak (NN):  {alpha_peak_nn:.1f}°  |  ref: {alpha_peak_ref:.1f}°  "
          f"|  err: {err_peak:.2f}°" + ("  ✓" if err_peak < 2.0 else "  ⚠"))

    # ── Export to ONNX ────────────────────────────────────────────────────────
    model.cpu().eval()
    onnx_path = DATASET_DIR / f"tire_pinn_{compound}.onnx"
    dummy_in  = torch.randn(1, 4)
    torch.onnx.export(
        model, dummy_in, str(onnx_path),
        input_names  = ["tire_input"],
        output_names = ["tire_output"],
        dynamic_axes = {"tire_input":  {0: "batch"},
                        "tire_output": {0: "batch"}},
        opset_version= 17,
    )

    # Verify ONNX model
    import onnx, onnxruntime as ort
    onnx_model = onnx.load(str(onnx_path))
    onnx.checker.check_model(onnx_model)
    sess = ort.InferenceSession(str(onnx_path))
    test_in  = np.array([[1470.0, 0.087, 0.0, 0.0]], dtype=np.float32)
    ort_out  = sess.run(None, {"tire_input": test_in})[0]
    pt_out   = model(torch.from_numpy(test_in)).detach().numpy()
    max_diff = float(np.max(np.abs(ort_out - pt_out)))
    print(f"\n── ONNX export ─────────────────────────────────────────────────")
    print(f"  Saved  : {onnx_path}")
    print(f"  Size   : {onnx_path.stat().st_size / 1e3:.1f} KB")
    print(f"  PT vs ORT max diff: {max_diff:.2e}" + ("  ✓" if max_diff < 1e-4 else "  ⚠"))
    print(f"  Sample  [Fz=1470, α=5°, κ=0, γ=0]:")
    print(f"    Fx={ort_out[0,0]:+.2f} N  Fy={ort_out[0,1]:+.2f} N  Mz={ort_out[0,2]:+.3f} Nm")

    # ── Save PyTorch weights too ──────────────────────────────────────────────
    pt_path = DATASET_DIR / f"tire_pinn_{compound}.pt"
    torch.save({"model_state": model.state_dict(),
                "in_center":   IN_CENTER,
                "in_scale":    IN_SCALE,
                "out_scale":   OUT_SCALE,
                "mu":          coeffs.mu,
                "compound":    compound,
                "val_mse":     best_val_loss,
                "Fx_mae":      Fx_mae,
                "Fy_mae":      Fy_mae,
                "Mz_mae":      Mz_mae}, pt_path)
    print(f"  Weights: {pt_path}")
    print()

    return onnx_path


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PINN-2: Train tire neural network")
    parser.add_argument("--compound", default="medium",
                        choices=["soft", "medium", "hard"])
    parser.add_argument("--epochs",      type=int,   default=250)
    parser.add_argument("--batch-size",  type=int,   default=4096)
    parser.add_argument("--lr",          type=float, default=1e-3)
    parser.add_argument("--lambda-fric", type=float, default=0.01,
                        help="Friction-ellipse penalty weight (default 0.01)")
    parser.add_argument("--lambda-sym",  type=float, default=0.0,
                        help="Symmetry penalty weight; 0=disabled (symmetry from data)")
    parser.add_argument("--seed",        type=int,   default=42)
    parser.add_argument("--all-compounds", action="store_true")
    args = parser.parse_args()

    compounds = ["soft", "medium", "hard"] if args.all_compounds else [args.compound]
    for c in compounds:
        train(compound    = c,
              epochs      = args.epochs,
              batch_size  = args.batch_size,
              lr          = args.lr,
              lambda_fric = args.lambda_fric,
              lambda_sym  = args.lambda_sym,
              seed        = args.seed)
