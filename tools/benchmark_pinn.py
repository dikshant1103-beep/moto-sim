#!/usr/bin/env python3
"""
tools/benchmark_pinn.py  —  PINN-3: Inference latency benchmark
──────────────────────────────────────────────────────────────────
Compares per-call latency of:
  1. Analytical Pacejka   (DummyTirePINN.predict_combined)
  2. ONNX Runtime         (OnnxTireModel via onnxruntime CPUExecutionProvider)
  3. PyTorch CPU          (TirePINN.forward, no-grad)

Tests both single-call and batch scenarios at typical simulation
operating points (v=10 m/s, α=5°, κ=0, γ=0, Fz=1470 N).

Run from moto_sim/ root:
    python tools/benchmark_pinn.py
    python tools/benchmark_pinn.py --compound soft --n 5000
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

DATASET_DIR = ROOT / "dataset"


def _bench(label: str, fn, n: int, warmup: int = 200) -> float:
    """Run fn() n+warmup times; return mean latency [µs] over n calls."""
    for _ in range(warmup):
        fn()
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    elapsed = time.perf_counter() - t0
    mean_us = elapsed / n * 1e6
    print(f"  {label:<38} {mean_us:>8.2f} µs / call   ({n} calls)")
    return mean_us


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--compound", default="medium",
                        choices=["soft", "medium", "hard"])
    parser.add_argument("--n", type=int, default=3000,
                        help="Benchmark iterations (default 3000)")
    args = parser.parse_args()

    compound = args.compound
    n        = args.n

    print(f"\n=== PINN-3: Inference Benchmark  [{compound} compound, n={n}] ===\n")

    # ── Test point ─────────────────────────────────────────────────────────────
    Fz    = 1470.0
    alpha = math.radians(5.0)
    kappa = 0.0
    gamma = 0.0
    vx    = 10.0
    dt    = 0.001

    yaml_path = ROOT / "core/tire/pacejka_coeffs" / f"{compound}.yaml"
    onnx_path = DATASET_DIR / f"tire_pinn_{compound}.onnx"
    pt_path   = DATASET_DIR / f"tire_pinn_{compound}.pt"

    # ── 1. Analytical Pacejka ─────────────────────────────────────────────────
    from core.tire.pinn_model  import DummyTirePINN, TireCoeffs
    from core.tire.tire_interface import TireModel

    coeffs      = TireCoeffs.from_yaml(yaml_path)
    pinn_ref    = DummyTirePINN.from_coeffs(coeffs)
    pacejka_model = TireModel.from_yaml(yaml_path)
    pacejka_model.reset()

    print("Single-call latency (simulating 1 kHz physics loop):")

    lat_pac_raw = _bench(
        "Pacejka predict_combined() [raw]",
        lambda: pinn_ref.predict_combined(Fz=Fz, slip_angle=alpha, kappa=kappa, camber=gamma),
        n,
    )

    lat_pac_full = _bench(
        "TireModel.compute_forces() [full w/ ZOH+M5]",
        lambda: pacejka_model.compute_forces(alpha, kappa, Fz, gamma, vx, dt),
        n,
    )

    # ── 2. ONNX Runtime ───────────────────────────────────────────────────────
    from core.tire.onnx_tire import OnnxTireModel

    onnx_model = OnnxTireModel.from_compound(compound, DATASET_DIR)
    onnx_model.reset()

    lat_ort_full = _bench(
        "OnnxTireModel.compute_forces() [ONNX+ZOH+M5]",
        lambda: onnx_model.compute_forces(alpha, kappa, Fz, gamma, vx, dt),
        n,
    )

    # ONNX inference only (no ZOH, no thermal) — closest to bare C++ target
    # Physical units: model.forward() handles normalisation internally
    x_phys = np.array([[Fz, alpha, kappa, gamma]], dtype=np.float32)
    _sess  = onnx_model._session
    _iname = onnx_model._input_name
    lat_ort_bare = _bench(
        "ONNX session.run() bare [no ZOH/M5]",
        lambda: _sess.run(None, {_iname: x_phys}),
        n,
    )

    # ── 3. PyTorch CPU ────────────────────────────────────────────────────────
    import torch
    saved = torch.load(pt_path, map_location="cpu", weights_only=False)

    sys.path.insert(0, str(ROOT / "rig"))
    from train_pinn import TirePINN
    pt_model = TirePINN(mu=float(coeffs.mu))
    pt_model.load_state_dict(saved["model_state"])
    pt_model.eval()

    x_pt = torch.tensor([[Fz, alpha, kappa, gamma]], dtype=torch.float32)
    with torch.no_grad():
        lat_pt = _bench(
            "PyTorch CPU TirePINN.forward() [no-grad]",
            lambda: pt_model(x_pt),
            n,
        )

    # ── Correctness cross-check ───────────────────────────────────────────────
    print("\nCorrectness check at [Fz=1470, α=5°, κ=0, γ=0]:")
    Fx_pac, Fy_pac, Mz_pac = pinn_ref.predict_combined(
        Fz=Fz, slip_angle=alpha, kappa=kappa, camber=gamma)
    Fx_ort, Fy_ort, Mz_ort = onnx_model._infer(Fz, alpha, kappa, gamma)
    with torch.no_grad():
        y_pt = pt_model(x_pt)[0].numpy()
    Fx_pt, Fy_pt, Mz_pt = float(y_pt[0]), float(y_pt[1]), float(y_pt[2])

    print(f"  {'Backend':<36} {'Fx [N]':>8} {'Fy [N]':>8} {'Mz [Nm]':>9}")
    print(f"  {'-'*63}")
    print(f"  {'Pacejka (analytical reference)':<36} {Fx_pac:>8.2f} {Fy_pac:>8.2f} {Mz_pac:>9.3f}")
    print(f"  {'ONNX Runtime (PINN)':<36} {Fx_ort:>8.2f} {Fy_ort:>8.2f} {Mz_ort:>9.3f}")
    print(f"  {'PyTorch CPU (PINN)':<36} {Fx_pt:>8.2f} {Fy_pt:>8.2f} {Mz_pt:>9.3f}")

    dFy_ort = abs(Fy_ort - Fy_pac)
    dFy_pt  = abs(Fy_pt  - Fy_pac)
    print(f"\n  ONNX vs Pacejka  |ΔFy| = {dFy_ort:.2f} N   (test MAE was ~20 N)")
    print(f"  PT   vs Pacejka  |ΔFy| = {dFy_pt:.2f} N")

    # ── Summary table ─────────────────────────────────────────────────────────
    print(f"\n{'─'*60}")
    print(f"  Summary  (target = <15 µs bare ONNX in C++)")
    print(f"{'─'*60}")
    print(f"  Pacejka raw          : {lat_pac_raw:>7.2f} µs   (baseline)")
    print(f"  Pacejka + ZOH + M5  : {lat_pac_full:>7.2f} µs")
    print(f"  ONNX bare inference  : {lat_ort_bare:>7.2f} µs   ({lat_ort_bare/lat_pac_raw:.1f}× Pacejka)")
    print(f"  ONNX + ZOH + M5     : {lat_ort_full:>7.2f} µs")
    print(f"  PyTorch CPU          : {lat_pt:>7.2f} µs   ({lat_pt/lat_pac_raw:.1f}× Pacejka)")
    print()

    ort_stats = onnx_model.stats
    print(f"  ORT guard fallbacks : {ort_stats.n_fallbacks}/{ort_stats.n_calls}"
          f"  ({ort_stats.fallback_pct:.2f}%)")
    print()

    # Budget check: at 1 kHz with 2 tires, budget = 500 µs / tire
    budget_us = 500.0
    ok = "✓ WITHIN BUDGET" if lat_ort_full < budget_us else "⚠ OVER BUDGET"
    print(f"  1 kHz loop budget   : {budget_us:.0f} µs / call  →  {ok}")
    print()


if __name__ == "__main__":
    main()
