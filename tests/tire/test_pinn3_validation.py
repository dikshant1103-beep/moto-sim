"""
tests/tire/test_pinn3_validation.py  —  PINN-3 integration gates

G1  onnx_loads            — OnnxTireModel loads without error
G2  api_identical         — compute_forces() signature matches TireModel
G3  fy_close_to_pacejka   — |ΔFy| < 50 N at nominal operating point
G4  mz_sign_correct       — Mz < 0 at α=5°, κ=0, γ=0 (stabilising)
G5  physics_guard_fires   — guard triggers on synthetic extreme input
G6  fallback_recovers     — after guard, forces satisfy friction ellipse
G7  thermal_state_updates — wear > 0 and T_tread changes after 10 s slip
G8  ort_faster_than_torch — ONNX Runtime bare inference faster than PyTorch
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

from core.tire.onnx_tire    import OnnxTireModel, GUARD_SLACK
from core.tire.tire_interface import TireModel, TireState
from core.tire.pinn_model   import DummyTirePINN, TireCoeffs

MEDIUM_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
MEDIUM_ONNX = ROOT / "dataset/tire_pinn_medium.onnx"

DT = 0.001
FZ = 1470.0
VX = 10.0
ALPHA = math.radians(5.0)


def _onnx():
    m = OnnxTireModel.from_compound("medium")
    m.reset()
    return m


def _pacejka():
    m = TireModel.from_yaml(MEDIUM_YAML)
    m.reset()
    return m


# ── G1 ────────────────────────────────────────────────────────────────────────

def test_g1_onnx_loads():
    """OnnxTireModel.from_compound() loads without error."""
    m = _onnx()
    assert m is not None
    assert m._session is not None


# ── G2 ────────────────────────────────────────────────────────────────────────

def test_g2_api_identical():
    """compute_forces() returns (Fx, Fy, Mz) and exposes alpha_prime and state."""
    m = _onnx()
    result = m.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)
    assert len(result) == 3
    assert all(math.isfinite(v) for v in result)
    assert math.isfinite(m.alpha_prime)
    st = m.state
    assert hasattr(st, "T_tread") and hasattr(st, "wear") and hasattr(st, "pressure")


# ── G3 ────────────────────────────────────────────────────────────────────────

def test_g3_fy_close_to_pacejka():
    """
    |ΔFy| between ONNX and Pacejka < 50 N at nominal point.
    Test MAE was ~20 N; 50 N gives comfortable margin for a single call.
    """
    m_ort = _onnx()
    m_pac = _pacejka()

    _, Fy_ort, _ = m_ort.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)
    _, Fy_pac, _ = m_pac.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)

    delta = abs(Fy_ort - Fy_pac)
    assert delta < 50.0, (
        f"|ΔFy| = {delta:.2f} N > 50 N limit  "
        f"(ONNX={Fy_ort:.1f} N, Pacejka={Fy_pac:.1f} N)"
    )


# ── G4 ────────────────────────────────────────────────────────────────────────

def test_g4_mz_sign_correct():
    """Mz must be negative (stabilising) at α=5°, κ=0, γ=0."""
    m = _onnx()
    _, _, Mz = m.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)
    assert Mz < 0.0, f"Mz={Mz:.4f} Nm — should be negative (stabilising) at α=5°"


# ── G5 ────────────────────────────────────────────────────────────────────────

def test_g5_physics_guard_fires():
    """
    Physics guard must trigger when ONNX output exceeds µFz × GUARD_SLACK.
    We test this by monkeypatching _infer to return an extreme value.
    """
    m = _onnx()
    mu = m._mu

    # Inject absurdly large Fx and Fy (well beyond friction ellipse)
    _orig = m._infer
    def _extreme(*args, **kwargs):
        return FZ * mu * 5.0, FZ * mu * 5.0, 0.0   # ellipse ratio = 50 >> GUARD_SLACK
    m._infer = _extreme

    m.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)

    m._infer = _orig  # restore
    assert m.stats.n_fallbacks >= 1, (
        "Physics guard did not fire despite extreme ONNX output"
    )


# ── G6 ────────────────────────────────────────────────────────────────────────

def test_g6_fallback_recovers():
    """
    After physics guard fires, returned forces must satisfy friction ellipse.
    """
    m = _onnx()
    mu = m._mu

    _orig = m._infer
    def _extreme(*args, **kwargs):
        return FZ * mu * 5.0, FZ * mu * 5.0, 0.0
    m._infer = _extreme

    Fx, Fy, _ = m.compute_forces(ALPHA, 0.0, FZ, 0.0, VX, DT)
    m._infer = _orig

    ellipse = (Fx ** 2 + Fy ** 2) / (mu * FZ) ** 2
    assert ellipse <= GUARD_SLACK + 0.01, (
        f"After fallback, ellipse ratio = {ellipse:.3f} > {GUARD_SLACK}"
    )


# ── G7 ────────────────────────────────────────────────────────────────────────

def test_g7_thermal_state_updates():
    """After 10 s of sustained slip, wear > 0 and T_tread has changed from 85°C."""
    m = _onnx()
    m.reset(TireState(T_tread=85.0, T_carcass=80.0, wear=0.0))

    T_init = m.state.T_tread
    for k in range(10_000):   # 10 s at dt=1ms
        m.compute_forces(ALPHA, 0.05, FZ, 0.0, VX, DT)

    assert m.state.wear > 0.0, "Wear not accumulating in OnnxTireModel"
    assert m.state.T_tread != T_init, "T_tread unchanged after 10 s slip"
    assert math.isfinite(m.state.pressure), "Pressure not finite"


# ── G8 ────────────────────────────────────────────────────────────────────────

def test_g8_ort_faster_than_torch():
    """
    ONNX Runtime bare inference must be faster than PyTorch CPU forward().
    Both measured over 1000 calls (no ZOH, no thermal overhead).
    """
    import numpy as np, torch
    from train_pinn import TirePINN

    coeffs   = TireCoeffs.from_yaml(MEDIUM_YAML)
    m_ort    = OnnxTireModel.from_compound("medium")
    pt_path  = ROOT / "dataset/tire_pinn_medium.pt"
    saved    = torch.load(pt_path, map_location="cpu", weights_only=False)
    pt_model = TirePINN(mu=float(coeffs.mu))
    pt_model.load_state_dict(saved["model_state"])
    pt_model.eval()

    x_np = np.array([[(FZ-1750)/750, ALPHA/0.175, 0.0/0.25, 0.0/0.20]],
                     dtype=np.float32)
    x_pt = torch.tensor([[FZ, ALPHA, 0.0, 0.0]], dtype=torch.float32)
    sess  = m_ort._session
    iname = m_ort._input_name

    N = 1000
    # Warmup
    for _ in range(100):
        sess.run(None, {iname: x_np})
        with torch.no_grad(): pt_model(x_pt)

    t0 = time.perf_counter()
    for _ in range(N): sess.run(None, {iname: x_np})
    lat_ort = (time.perf_counter() - t0) / N * 1e6

    t0 = time.perf_counter()
    for _ in range(N):
        with torch.no_grad(): pt_model(x_pt)
    lat_pt = (time.perf_counter() - t0) / N * 1e6

    assert lat_ort < lat_pt, (
        f"ONNX ({lat_ort:.1f} µs) not faster than PyTorch ({lat_pt:.1f} µs)"
    )


# ── runner ────────────────────────────────────────────────────────────────────

TESTS = [
    ("G1 OnnxTireModel loads",             test_g1_onnx_loads),
    ("G2 API identical to TireModel",       test_g2_api_identical),
    ("G3 Fy within 50 N of Pacejka",        test_g3_fy_close_to_pacejka),
    ("G4 Mz sign correct at α=5°",          test_g4_mz_sign_correct),
    ("G5 Physics guard fires on extreme",   test_g5_physics_guard_fires),
    ("G6 Fallback satisfies ellipse",        test_g6_fallback_recovers),
    ("G7 Thermal state updates after slip", test_g7_thermal_state_updates),
    ("G8 ORT faster than PyTorch CPU",      test_g8_ort_faster_than_torch),
]

if __name__ == "__main__":
    sys.path.insert(0, str(ROOT / "rig"))
    passed = failed = 0
    print("\n=== PINN-3 Validation Gates ===\n")
    for name, fn in TESTS:
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {name}\n        {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
            failed += 1
    print(f"\n  {passed}/{passed+failed} passed"
          + ("  ✓ ALL PASS" if failed == 0 else f"  ✗ {failed} FAILED"))
    print()
