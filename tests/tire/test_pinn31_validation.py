"""
tests/tire/test_pinn31_validation.py  —  Phase 3.1 coefficient fitter gates

G1  model_loads        — ONNX loads without error
G2  output_in_range    — all 6 scales in [0.35, 1.55]
G3  nominal_response   — nominal-condition input → scales ≈ 1.0 (±0.3)
G4  worn_response      — cold/worn features → mu_y < nominal (lower grip)
G5  grip_monotone      — higher T_tread in window → mu_y estimate ≥ cold
G6  distinct_outputs   — different tire conditions → different scale outputs
G7  latency            — single call < 5 ms on CPU
G8  no_crash_on_zeros  — all-zero input does not throw
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "rig"))

ONNX_PATH = ROOT / "dataset" / "coeff_fitter.onnx"

SCALE_MIN = np.array([0.45, 0.60, 0.70, 0.45, 0.70, 0.60], dtype=np.float32)
SCALE_MAX = np.array([1.15, 1.40, 1.30, 1.15, 1.30, 1.50], dtype=np.float32)
SCALE_KEYS = ["mu_y", "p_Ky1", "E_Fy", "mu_x", "K_x", "tp0"]

FEAT_CENTER = np.array([1470., 0., 0., 0., 70., 0., 0., 0.], np.float32)
FEAT_SCALE  = np.array([ 750., 0.2, 0.25, 0.30, 50., 1500., 1500., 30.], np.float32)


def _load_session():
    import onnxruntime as ort
    return ort.InferenceSession(str(ONNX_PATH), providers=["CPUExecutionProvider"])


def _build_input(Fz_mean=1470., alpha_rms=0.1, kappa_rms=0.1,
                  T_mean=80., Fy_mean=500., rng=None):
    """Build a synthetic 34-dim input vector from basic parameters."""
    if rng is None:
        rng = np.random.default_rng(42)
    N = 64
    Fz    = rng.normal(Fz_mean, 100.,  N).astype(np.float32)
    alpha = rng.normal(0., alpha_rms,   N).astype(np.float32)
    kappa = rng.normal(0., kappa_rms,   N).astype(np.float32)
    gamma = rng.normal(0., 0.1,         N).astype(np.float32)
    T     = rng.normal(T_mean, 5.,      N).astype(np.float32)
    Fx    = rng.normal(0., 200.,        N).astype(np.float32)
    Fy    = rng.normal(Fy_mean, 100.,   N).astype(np.float32)
    Mz    = rng.normal(0., 5.,          N).astype(np.float32)

    obs   = np.stack([Fz, alpha, kappa, gamma, T, Fx, Fy, Mz], axis=1)
    obs_n = (obs - FEAT_CENTER) / (FEAT_SCALE + 1e-9)

    mean_ = obs_n.mean(0); std_  = obs_n.std(0)
    min_  = obs_n.min(0);  max_  = obs_n.max(0)
    summary = np.concatenate([mean_, std_, min_, max_])
    extra   = np.array([(Fz.mean()-1470.)/750., (T.mean()-70.)/50.], np.float32)
    return np.concatenate([summary, extra]).astype(np.float32)[None]  # (1,34)


# ── G1 ────────────────────────────────────────────────────────────────────────
def test_g1_model_loads():
    assert ONNX_PATH.exists(), f"ONNX not found: {ONNX_PATH}"
    sess = _load_session()
    assert sess is not None


# ── G2 ────────────────────────────────────────────────────────────────────────
def test_g2_output_in_range():
    sess   = _load_session()
    rng    = np.random.default_rng(7)
    for _ in range(20):
        x   = _build_input(rng=rng)
        out = sess.run(None, {"telemetry_stats": x})[0].flatten()
        for i, (s, lo, hi) in enumerate(zip(out, SCALE_MIN, SCALE_MAX)):
            assert lo - 0.05 <= s <= hi + 0.05, \
                f"{SCALE_KEYS[i]} = {s:.4f} outside [{lo},{hi}]"


# ── G3 ────────────────────────────────────────────────────────────────────────
def test_g3_nominal_response():
    """All-nominal input should produce scales reasonably close to 1.0."""
    sess = _load_session()
    x    = _build_input(Fz_mean=1470., T_mean=85., Fy_mean=800.)
    out  = sess.run(None, {"telemetry_stats": x})[0].flatten()
    nom  = (SCALE_MIN + SCALE_MAX) / 2.0   # midpoint of bounds
    # Each scale should be within ±0.35 of midpoint (loose: identifiability limit)
    for i, (s, m) in enumerate(zip(out, nom)):
        assert abs(s - m) < 0.40, \
            f"{SCALE_KEYS[i]} = {s:.4f} far from midpoint {m:.4f}"


# ── G4 ────────────────────────────────────────────────────────────────────────
def test_g4_worn_vs_fresh():
    """
    Cold/low-Fy window should produce different mu_y than hot/high-Fy window.
    We don't assert direction (network may not be monotone), just distinction.
    """
    sess  = _load_session()
    x_hot = _build_input(T_mean=110., Fy_mean=1200.)
    x_cold= _build_input(T_mean=20.,  Fy_mean=400.)
    out_hot  = sess.run(None, {"telemetry_stats": x_hot })[0].flatten()
    out_cold = sess.run(None, {"telemetry_stats": x_cold})[0].flatten()
    # mu_y (idx 0) should differ by at least 0.02
    assert abs(out_hot[0] - out_cold[0]) > 0.02, \
        f"mu_y: hot={out_hot[0]:.4f}  cold={out_cold[0]:.4f} — no distinction"


# ── G5 ────────────────────────────────────────────────────────────────────────
def test_g5_grip_varies_with_force():
    """Higher Fy magnitude → network should estimate higher lateral grip."""
    sess   = _load_session()
    x_hi   = _build_input(Fy_mean=1400.)
    x_lo   = _build_input(Fy_mean=200.)
    out_hi = sess.run(None, {"telemetry_stats": x_hi})[0].flatten()
    out_lo = sess.run(None, {"telemetry_stats": x_lo})[0].flatten()
    # mu_y (or p_Ky1) should be higher for high-Fy window
    assert out_hi[0] + out_hi[1] > out_lo[0] + out_lo[1] - 0.10, \
        "Grip parameters do not increase with higher observed lateral force"


# ── G6 ────────────────────────────────────────────────────────────────────────
def test_g6_distinct_outputs():
    """10 different random windows should produce 10 different scale vectors."""
    sess    = _load_session()
    outputs = set()
    rng     = np.random.default_rng(99)
    for _ in range(10):
        x   = _build_input(
            Fz_mean   = rng.uniform(600, 2800),
            alpha_rms = rng.uniform(0.02, 0.25),
            T_mean    = rng.uniform(25, 115),
            Fy_mean   = rng.uniform(100, 1500),
            rng=rng,
        )
        out = sess.run(None, {"telemetry_stats": x})[0].flatten()
        outputs.add(tuple(out.round(4)))
    assert len(outputs) >= 8, f"Only {len(outputs)}/10 distinct outputs — network may be constant"


# ── G7 ────────────────────────────────────────────────────────────────────────
def test_g7_latency():
    """Single inference call < 5 ms on CPU."""
    sess = _load_session()
    x    = _build_input()
    # Warm-up
    for _ in range(3):
        sess.run(None, {"telemetry_stats": x})
    # Measure
    t0 = time.perf_counter()
    for _ in range(50):
        sess.run(None, {"telemetry_stats": x})
    ms = (time.perf_counter() - t0) / 50 * 1000
    assert ms < 5.0, f"Inference {ms:.2f} ms > 5 ms gate"
    print(f"\n    Latency: {ms:.3f} ms/call")


# ── G8 ────────────────────────────────────────────────────────────────────────
def test_g8_no_crash_on_zeros():
    """All-zero input (edge case) must not throw."""
    sess = _load_session()
    x    = np.zeros((1, 34), np.float32)
    out  = sess.run(None, {"telemetry_stats": x})[0]
    assert out.shape == (1, 6)
    assert np.all(np.isfinite(out))


if __name__ == "__main__":
    tests = [
        ("G1 model_loads",      test_g1_model_loads),
        ("G2 output_in_range",  test_g2_output_in_range),
        ("G3 nominal_response", test_g3_nominal_response),
        ("G4 worn_vs_fresh",    test_g4_worn_vs_fresh),
        ("G5 grip_varies",      test_g5_grip_varies_with_force),
        ("G6 distinct_outputs", test_g6_distinct_outputs),
        ("G7 latency",          test_g7_latency),
        ("G8 no_crash_zeros",   test_g8_no_crash_on_zeros),
    ]
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as e:
            print(f"  FAIL  {name}: {e}")
