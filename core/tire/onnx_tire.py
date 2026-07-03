"""
core/tire/onnx_tire.py  —  PINN-3: ONNX Runtime tire model
─────────────────────────────────────────────────────────────
Drop-in replacement for TireModel that uses the trained PINN
(ONNX format) instead of the analytical Pacejka formula.

API is identical to TireModel.compute_forces() so the motorcycle
simulation loop needs zero changes — just swap the instance.

Physics guard
─────────────
If the PINN output violates the friction ellipse by more than
GUARD_SLACK (10%), the call falls back to analytical Pacejka
and increments a fallback counter.  This ensures safety at the
cost of one extra Pacejka evaluation in rare cases.

Normalization (must match train_pinn.py constants)
───────────────────────────────────────────────────
IN_CENTER  = [1750, 0, 0, 0]
IN_SCALE   = [750, 0.175, 0.25, 0.20]
OUT_SCALE  = [3000, 3000, 65]

Performance (GTX 1650 Ti host, CPUExecutionProvider)
──────────────────────────────────────────────────────
Single call:  ~60–120 µs  (vs 2 µs Pacejka, 400 µs PyTorch)
The ONNX Runtime C++ API target is <15 µs — see Part B roadmap.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

_ORT_OK = False
try:
    import onnxruntime as ort
    _ORT_OK = True
except ImportError:
    pass

from .pinn_model import DummyTirePINN, TireCoeffs
from .tire_interface import TireState

# Physics guard: allow up to 10% over the friction ellipse before fallback
GUARD_SLACK = 1.10


@dataclass
class OnnxInferenceStats:
    """Cumulative inference statistics — reset with .reset()."""
    n_calls:        int   = 0
    n_fallbacks:    int   = 0
    total_us:       float = 0.0      # cumulative ONNX inference time [µs]

    def reset(self) -> None:
        self.n_calls = self.n_fallbacks = 0
        self.total_us = 0.0

    @property
    def mean_us(self) -> float:
        return self.total_us / max(self.n_calls, 1)

    @property
    def fallback_pct(self) -> float:
        return self.n_fallbacks / max(self.n_calls, 1) * 100.0

    def __str__(self) -> str:
        return (f"calls={self.n_calls}  mean={self.mean_us:.1f} µs  "
                f"fallbacks={self.n_fallbacks} ({self.fallback_pct:.2f}%)")


class OnnxTireModel:
    """
    ONNX Runtime tire model with ZOH relaxation, M5 thermal/wear,
    and a physics guard that falls back to analytical Pacejka.

    Parameters
    ----------
    onnx_path : path to .onnx file from PINN-2
    coeffs    : TireCoeffs for the same compound (used by fallback + thermal)
    """

    def __init__(self, onnx_path: str | Path, coeffs: TireCoeffs) -> None:
        assert _ORT_OK, "onnxruntime not installed — pip install onnxruntime"
        onnx_path = Path(onnx_path)
        assert onnx_path.exists(), f"ONNX model not found: {onnx_path}"

        opts = ort.SessionOptions()
        opts.inter_op_num_threads = 1
        opts.intra_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._session = ort.InferenceSession(str(onnx_path), sess_options=opts)
        self._input_name = self._session.get_inputs()[0].name

        self._coeffs = coeffs
        self._mu     = coeffs.mu
        self._sigma  = coeffs.sigma

        # Analytical fallback (Pacejka) — used by physics guard
        self._fallback = DummyTirePINN.from_coeffs(coeffs)

        self._state = TireState()
        self.stats  = OnnxInferenceStats()

    # ── Factories ─────────────────────────────────────────────────────────────

    @classmethod
    def from_compound(cls, compound: str = "medium",
                      dataset_dir: Optional[Path] = None) -> "OnnxTireModel":
        """Load by compound name — resolves paths automatically."""
        root = Path(__file__).resolve().parent.parent.parent
        if dataset_dir is None:
            dataset_dir = root / "dataset"
        onnx_path = dataset_dir / f"tire_pinn_{compound}.onnx"
        yaml_path = root / "core/tire/pacejka_coeffs" / f"{compound}.yaml"
        coeffs    = TireCoeffs.from_yaml(yaml_path)
        return cls(onnx_path, coeffs)

    # ── State management ──────────────────────────────────────────────────────

    def reset(self, state: Optional[TireState] = None) -> None:
        self._state = state if state is not None else TireState()

    @property
    def alpha_prime(self) -> float:
        return self._state.alpha_prime

    @property
    def state(self) -> TireState:
        import copy
        return copy.copy(self._state)

    # ── Core inference ────────────────────────────────────────────────────────

    def _infer(self, Fz: float, alpha_prime: float,
               kappa: float, gamma: float) -> tuple[float, float, float]:
        """
        Run ONNX inference and return (Fx, Fy, Mz) in physical units [N, N, Nm].

        The exported ONNX model includes normalise_in and denormalise_out
        operations baked in (they are part of TirePINN.forward()), so this
        method passes PHYSICAL units directly — no manual scaling needed.
        """
        x = np.array([[Fz, alpha_prime, kappa, gamma]], dtype=np.float32)

        t0 = time.perf_counter()
        y  = self._session.run(None, {self._input_name: x})[0]
        elapsed_us = (time.perf_counter() - t0) * 1e6

        self.stats.total_us += elapsed_us
        self.stats.n_calls  += 1

        return float(y[0, 0]), float(y[0, 1]), float(y[0, 2])

    # ── Physics guard ─────────────────────────────────────────────────────────

    def _guard(self, Fx: float, Fy: float, Fz: float,
               alpha_prime: float, kappa: float,
               gamma: float) -> tuple[float, float, float]:
        """
        Check friction ellipse.  If violated by > GUARD_SLACK, fall back
        to analytical Pacejka and increment fallback counter.
        """
        mu_fz = self._mu * Fz + 1e-6
        ellipse = (Fx ** 2 + Fy ** 2) / (mu_fz ** 2)

        if ellipse <= GUARD_SLACK:
            return Fx, Fy, None   # None signals Mz still needed

        # Fallback
        self.stats.n_fallbacks += 1
        Fx_fb, Fy_fb, Mz_fb = self._fallback.predict_combined(
            Fz=Fz, slip_angle=alpha_prime, kappa=kappa, camber=gamma,
        )
        return Fx_fb, Fy_fb, Mz_fb

    # ── Public API — identical to TireModel.compute_forces() ─────────────────

    def compute_forces(
        self,
        alpha: float,
        kappa: float,
        Fz:    float,
        gamma: float,
        vx:    float,
        dt:    float,
    ) -> tuple[float, float, float]:
        """
        Advance ZOH relaxation, run ONNX inference, apply physics guard
        and M5 thermal/wear scaling.

        Returns (Fx, Fy, Mz) in physical units [N, N, Nm].
        """
        # ── ZOH relaxation (same as TireModel) ───────────────────────────────
        vx_safe = max(abs(vx), 1e-3)
        decay   = math.exp(-vx_safe * dt / self._sigma)
        self._state.alpha_prime = alpha + (self._state.alpha_prime - alpha) * decay

        # ── ONNX inference ────────────────────────────────────────────────────
        Fx, Fy, Mz = self._infer(Fz, self._state.alpha_prime, kappa, gamma)

        # ── Physics guard ─────────────────────────────────────────────────────
        Fx, Fy, Mz_guard = self._guard(Fx, Fy, Fz, self._state.alpha_prime,
                                        kappa, gamma)
        if Mz_guard is not None:
            Mz = Mz_guard   # fallback already computed Mz

        # ── M5: Thermal / wear state update (same as TireModel) ───────────────
        c = self._coeffs
        P_slip = (abs(Fx * vx_safe * abs(kappa))
                  + abs(Fy * vx_safe * abs(self._state.alpha_prime)))

        self._state.T_tread += (
            c.heat_scale * P_slip
            - (self._state.T_tread - c.T_ambient) / c.tau_tread
        ) * dt
        self._state.T_carcass += (
            (self._state.T_tread - self._state.T_carcass) / c.tau_carcass
        ) * dt
        self._state.pressure = (
            c.p_cold * (273.15 + self._state.T_carcass) / (273.15 + c.T_ambient)
        )
        self._state.wear = min(
            1.0,
            self._state.wear + c.k_wear * (P_slip ** c.wear_exp) * dt,
        )

        # ── M5: Grip scaling ──────────────────────────────────────────────────
        g_T = (c.T_cold_grip
               + (1.0 - c.T_cold_grip)
               * math.exp(-((self._state.T_tread - c.T_optimal) / c.T_width) ** 2))
        g_W = 1.0 - c.grip_wear_max * self._state.wear
        mu_scale = g_T * g_W

        return Fx * mu_scale, Fy * mu_scale, Mz * mu_scale

    # ── Convenience: pure-slip query (no state advance) ───────────────────────
    def query_pure(self, Fz: float, alpha: float,
                   camber: float = 0.0) -> tuple[float, float]:
        Fx, Fy, Mz = self._infer(Fz, alpha, 0.0, camber)
        return Fy, Mz

    # ── Analytical properties (delegated to fallback) ─────────────────────────
    def peak_slip_angle_deg(self, Fz: float = 1470.0) -> float:
        return self._fallback.peak_slip_angle_deg(Fz)

    def cornering_stiffness(self, Fz: float = 1470.0) -> float:
        return self._fallback.cornering_stiffness(Fz)
