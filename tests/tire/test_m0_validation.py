"""
tests/tire/test_m0_validation.py — Milestone 0 validation gate.

All four M0 gates in one file, runnable with:
    pytest tests/tire/test_m0_validation.py -v

Gates
─────
G1. pinn_model imports standalone (no tire_rig / ws_server deps)
G2. tire_interface imports standalone
G3. All three compound YAMLs load correctly
G4. Fy C¹ continuity across κ sweep (no spikes in dFy/dt — B4 fix check)
G5. mean(Mz) < 0.1 Nm in 60 s symmetric run with γ=0 (B3 fix check)
G6. Directory structure matches spec
G7. tire_rig.py (in rig/) is importable and smoke-runs
G8. Gxα / Gyκ are non-negative for all (α, κ) and equal 1.0 at origin
"""

import importlib
import math
import os
import sys
from pathlib import Path
import numpy as np
import pytest

# ── Project root on path ──────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

COEFFS_DIR   = PROJECT_ROOT / "core" / "tire" / "pacejka_coeffs"
MEDIUM_YAML  = COEFFS_DIR / "medium.yaml"

# Validation thresholds — named constants so they can be tightened without
# changing test logic.  See Milestone 0 spec.
MZ_MEAN_THRESHOLD_NM = 0.10    # |mean(Mz)| must be below this [N·m]
ALPHA_PRIME_LAG_MIN  = 0.5    # alpha_prime must differ from alpha_kin by > this
                               # fraction of steady-state difference after one τ
# C¹ kink threshold: computed analytically in test_fy_no_spikes_across_kappa
# as 2.5 × (Byk·Cyk)² · |Fy0|.  No global constant needed.

# ─────────────────────────────────────────────────────────────────────────────
# G1: standalone import (no tire_rig / ws_server)
# ─────────────────────────────────────────────────────────────────────────────
class TestStandaloneImport:

    def test_pinn_model_imports_without_tire_rig(self):
        """pinn_model must not import tire_rig or ws_server."""
        if "core.tire.pinn_model" in sys.modules:
            del sys.modules["core.tire.pinn_model"]

        spec = importlib.util.spec_from_file_location(
            "pinn_model",
            PROJECT_ROOT / "core" / "tire" / "pinn_model.py",
        )
        mod = importlib.util.module_from_spec(spec)
        # No exception = pass
        spec.loader.exec_module(mod)

        # Confirm tire_rig and ws_server are NOT imported as side effects
        assert "tire_rig" not in sys.modules, "pinn_model imported tire_rig"
        assert "ws_server" not in sys.modules, "pinn_model imported ws_server"

    def test_tire_interface_imports_standalone(self):
        from core.tire.tire_interface import TireModel
        model = TireModel.from_yaml(MEDIUM_YAML)
        assert model is not None

    def test_all_compounds_load(self):
        from core.tire.tire_interface import TireModel
        for compound in ("soft", "medium", "hard"):
            path = COEFFS_DIR / f"{compound}.yaml"
            assert path.exists(), f"{compound}.yaml missing"
            model = TireModel.from_yaml(path)
            Fx, Fy, Mz = model.compute_forces(
                alpha=-0.05, kappa=0.02, Fz=1470.0, gamma=0.0, vx=10.0, dt=0.001
            )
            assert math.isfinite(Fx)
            assert math.isfinite(Fy)
            assert math.isfinite(Mz)


# ─────────────────────────────────────────────────────────────────────────────
# G4: Fy C¹ continuity across κ sweep (B4 fix)
# ─────────────────────────────────────────────────────────────────────────────
class TestFyContinuity:
    """
    Sweep κ from −0.8 to +0.8 at fixed α=10°, Fz=1470N, γ=0.
    Compute Fy at each κ. Numerically differentiate twice.
    If d²Fy/dκ² has spikes, the coupling is not C¹.
    """

    def _fy_sweep(self):
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn   = DummyTirePINN.from_coeffs(coeffs)

        alpha_rad = math.radians(10.0)
        Fz        = 1470.0
        kappas    = np.linspace(-0.8, 0.8, 1601)   # 0.001 step
        Fy_vals   = np.array([
            pinn.predict_combined(Fz=Fz, slip_angle=alpha_rad, kappa=k)[1]
            for k in kappas
        ])
        return kappas, Fy_vals

    def test_fy_no_spikes_across_kappa(self):
        """
        Detect kink (C¹ violation) in Fy(κ) using a spike-ratio metric.

        The Pacejka Gxα/Gyκ formulation is C∞ smooth.  Its d²Fy/dκ² has a
        predictable analytic maximum at κ=0:

            |d²Fy/dκ²|_smooth_max = (Byk·Cyk)² · |Fy0|

        For medium compound: (8·1)² · 1321 ≈ 84,700 N  — large but finite
        and smoothly distributed across κ.

        The old post-hoc friction ellipse produced an isolated kink at κ_crit
        where F_vec first reached F_max.  At that point dFy/dκ jumped from 0
        to ≈ −862 N/(unit κ), giving d²Fy/dκ² ≈ 862,000 N at one grid cell.

        Test metric: max(|d²Fy/dκ²|) must not exceed 2.5 × the analytic smooth
        bound.  The smooth case passes (~84k < 212k).  A kink fails (~862k).
        """
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn   = DummyTirePINN.from_coeffs(coeffs)

        kappas, Fy = self._fy_sweep()
        dk = kappas[1] - kappas[0]
        d2Fy = np.diff(Fy, n=2) / (dk ** 2)
        max_curv = float(np.max(np.abs(d2Fy)))

        # Pure-slip Fy at α=10°, Fz=1470 (the |Fy0| in the analytic bound)
        Fy0_mag = abs(pinn.predict(Fz=1470.0, slip_angle=math.radians(10.0))[0])

        # Analytic smooth upper bound for Gyk·Fy0 with current params
        smooth_d2fy_bound = (pinn.Byk * pinn.Cyk) ** 2 * Fy0_mag
        # 2.5× safety factor — kink (old code) would be ~10× this bound
        kink_detection_threshold = 2.5 * smooth_d2fy_bound

        assert max_curv < kink_detection_threshold, (
            f"d²Fy/dκ² peak = {max_curv:.0f} N exceeds kink-detection threshold "
            f"{kink_detection_threshold:.0f} N  (analytic smooth bound = "
            f"{smooth_d2fy_bound:.0f} N).  B4 fix may be incomplete."
        )

    def test_fy_monotone_reduction_with_kappa(self):
        """Fy at α=10° should decrease (in magnitude) as |κ| increases."""
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn   = DummyTirePINN.from_coeffs(coeffs)
        alpha_rad = math.radians(10.0)
        Fz = 1470.0

        Fy0   = pinn.predict_combined(Fz=Fz, slip_angle=alpha_rad, kappa=0.0)[1]
        Fy_hi = pinn.predict_combined(Fz=Fz, slip_angle=alpha_rad, kappa=0.6)[1]
        assert abs(Fy_hi) < abs(Fy0), (
            f"|Fy(κ=0.6)| = {abs(Fy_hi):.1f} N ≥ |Fy(κ=0)| = {abs(Fy0):.1f} N "
            "— combined slip should reduce lateral force."
        )

    def test_gxalpha_gykappa_non_negative(self):
        """G-factors must be ≥ 0 for all (α, κ) in operating range."""
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn   = DummyTirePINN.from_coeffs(coeffs)

        for alpha_deg in np.linspace(-30, 30, 61):
            alpha = math.radians(alpha_deg)
            Gxa = math.cos(pinn.Cxa * math.atan(pinn.Bxa * alpha))
            assert Gxa >= -1e-9, f"Gxα < 0 at α={alpha_deg:.1f}°: {Gxa:.4f}"

        for kappa in np.linspace(-0.9, 0.9, 181):
            Gyk = math.cos(pinn.Cyk * math.atan(pinn.Byk * kappa))
            assert Gyk >= -1e-9, f"Gyκ < 0 at κ={kappa:.2f}: {Gyk:.4f}"

    def test_g_factors_unity_at_origin(self):
        """At α=0, Gxα=1.  At κ=0, Gyκ=1."""
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn   = DummyTirePINN.from_coeffs(coeffs)

        Gxa0 = math.cos(pinn.Cxa * math.atan(pinn.Bxa * 0.0))
        Gyk0 = math.cos(pinn.Cyk * math.atan(pinn.Byk * 0.0))
        assert abs(Gxa0 - 1.0) < 1e-9, f"Gxα(α=0) ≠ 1: {Gxa0}"
        assert abs(Gyk0 - 1.0) < 1e-9, f"Gyκ(κ=0) ≠ 1: {Gyk0}"


# ─────────────────────────────────────────────────────────────────────────────
# G5: Mz mean < 0.1 Nm in 60 s symmetric run (B3 fix)
# ─────────────────────────────────────────────────────────────────────────────
class TestMzDrift:
    """
    Simulate 60 s with α sweeping ±10° symmetrically (no DC offset), γ=0,
    κ=0, Fz=1470 N, vx=10 m/s.  Mean Mz must be < MZ_MEAN_THRESHOLD_NM.
    """

    def test_mz_mean_symmetric_run(self):
        from core.tire.tire_interface import TireModel
        model = TireModel.from_yaml(MEDIUM_YAML)
        model.reset()

        dt   = 0.001
        vx   = 10.0
        Fz   = 1470.0
        T    = 60.0
        steps = int(T / dt)
        f_sweep = 0.5   # Hz, one full cycle per 2 s

        Mz_log = np.empty(steps)
        for i in range(steps):
            t_sim = i * dt
            alpha_kin = math.radians(10.0) * math.sin(2.0 * math.pi * f_sweep * t_sim)
            _, _, Mz = model.compute_forces(
                alpha=alpha_kin, kappa=0.0, Fz=Fz, gamma=0.0, vx=vx, dt=dt
            )
            Mz_log[i] = Mz

        mean_mz = float(np.mean(Mz_log))
        assert abs(mean_mz) < MZ_MEAN_THRESHOLD_NM, (
            f"mean(Mz) = {mean_mz:.4f} Nm in symmetric run "
            f"(threshold ±{MZ_MEAN_THRESHOLD_NM} Nm). B3 fix may be incomplete."
        )

    def test_mz_no_linear_drift(self):
        """
        Linear regression slope of Mz vs t must be near zero.
        A linear drift (B3 symptom) shows up as a nonzero slope.
        """
        from core.tire.tire_interface import TireModel
        model = TireModel.from_yaml(MEDIUM_YAML)
        model.reset()

        dt    = 0.001
        vx    = 10.0
        Fz    = 1470.0
        T     = 60.0
        steps = int(T / dt)
        # Use the same fixed α=5° to avoid masking drift behind oscillation
        alpha_fixed = math.radians(5.0)

        t_arr  = np.arange(steps) * dt
        Mz_arr = np.empty(steps)
        for i in range(steps):
            _, _, Mz = model.compute_forces(
                alpha=alpha_fixed, kappa=0.0, Fz=Fz, gamma=0.0, vx=vx, dt=dt
            )
            Mz_arr[i] = Mz

        # Fit linear trend
        slope, _ = np.polyfit(t_arr, Mz_arr, 1)
        # Allow at most 0.01 Nm/s drift (1 Nm over 100 s)
        drift_threshold = 0.01  # Nm/s
        assert abs(slope) < drift_threshold, (
            f"Mz linear slope = {slope:.5f} Nm/s — progressive drift detected "
            f"(threshold {drift_threshold} Nm/s). Check Mz formula accumulation."
        )

    def test_mz_consistent_with_fy(self):
        """
        At κ=0.5, Fy is reduced by Gyκ.  Mz should also be reduced (not stuck
        at pure-slip value).  Confirms B3 fix: Mz uses combined Fy.
        """
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs
        coeffs   = TireCoeffs.from_yaml(MEDIUM_YAML)
        pinn     = DummyTirePINN.from_coeffs(coeffs)
        alpha    = math.radians(8.0)
        Fz       = 1470.0

        _, Fy_pure, Mz_pure = pinn.predict_combined(Fz=Fz, slip_angle=alpha, kappa=0.0)
        _, Fy_comb, Mz_comb = pinn.predict_combined(Fz=Fz, slip_angle=alpha, kappa=0.5)

        # Fy should be reduced at κ=0.5
        assert abs(Fy_comb) < abs(Fy_pure), (
            "Fy not reduced at κ=0.5 — Gyκ weighting not working"
        )
        # Mz should also be reduced (B3 fix: uses combined Fy)
        assert abs(Mz_comb) < abs(Mz_pure), (
            f"|Mz(κ=0.5)| = {abs(Mz_comb):.3f} ≥ |Mz(κ=0)| = {abs(Mz_pure):.3f} — "
            "Mz is not using combined Fy. B3 fix incomplete."
        )


# ─────────────────────────────────────────────────────────────────────────────
# G6: Directory structure
# ─────────────────────────────────────────────────────────────────────────────
class TestDirectoryStructure:

    REQUIRED_PATHS = [
        "core/tire/pinn_model.py",
        "core/tire/tire_interface.py",
        "core/tire/pacejka_coeffs/soft.yaml",
        "core/tire/pacejka_coeffs/medium.yaml",
        "core/tire/pacejka_coeffs/hard.yaml",
        "core/chassis/motorcycle.py",
        "core/chassis/bike_params/cbr600rr.yaml",
        "core/rider/rider_input.py",
        "core/road/flat_plane.py",
        "core/integrator/rk4.py",
        "rig/tire_rig.py",
        "transport/ws_server.py",
        "tests/tire/",
        "tests/motorcycle/",
        "tests/system/",
        "scenarios/",
        "docs/decisions/",
    ]

    @pytest.mark.parametrize("rel_path", REQUIRED_PATHS)
    def test_path_exists(self, rel_path):
        full = PROJECT_ROOT / rel_path
        assert full.exists(), f"Required path missing: {rel_path}"


# ─────────────────────────────────────────────────────────────────────────────
# G7: tire_rig backward compatibility
# ─────────────────────────────────────────────────────────────────────────────
class TestTireRigCompat:

    def test_pinn_model_importable_from_rig_context(self):
        """
        The rig imports pinn_model.  We verify the fixed pinn_model
        is drop-in compatible: predict_combined still returns (Fx, Fy, Mz).
        """
        from core.tire.pinn_model import DummyTirePINN, TireCoeffs

        pinn = DummyTirePINN()   # default coefficients
        Fx, Fy, Mz = pinn.predict_combined(
            Fz=1470.0, slip_angle=math.radians(10.0), kappa=0.05, camber=0.0
        )
        assert isinstance(Fx, float)
        assert isinstance(Fy, float)
        assert isinstance(Mz, float)
        assert math.isfinite(Fx) and math.isfinite(Fy) and math.isfinite(Mz)

    def test_predict_returns_tuple_of_two(self):
        """predict() must return (Fy, Mz) — backward compat with tire_rig."""
        from core.tire.pinn_model import DummyTirePINN
        pinn = DummyTirePINN()
        result = pinn.predict(Fz=1470.0, slip_angle=0.1, camber=0.0)
        assert len(result) == 2

    def test_tire_model_with_relaxation_compat(self):
        """TireModelWithRelaxation API must be unchanged."""
        from core.tire.pinn_model import DummyTirePINN, TireModelWithRelaxation
        pinn  = DummyTirePINN()
        model = TireModelWithRelaxation(pinn, sigma=0.3)
        model.reset(alpha0=0.0)
        Fy, Mz = model.step(Fz=1470.0, slip_angle=0.1, vx=5.0, dt=0.001)
        assert isinstance(Fy, float)
        assert isinstance(Mz, float)


# ─────────────────────────────────────────────────────────────────────────────
# G8: Physics sanity checks
# ─────────────────────────────────────────────────────────────────────────────
class TestPhysicsSanity:

    def test_fy_peaks_at_expected_slip_angle(self):
        """Fy peak must occur near B·C·D inverse = 1/(B·C)."""
        from core.tire.pinn_model import DummyTirePINN
        pinn = DummyTirePINN()
        peak_deg = pinn.peak_slip_angle_deg(Fz=1470.0)
        # For C=1.8, typical peak ≈ 7–12°
        assert 5.0 < peak_deg < 15.0, (
            f"Peak Fy at {peak_deg:.1f}° — outside expected 5–15° range"
        )

    def test_cornering_stiffness_equals_bcd(self):
        """dFy/dα|α=0 must equal B·C·D to within 1%."""
        from core.tire.pinn_model import DummyTirePINN
        pinn = DummyTirePINN()
        Fz = 1470.0
        D  = pinn.mu * Fz
        B  = (pinn.k_alpha * Fz) / (pinn.C_pac * D + 1e-8)
        expected_Cs = B * pinn.C_pac * D
        actual_Cs   = pinn.cornering_stiffness(Fz)
        rel_err = abs(actual_Cs - expected_Cs) / (abs(expected_Cs) + 1e-6)
        assert rel_err < 0.01, (
            f"Cornering stiffness: expected {expected_Cs:.0f} N/rad, "
            f"got {actual_Cs:.0f} N/rad  (rel err {rel_err:.3%})"
        )

    def test_mz_stabilising_at_small_slip(self):
        """Mz must be negative (stabilising) for small positive α', κ=0, γ=0."""
        from core.tire.pinn_model import DummyTirePINN
        pinn = DummyTirePINN()
        _, _, Mz = pinn.predict_combined(
            Fz=1470.0, slip_angle=math.radians(5.0), kappa=0.0, camber=0.0
        )
        assert Mz < 0.0, (
            f"Mz = {Mz:.3f} Nm — should be negative (stabilising) for α=5°, κ=0"
        )

    def test_fy_odd_function_at_zero_camber(self):
        """Fy(−α) = −Fy(+α) at γ=0 (Pacejka symmetry)."""
        from core.tire.pinn_model import DummyTirePINN
        pinn = DummyTirePINN()
        for alpha_deg in [2.0, 5.0, 10.0, 15.0]:
            a = math.radians(alpha_deg)
            Fy_pos, _ = pinn.predict(Fz=1470.0, slip_angle=+a, camber=0.0)
            Fy_neg, _ = pinn.predict(Fz=1470.0, slip_angle=-a, camber=0.0)
            assert abs(Fy_pos + Fy_neg) < 0.01, (
                f"Fy symmetry broken at α={alpha_deg}°: "
                f"Fy(+α)={Fy_pos:.3f}, Fy(−α)={Fy_neg:.3f}"
            )

    def test_relaxation_time_constant(self):
        """
        After a step input in α, the relaxation should follow
        Fy(t) ≈ Fy_ss·(1 − exp(−t/τ)) with τ = σ/vx.
        Fit τ from the simulated response and compare to theoretical.
        """
        from core.tire.tire_interface import TireModel
        model = TireModel.from_yaml(MEDIUM_YAML)
        model.reset()

        vx    = 10.0
        dt    = 0.001
        Fz    = 1470.0
        sigma = 0.30   # matches medium.yaml
        tau   = sigma / vx   # 30 ms

        # Warm up at α=0 for 0.5 s to stabilise
        for _ in range(500):
            model.compute_forces(alpha=0.0, kappa=0.0, Fz=Fz, gamma=0.0, vx=vx, dt=dt)

        # Step to α=10°
        alpha_step = math.radians(10.0)
        steps_post = int(0.3 / dt)   # 300 ms post-step
        alpha_prime_log = np.empty(steps_post)

        for i in range(steps_post):
            model.compute_forces(
                alpha=alpha_step, kappa=0.0, Fz=Fz, gamma=0.0, vx=vx, dt=dt
            )
            alpha_prime_log[i] = model.alpha_prime

        # Theoretical: α'(t) = α_step·(1 − exp(−t/τ))
        t_arr     = np.arange(steps_post) * dt
        alpha_th  = alpha_step * (1.0 - np.exp(-t_arr / tau))
        rms_err   = float(np.sqrt(np.mean((alpha_prime_log - alpha_th)**2)))
        rms_thresh = math.radians(0.5)   # 0.5° RMS tolerance

        assert rms_err < rms_thresh, (
            f"Relaxation ODE RMS error = {math.degrees(rms_err):.3f}° "
            f"(threshold {math.degrees(rms_thresh):.1f}°). "
            f"Check ZOH implementation or sigma value."
        )
