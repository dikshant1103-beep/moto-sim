"""
tests/tire/test_m5_validation.py — M5 thermal/wear tire model gates.

M5 gates:

  G1  cold_tyre_less_grip     — cold tyre (20°C) produces less Fy than warm (85°C)
  G2  warm_up_occurs          — tread heats above 40°C after sustained slip
  G3  cool_down_occurs        — tread cools toward ambient after slip ceases
  G4  wear_accumulates        — wear W > 0 after sustained sliding
  G5  worn_tyre_less_grip     — W=0.5 tyre produces less Fy than W=0 tyre
  G6  pressure_rises_with_temp — tyre pressure increases with carcass temperature
  G7  thermal_in_telemetry    — motorcycle step() dict contains T_tread, wear, pressure
  G8  m4_regression           — all M4 gates pass with warm default tyre state

Run from moto_sim/ root:
    python -c "
    import sys; sys.path.insert(0, '.')
    import tests.tire.test_m5_validation as m5
    m5.test_g1_cold_tyre_less_grip()
    ...
    "
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

from core.tire.tire_interface import TireModel, TireState

MEDIUM_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"

DT  = 0.001
FZ  = 1470.0   # [N]  nominal front load
VX  = 10.0     # [m/s]
ALPHA_SLIP = math.radians(8.0)   # moderate slip — enough to generate P_slip


# ── helpers ───────────────────────────────────────────────────────────────────

def _model_at_temp(T_tread: float, T_carcass: float | None = None) -> TireModel:
    m = TireModel.from_yaml(MEDIUM_YAML)
    tc = T_tread - 5.0 if T_carcass is None else T_carcass
    m.reset(TireState(T_tread=T_tread, T_carcass=tc, wear=0.0))
    return m


def _model_at_wear(wear: float) -> TireModel:
    m = TireModel.from_yaml(MEDIUM_YAML)
    m.reset(TireState(T_tread=85.0, T_carcass=80.0, wear=wear))
    return m


def _run_slip(model: TireModel, n_steps: int, alpha=ALPHA_SLIP, kappa=0.0) -> None:
    for k in range(n_steps):
        model.compute_forces(alpha=alpha, kappa=kappa,
                             Fz=FZ, gamma=0.0, vx=VX, dt=DT)


# ── G1: Cold tyre less grip ───────────────────────────────────────────────────

def test_g1_cold_tyre_less_grip():
    """
    At 20°C (cold start) Fy is significantly less than at 85°C (optimal).
    Grip ratio at cold ≈ T_cold_grip = 0.65.
    """
    model_cold = _model_at_temp(20.0)
    model_warm = _model_at_temp(85.0)

    _, Fy_cold, _ = model_cold.compute_forces(ALPHA_SLIP, 0.0, FZ, 0.0, VX, DT)
    _, Fy_warm, _ = model_warm.compute_forces(ALPHA_SLIP, 0.0, FZ, 0.0, VX, DT)

    assert abs(Fy_cold) < abs(Fy_warm), (
        f"Cold tyre Fy={abs(Fy_cold):.1f} N is NOT less than warm Fy={abs(Fy_warm):.1f} N"
    )
    ratio = abs(Fy_cold) / abs(Fy_warm)
    assert ratio < 0.80, (
        f"Grip ratio cold/warm = {ratio:.3f}; expected < 0.80 "
        f"(T_cold_grip=0.65 means ~35% grip loss)"
    )


# ── G2: Warm-up occurs ────────────────────────────────────────────────────────

def test_g2_warm_up_occurs():
    """
    After 30 s of sustained slip at v=10 m/s, tread temperature rises above 40°C.
    Starting from cold (20°C), the equilibrium temperature should be ~73°C.
    """
    model = _model_at_temp(20.0)
    _run_slip(model, n_steps=30_000)   # 30 s at dt=1ms

    T_final = model.state.T_tread
    assert T_final > 40.0, (
        f"Tread temperature after 30 s slip = {T_final:.1f}°C; expected > 40°C. "
        f"Thermal warm-up not working."
    )


# ── G3: Cool-down after slip ceases ──────────────────────────────────────────

def test_g3_cool_down_occurs():
    """
    After reaching 75°C, idling (P_slip=0) for 30 s must cool the tread.
    """
    # First warm up
    model = _model_at_temp(20.0)
    _run_slip(model, n_steps=20_000)   # 20 s slip → heats to ~60°C
    T_hot = model.state.T_tread
    assert T_hot > 40.0, "Warm-up prerequisite failed"

    # Now idle (zero slip angle, zero kappa → P_slip ≈ 0)
    _run_slip(model, n_steps=30_000, alpha=0.0, kappa=0.0)
    T_cooled = model.state.T_tread

    assert T_cooled < T_hot - 5.0, (
        f"Tread did not cool: T_before={T_hot:.1f}°C, T_after={T_cooled:.1f}°C. "
        f"Expected at least 5°C drop after 30 s idle."
    )


# ── G4: Wear accumulates ──────────────────────────────────────────────────────

def test_g4_wear_accumulates():
    """
    After 30 s of sustained slip, wear W must be > 0.
    """
    model = _model_at_temp(85.0)   # start warm so grip is full (pure wear test)
    _run_slip(model, n_steps=30_000)

    W = model.state.wear
    assert W > 0.0, (
        f"Wear = {W:.2e} after 30 s slip; expected W > 0. "
        f"Wear accumulation not working."
    )


# ── G5: Worn tyre less grip ───────────────────────────────────────────────────

def test_g5_worn_tyre_less_grip():
    """
    W=0.5 tyre has noticeably less lateral grip than W=0 tyre (both warm).
    Expected: |Fy(W=0.5)| / |Fy(W=0)| ≈ 1 − 0.5·grip_wear_max = 1 − 0.15 = 0.85.
    """
    model_new  = _model_at_wear(0.0)
    model_worn = _model_at_wear(0.5)

    _, Fy_new,  _ = model_new.compute_forces(ALPHA_SLIP, 0.0, FZ, 0.0, VX, DT)
    _, Fy_worn, _ = model_worn.compute_forces(ALPHA_SLIP, 0.0, FZ, 0.0, VX, DT)

    assert abs(Fy_worn) < abs(Fy_new), (
        f"Worn tyre Fy={abs(Fy_worn):.1f} N is NOT less than new Fy={abs(Fy_new):.1f} N"
    )
    ratio = abs(Fy_worn) / abs(Fy_new)
    assert ratio < 0.95, (
        f"Wear grip ratio = {ratio:.3f} at W=0.5; expected < 0.95 "
        f"(grip_wear_max=0.30 → ratio should be ~0.85)"
    )


# ── G6: Pressure rises with carcass temperature ───────────────────────────────

def test_g6_pressure_rises_with_temp():
    """
    Tyre pressure increases with carcass temperature (ideal gas law).
    Warm carcass (80°C) must have higher pressure than cold carcass (20°C).
    """
    model_cold = _model_at_temp(20.0, T_carcass=20.0)
    model_warm = _model_at_temp(85.0, T_carcass=80.0)

    # Single step to trigger pressure update
    model_cold.compute_forces(0.0, 0.0, FZ, 0.0, VX, DT)
    model_warm.compute_forces(0.0, 0.0, FZ, 0.0, VX, DT)

    p_cold = model_cold.state.pressure
    p_warm = model_warm.state.pressure

    assert p_warm > p_cold, (
        f"Pressure at 80°C carcass ({p_warm:.1f} kPa) should be > "
        f"pressure at 20°C ({p_cold:.1f} kPa)"
    )
    # Ideal gas: Δp ≈ p_cold × ΔT/T_cold_K = 230 × 60/293 ≈ 47 kPa
    dp = p_warm - p_cold
    assert dp > 20.0, (
        f"Pressure rise Δp = {dp:.1f} kPa; expected > 20 kPa for 60°C rise"
    )


# ── G7: Thermal state appears in motorcycle telemetry ────────────────────────

def test_g7_thermal_in_telemetry():
    """
    motorcycle.step() dict must contain T_tread, T_carcass, wear, pressure
    in both 'front' and 'rear' sub-dicts.
    """
    import pychrono as chrono
    from core.chassis.motorcycle import Motorcycle
    from core.rider.rider_input  import ConstantInputSource
    from core.road.flat_plane    import FlatPlane

    BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
    FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
    REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"

    rider = ConstantInputSource()
    moto  = Motorcycle.from_yaml(BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
                                 FlatPlane(), rider, dt=DT)
    moto.build()
    moto.reset(v0=6.0)
    telem = moto.step(0.0)

    for wheel in ("front", "rear"):
        for key in ("T_tread", "T_carcass", "wear", "pressure"):
            assert key in telem[wheel], (
                f"Telemetry missing '{wheel}/{key}'"
            )
        assert math.isfinite(telem[wheel]["T_tread"]),   f"{wheel}/T_tread not finite"
        assert math.isfinite(telem[wheel]["pressure"]),  f"{wheel}/pressure not finite"
        assert 0.0 <= telem[wheel]["wear"] <= 1.0,       f"{wheel}/wear out of [0,1]"


# ── G8: M4 regression ────────────────────────────────────────────────────────

def test_g8_m4_regression():
    """All M4 gates pass with warm-default tyre state."""
    import tests.motorcycle.test_m4_validation as m4
    m4.test_g1_road_queried()
    m4.test_g2_flat_mu1_no_change()
    m4.test_g3_low_mu_reduces_fy()
    m4.test_g4_curved_flat_normal()
    m4.test_g5_curved_banked_normal()
    m4.test_g6_road_in_telemetry()
    m4.test_g7_m3_regression()
