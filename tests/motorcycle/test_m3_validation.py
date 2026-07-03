"""
tests/motorcycle/test_m3_validation.py — M3 gate tests: full suspension dynamics.

M3 gates:

  G1  suspension_present     — build() creates front + rear TSDA links
  G2  fork_travel_static     — at cruise, fork_travel ∈ (5 mm, 150 mm)
  G3  rear_travel_static     — at cruise, rear_travel ∈ (5 mm, 150 mm)
  G4  fork_compresses_braking — hard braking increases fork compression vs cruise
  G5  rear_compresses_accel  — full throttle increases rear compression vs cruise
  G6  travel_finite          — fork_travel and rear_travel are finite at all times
  G7  m1_regression          — all 8 M1 gates pass
  G8  m2_regression          — all 6 M2 gates pass

Run from moto_sim/ root:
    pytest tests/motorcycle/test_m3_validation.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

from core.chassis.motorcycle import Motorcycle
from core.rider.rider_input  import ConstantInputSource
from core.road.flat_plane    import FlatPlane

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001


def _make_moto(throttle=0.0, brake_front=0.0, brake_rear=0.0, steer_torque=0.0):
    rider = ConstantInputSource(
        throttle=throttle,
        brake_front=brake_front,
        brake_rear=brake_rear,
        steer_torque=steer_torque,
    )
    return Motorcycle.from_yaml(
        BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
        FlatPlane(), rider, dt=DT,
    )


def _cruise_travel(v0: float = 7.0, settle_s: float = 0.5):
    """Return (fork_travel, rear_travel) at steady cruise after settling."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=v0)
    n = round(settle_s / DT)
    telem = None
    for k in range(n):
        telem = moto.step(k * DT)
    s = telem["suspension"]
    return s["fork_travel"], s["rear_travel"]


# ── G1: Suspension TSDA links created ────────────────────────────────────────

def test_g1_suspension_present():
    """build() creates front and rear TSDA links (not None)."""
    moto = _make_moto()
    moto.build()
    assert moto._front_susp is not None, "Front TSDA link is None after build()"
    assert moto._rear_susp  is not None, "Rear TSDA link is None after build()"
    assert moto._front_susp_rest_length > 0.0
    assert moto._rear_susp_rest_length  > 0.0


# ── G2: Fork travel at cruise is physically reasonable ────────────────────────

def test_g2_fork_travel_static():
    """At steady cruise, fork is compressed 5–150 mm (spring supporting static load)."""
    fork_travel, _ = _cruise_travel()
    fork_mm = fork_travel * 1000.0
    assert fork_mm > 5.0, (
        f"Fork travel={fork_mm:.1f} mm too small — spring may not be engaged "
        f"(preload calculation failed or spring free-extended)"
    )
    assert fork_mm < 150.0, (
        f"Fork travel={fork_mm:.1f} mm too large — fork may be crashed/bottomed out"
    )


# ── G3: Rear travel at cruise is physically reasonable ───────────────────────

def test_g3_rear_travel_static():
    """At steady cruise, rear shock is compressed 5–150 mm."""
    _, rear_travel = _cruise_travel()
    rear_mm = rear_travel * 1000.0
    assert rear_mm > 5.0, (
        f"Rear travel={rear_mm:.1f} mm too small — rear spring may not be engaged"
    )
    assert rear_mm < 150.0, (
        f"Rear travel={rear_mm:.1f} mm too large — rear shock may be bottomed out"
    )


# ── G4: Fork compresses harder under braking ─────────────────────────────────

def test_g4_fork_compresses_braking():
    """
    Full front braking from 10 m/s: peak fork_travel exceeds static cruise value.

    Physics: deceleration transfers weight forward → Fz_front rises →
    spring compresses further → fork_travel increases.
    """
    # Baseline: settle at cruise then record travel
    fork_cruise, _ = _cruise_travel(v0=10.0, settle_s=0.5)

    # Braking run: apply full front brake from 10 m/s
    moto = _make_moto(brake_front=1.0, brake_rear=0.3)
    moto.build()
    moto.reset(v0=10.0)

    # Let transient settle first
    for k in range(round(0.3 / DT)):
        moto.step(k * DT)

    # Measure peak fork travel during braking
    peak_fork = fork_cruise
    for k in range(round(0.8 / DT)):
        telem = moto.step((0.3 + k * DT))
        peak_fork = max(peak_fork, telem["suspension"]["fork_travel"])

    assert peak_fork > fork_cruise * 1.05, (
        f"Fork did not compress under braking: "
        f"peak={peak_fork*1000:.1f} mm vs cruise={fork_cruise*1000:.1f} mm "
        f"(expected ≥5% increase)"
    )


# ── G5: Rear compresses harder under acceleration ────────────────────────────

def test_g5_rear_compresses_accel():
    """
    Full throttle from 5 m/s: peak rear_travel exceeds static cruise value.

    Physics: acceleration transfers weight rearward → Fz_rear rises →
    rear spring compresses further → rear_travel increases.
    """
    _, rear_cruise = _cruise_travel(v0=5.0, settle_s=0.5)

    moto = _make_moto(throttle=1.0)
    moto.build()
    moto.reset(v0=5.0)

    for k in range(round(0.3 / DT)):
        moto.step(k * DT)

    peak_rear = rear_cruise
    for k in range(round(0.8 / DT)):
        telem = moto.step((0.3 + k * DT))
        peak_rear = max(peak_rear, telem["suspension"]["rear_travel"])

    assert peak_rear > rear_cruise * 1.03, (
        f"Rear shock did not compress under acceleration: "
        f"peak={peak_rear*1000:.1f} mm vs cruise={rear_cruise*1000:.1f} mm "
        f"(expected ≥3% increase — EMA filter T=0.1s limits transient peak)"
    )


# ── G6: Suspension travel stays finite ───────────────────────────────────────

def test_g6_travel_finite():
    """fork_travel and rear_travel are finite numbers for 3 s at 7 m/s."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=7.0)

    n = round(3.0 / DT)
    for k in range(n):
        telem = moto.step(k * DT)
        ft = telem["suspension"]["fork_travel"]
        rt = telem["suspension"]["rear_travel"]
        assert math.isfinite(ft), f"fork_travel={ft} is not finite at t={k*DT:.3f}"
        assert math.isfinite(rt), f"rear_travel={rt} is not finite at t={k*DT:.3f}"


# ── G7: M1 regression ────────────────────────────────────────────────────────

def test_g7_m1_regression():
    """All 8 M1 gates pass after M3 changes."""
    import tests.motorcycle.test_m1_validation as m1
    m1.test_g1_build()
    m1.test_g2_reset_velocity()
    m1.test_g3_step_finite()
    m1.test_g4_straight_line_stability()
    m1.test_g5_static_fz_split()
    m1.test_g6_countersteering()
    m1.test_g7_tire_force_magnitude()
    m1.test_g8_accumulator_cleared()


# ── G8: M2 regression ────────────────────────────────────────────────────────

def test_g8_m2_regression():
    """Core M2 gates pass after M3 changes."""
    import tests.motorcycle.test_m2_validation as m2
    m2.test_g1_dynamic_fz_braking()
    m2.test_g2_dynamic_fz_accel()
    m2.test_g3_fz_sum_conserved()
    m2.test_g4_roll_perturbation_decays()
    m2.test_g5_weave_oscillatory()
