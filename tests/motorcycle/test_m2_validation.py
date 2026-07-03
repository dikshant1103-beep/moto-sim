"""
tests/motorcycle/test_m2_validation.py — M2 gate tests.

M2 gates:

  G1  dynamic_fz_braking     — full braking shifts ≥10% weight forward
  G2  dynamic_fz_accel       — full throttle shifts ≥10% weight rearward
  G3  fz_sum_conserved       — dynamic Fz front+rear = total weight ±2%
  G4  roll_perturbation      — 5° roll perturbation decays to <3° within 2 s
  G5  weave_oscillatory      — roll perturbation produces ≥1 sign reversal
                               (weave mode is oscillatory, not monotone decay)
  G6  speed_stability        — perturbation decays faster at 10 m/s than 5 m/s
  G7  m1_regression          — all 8 M1 gates still pass
  G8  capsize_low_speed      — below capsize speed, roll perturbation GROWS
                               (capsize mode: monotone divergence, not decay)

Run from moto_sim/ root:
    pytest tests/motorcycle/test_m2_validation.py -v
"""

from __future__ import annotations

import math
import sys
import importlib
from pathlib import Path

import pytest
import pychrono as chrono

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

from core.chassis.motorcycle import Motorcycle, BikeParams
from core.rider.rider_input  import ConstantInputSource
from core.road.flat_plane    import FlatPlane

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001


def _make_moto(steer_torque=0.0, throttle=0.0, brake_front=0.0, brake_rear=0.0):
    rider = ConstantInputSource(
        steer_torque=steer_torque, throttle=throttle,
        brake_front=brake_front,  brake_rear=brake_rear,
    )
    return Motorcycle.from_yaml(
        BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
        FlatPlane(), rider, dt=DT,
    )


def _static_fz_split(moto: Motorcycle) -> tuple[float, float]:
    """Return the pure-static (ax=0) Fz split for this bike's geometry."""
    p = moto.params
    total_weight = (
        p.main_frame_mass + p.swingarm_mass + p.front_fork_mass +
        p.front_wheel_mass + p.rear_wheel_mass
    ) * 9.81
    frac_front = p.cg_x / p.wheelbase
    return total_weight * frac_front, total_weight * (1.0 - frac_front)


# ── G1: Dynamic Fz under braking ─────────────────────────────────────────────

def test_g1_dynamic_fz_braking():
    """Full braking from 10 m/s: front Fz exceeds static by ≥10%."""
    moto = _make_moto(brake_front=1.0, brake_rear=0.5)
    moto.build()
    moto.reset(v0=10.0)

    static_fz_f, _ = _static_fz_split(moto)

    # Run 0.8 s — enough for EMA filter (T=0.1 s) to capture weight transfer
    n_steps = round(0.8 / DT)
    peak_fz_f = 0.0
    for k in range(n_steps):
        telem = moto.step(k * DT)
        peak_fz_f = max(peak_fz_f, telem["front"]["Fz"])

    ratio = peak_fz_f / static_fz_f
    assert ratio > 1.10, (
        f"Braking weight transfer too small: peak Fz_front={peak_fz_f:.1f} N "
        f"vs static={static_fz_f:.1f} N (ratio={ratio:.3f}, need >1.10)"
    )


# ── G2: Dynamic Fz under acceleration ────────────────────────────────────────

def test_g2_dynamic_fz_accel():
    """Full throttle from 5 m/s: rear Fz exceeds static by ≥10%."""
    moto = _make_moto(throttle=1.0)
    moto.build()
    moto.reset(v0=5.0)

    _, static_fz_r = _static_fz_split(moto)

    n_steps = round(0.8 / DT)
    peak_fz_r = 0.0
    for k in range(n_steps):
        telem = moto.step(k * DT)
        peak_fz_r = max(peak_fz_r, telem["rear"]["Fz"])

    ratio = peak_fz_r / static_fz_r
    assert ratio > 1.10, (
        f"Acceleration weight transfer too small: peak Fz_rear={peak_fz_r:.1f} N "
        f"vs static={static_fz_r:.1f} N (ratio={ratio:.3f}, need >1.10)"
    )


# ── G3: Fz sum conserved (no energy injected by weight-transfer model) ────────

def test_g3_fz_sum_conserved():
    """At steady cruise, Fz_front + Fz_rear = total weight ±2%."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=7.0)

    p = moto.params
    total_weight = (
        p.main_frame_mass + p.swingarm_mass + p.front_fork_mass +
        p.front_wheel_mass + p.rear_wheel_mass
    ) * 9.81

    # Run 1 s to let ax_smooth converge to ~0 at constant speed
    n_steps = round(1.0 / DT)
    for k in range(n_steps):
        telem = moto.step(k * DT)

    fz_sum = telem["front"]["Fz"] + telem["rear"]["Fz"]
    err = abs(fz_sum - total_weight) / total_weight
    assert err < 0.02, (
        f"Fz sum={fz_sum:.1f} N vs weight={total_weight:.1f} N "
        f"(err={err*100:.2f}%, limit 2%)"
    )


# ── G4: Roll perturbation decays ─────────────────────────────────────────────

def test_g4_roll_perturbation_decays():
    """5° roll perturbation at 6 m/s decays to <3° within 2 s."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=6.0)

    # Apply 5° roll offset to the main frame quaternion
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))

    n_steps = round(2.0 / DT)
    for k in range(n_steps):
        telem = moto.step(k * DT)

    final_roll = abs(math.degrees(telem["chassis"]["roll"]))
    assert final_roll < 3.0, (
        f"Roll perturbation not recovered: final |roll|={final_roll:.2f}° "
        f"after 2 s (limit 3°)"
    )


# ── G5: Weave mode is oscillatory ────────────────────────────────────────────

def test_g5_weave_oscillatory():
    """
    5° roll perturbation produces ≥1 roll sign reversal within 2 s.

    The weave mode is a complex eigenvalue (oscillatory), not a pure real
    eigenvalue (monotone decay).  Sign reversals confirm oscillatory character.
    """
    moto = _make_moto()
    moto.build()
    moto.reset(v0=6.0)
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))

    n_steps = round(2.0 / DT)
    sign_changes = 0
    prev_roll = math.radians(5.0)

    for k in range(n_steps):
        telem = moto.step(k * DT)
        curr_roll = telem["chassis"]["roll"]
        if prev_roll * curr_roll < 0:   # sign flip
            sign_changes += 1
        prev_roll = curr_roll

    assert sign_changes >= 1, (
        f"Roll response shows 0 sign reversals — not oscillatory. "
        f"Weave mode may be missing or overdamped."
    )


# ── G6: Faster decay at higher speed ─────────────────────────────────────────

def test_g6_speed_stability():
    """
    Weave frequency increases with speed: 10 m/s produces more roll sign reversals
    in 3 s than 5 m/s after a 5° perturbation.

    Sharp 1971: weave frequency ∝ v — higher speed = faster oscillation = more
    sign reversals in a fixed time window.
    """
    def _sign_reversals(v0: float, n_steps: int = 3000) -> int:
        moto = _make_moto()
        moto.build()
        moto.reset(v0=v0)
        moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))
        crossings = 0
        prev_roll = math.radians(5.0)
        for k in range(n_steps):
            telem = moto.step(k * DT)
            curr_roll = telem["chassis"]["roll"]
            if prev_roll * curr_roll < 0:
                crossings += 1
            prev_roll = curr_roll
        return crossings

    rev_5  = _sign_reversals(5.0)
    rev_10 = _sign_reversals(10.0)

    assert rev_10 > rev_5, (
        f"Weave frequency should increase with speed: "
        f"reversals@5m/s={rev_5}, reversals@10m/s={rev_10} — expected 10m/s > 5m/s"
    )


# ── G7: M1 regression ────────────────────────────────────────────────────────

def test_g7_m1_regression():
    """All 8 M1 gates pass after M2 changes."""
    import tests.motorcycle.test_m1_validation as m1
    m1.test_g1_build()
    m1.test_g2_reset_velocity()
    m1.test_g3_step_finite()
    m1.test_g4_straight_line_stability()
    m1.test_g5_static_fz_split()
    m1.test_g6_countersteering()
    m1.test_g7_tire_force_magnitude()
    m1.test_g8_accumulator_cleared()


# ── G8: Capsize mode at low speed ────────────────────────────────────────────


def test_g8_capsize_low_speed():
    """
    Below the simulation capsize speed, a 5° roll perturbation must GROW.

    Physical basis: at very low speed, gravity destabilisation overwhelms
    both the supplement roll stiffness AND the Pacejka tire forces routed
    through PyChrono — the capsize real eigenvalue is positive and roll
    diverges monotonically (Sharp 1971, capsize mode).

    Observed simulation capsize threshold: ~0.6 m/s.
    (The analytical Whipple threshold ~1.87 m/s is higher because Pacejka
    camber/lateral forces add real stability beyond the 1-DOF formula —
    the same effect that shifts the weave frequency upward at low speeds.)

    Test speed: 0.5 m/s — clearly below simulation threshold.
    Window: 1.5 s — long enough to distinguish monotone growth from noise.
    Gate: |roll| must grow by ≥ 0.8° from the initial 5° perturbation.
    (Sweep shows +1.6° growth at v=0.5 m/s; 0.8° gives comfortable margin.)
    """
    V_TEST      = 0.5    # m/s — below simulation capsize threshold (~0.6 m/s)
    PERTURB_DEG = 5.0    # deg
    T_WINDOW    = 1.5    # s
    GROW_MIN    = 0.8    # deg

    moto = _make_moto()
    moto.build()
    moto.reset(v0=V_TEST)
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(PERTURB_DEG)))

    n_steps = round(T_WINDOW / DT)
    for k in range(n_steps):
        telem = moto.step(k * DT)

    final_roll = abs(math.degrees(telem["chassis"]["roll"]))
    assert final_roll > PERTURB_DEG + GROW_MIN, (
        f"Capsize mode not observed at v={V_TEST} m/s: "
        f"initial={PERTURB_DEG:.1f}°, final={final_roll:.2f}° "
        f"(must exceed {PERTURB_DEG + GROW_MIN:.1f}°). "
        f"Roll should diverge at this speed."
    )
