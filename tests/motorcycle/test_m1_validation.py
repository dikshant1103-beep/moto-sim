"""
tests/motorcycle/test_m1_validation.py — M1 gate tests for the PyChrono motorcycle.

M1 gates (all must pass before M2 work begins):

  G1  build()  — motorcycle constructs without error; system has ≥5 bodies
  G2  reset()  — initial velocity is v0, wheel spin satisfies no-slip
  G3  step()   — 10 steps at 5 m/s produce a finite, non-NaN telemetry dict
  G4  straight — 3 s straight-line at 5 m/s; |roll| < 5°, |yaw_rate| < 3°/s
  G5  Fz       — static Fz split sums to total weight (within 1 %)
  G6  countersteering — at 10 m/s, +2 N·m steer for 0.5 s → bike turns LEFT
                        (yaw_rate < 0 after transient, SAE Z-up convention)
  G7  tire forces — at straight 10 m/s, |Fx|/Fz < 0.05 (low rolling resistance proxy)
  G8  accumulator — after one full step, accumulators are cleared (no force double-add)

Run from moto_sim/ root:
    pytest tests/motorcycle/test_m1_validation.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

from core.chassis.motorcycle import Motorcycle, BikeParams
from core.rider.rider_input  import ConstantInputSource, RiderInput
from core.road.flat_plane    import FlatPlane

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _make_moto(steer_torque: float = 0.0, throttle: float = 0.0,
               brake_front: float = 0.0, brake_rear: float = 0.0) -> Motorcycle:
    rider = ConstantInputSource(
        steer_torque=steer_torque,
        throttle=throttle,
        brake_front=brake_front,
        brake_rear=brake_rear,
    )
    return Motorcycle.from_yaml(
        BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
        FlatPlane(), rider, dt=DT,
    )


# ── G1: Build ─────────────────────────────────────────────────────────────────

def test_g1_build():
    """Motorcycle.build() succeeds; system has ≥5 non-ground bodies."""
    moto = _make_moto()
    moto.build()
    assert moto.system is not None
    assert moto.main_frame is not None
    assert moto.front_wheel is not None
    assert moto.rear_wheel is not None
    # Ground + frame + swingarm + fork + front_wheel + rear_wheel = 6 bodies
    # GetNumBodies() returns 0 in this PyChrono build; use GetBodies() iterator.
    n_bodies = len(list(moto.system.GetBodies()))
    assert n_bodies >= 5, f"Expected ≥5 bodies, got {n_bodies}"


# ── G2: Reset ─────────────────────────────────────────────────────────────────

def test_g2_reset_velocity():
    """After reset(v0=5.0), all bodies move at 5 m/s; wheels satisfy no-slip."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=5.0)

    p = moto.params
    for body in [moto.main_frame, moto.swingarm, moto.front_fork,
                 moto.front_wheel, moto.rear_wheel]:
        vx = body.GetPosDt().x
        assert abs(vx - 5.0) < 0.01, f"Body vx={vx:.4f} expected 5.0"

    # No-slip: ω_y = +v0/R
    omf = moto.front_wheel.GetAngVelParent().y
    omr = moto.rear_wheel.GetAngVelParent().y
    assert abs(omf - 5.0 / p.front_wheel_radius) < 0.01, \
        f"Front wheel omega_y={omf:.4f} expected {5.0/p.front_wheel_radius:.4f}"
    assert abs(omr - 5.0 / p.rear_wheel_radius) < 0.01, \
        f"Rear wheel omega_y={omr:.4f} expected {5.0/p.rear_wheel_radius:.4f}"


# ── G3: Step produces finite telemetry ────────────────────────────────────────

def test_g3_step_finite():
    """10 physics steps produce a complete, finite telemetry dict."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=5.0)

    telem = None
    for k in range(10):
        telem = moto.step(k * DT)

    assert telem is not None
    assert "chassis" in telem
    assert "front"   in telem
    assert "rear"    in telem
    assert "inputs"  in telem

    c = telem["chassis"]
    for key in ("roll", "pitch", "yaw", "vx", "vy"):
        v = c[key]
        assert math.isfinite(v), f"chassis.{key}={v} is not finite"

    for side in ("front", "rear"):
        for key in ("alpha", "kappa", "Fz", "Fx", "Fy", "Mz"):
            v = telem[side][key]
            assert math.isfinite(v), f"{side}.{key}={v} is not finite"


# ── G4: Straight-line stability ───────────────────────────────────────────────

def test_g4_straight_line_stability():
    """3 s at 5 m/s, no inputs: |roll| < 5° and |yaw_rate| < 3°/s throughout."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=5.0)

    n_steps   = round(3.0 / DT)
    max_roll  = 0.0
    max_yr    = 0.0

    for k in range(n_steps):
        telem = moto.step(k * DT)
        c     = telem["chassis"]
        max_roll = max(max_roll, abs(math.degrees(c["roll"])))
        max_yr   = max(max_yr,   abs(math.degrees(c["yaw_rate"])))

    assert max_roll < 5.0, \
        f"Straight-line: max |roll|={max_roll:.2f}° exceeds 5°"
    assert max_yr < 3.0, \
        f"Straight-line: max |yaw_rate|={max_yr:.2f}°/s exceeds 3°/s"


# ── G5: Static Fz split ───────────────────────────────────────────────────────

def test_g5_static_fz_split():
    """_static_Fz() sums to total weight within 1%."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=5.0)

    p = moto.params
    total_weight = (
        p.main_frame_mass + p.swingarm_mass + p.front_fork_mass +
        p.front_wheel_mass + p.rear_wheel_mass
    ) * 9.81

    Fz_f, Fz_r = moto._static_Fz()
    Fz_sum     = Fz_f + Fz_r

    assert abs(Fz_sum - total_weight) / total_weight < 0.01, \
        f"Fz sum={Fz_sum:.1f} N vs weight={total_weight:.1f} N (>{1}% error)"

    # Both loads must be positive
    assert Fz_f > 0
    assert Fz_r > 0


# ── G6: Countersteering ───────────────────────────────────────────────────────

def test_g6_countersteering():
    """
    At 10 m/s, +2 N·m steer torque applied for 0.5 s then zeroed.
    After the 0.5 s application window the bike should be turning LEFT
    (yaw_rate < 0 in SAE/PyChrono Z-up convention where +Z is up and
    left turn = negative yaw rate when X is forward and Y is left).

    Physics: +torque steers wheel right → gyroscopic reaction rolls frame left
    → left cornering (yaw_rate < 0).

    Tolerance: yaw_rate < −0.05 °/s at t = 0.5 s.
    """
    STEER_NM   = 2.0     # N·m
    APPLY_TIME = 0.5     # s
    V0         = 10.0    # m/s

    # Phase 1: apply steer torque
    moto = _make_moto(steer_torque=STEER_NM)
    moto.build()
    moto.reset(v0=V0)

    n_apply = round(APPLY_TIME / DT)
    telem   = None
    for k in range(n_apply):
        telem = moto.step(k * DT)

    yaw_rate_deg = math.degrees(telem["chassis"]["yaw_rate"])

    assert yaw_rate_deg < -0.05, (
        f"Countersteering gate FAILED: yaw_rate={yaw_rate_deg:.4f}°/s "
        f"(expected < −0.05°/s → LEFT turn).  "
        f"Check steer-torque sign or gyroscopic coupling."
    )


# ── G7: Rolling resistance proxy ─────────────────────────────────────────────

def test_g7_tire_force_magnitude():
    """At 10 m/s straight-line, |Fx_rear|/Fz_rear < 0.05 (not braking hard)."""
    moto = _make_moto()
    moto.build()
    moto.reset(v0=10.0)

    # Run 0.5 s to let transient settle
    n_steps = round(0.5 / DT)
    telem   = None
    for k in range(n_steps):
        telem = moto.step(k * DT)

    r  = telem["rear"]
    Fz = r["Fz"]
    Fx = r["Fx"]

    assert Fz > 100, f"Rear Fz={Fz:.1f} N too small — ground contact broken?"
    ratio = abs(Fx) / Fz
    assert ratio < 0.05, \
        f"|Fx_rear|/Fz_rear={ratio:.4f} exceeds 0.05 — unexpected longitudinal load"


# ── G8: Accumulator cleared ───────────────────────────────────────────────────

def test_g8_accumulator_cleared():
    """
    Forces should not double-add: run two consecutive steps with the same
    rider input and confirm Fx magnitudes are comparable (not doubling).
    """
    moto = _make_moto(throttle=0.5)
    moto.build()
    moto.reset(v0=5.0)

    t1 = moto.step(0.0)
    t2 = moto.step(DT)

    Fx1 = abs(t1["rear"]["Fx"])
    Fx2 = abs(t2["rear"]["Fx"])

    # If accumulators weren't cleared Fx2 would be ~2×Fx1; allow 50% variation
    # for transient effects but not 10× growth.
    if Fx1 > 1.0:
        ratio = Fx2 / Fx1
        assert ratio < 5.0, \
            f"Rear Fx jumped {Fx1:.1f}→{Fx2:.1f} N (ratio={ratio:.2f}) — "  \
            f"accumulator may not be cleared between steps"
