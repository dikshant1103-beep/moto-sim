"""
tests/motorcycle/test_m4_validation.py — M4-road gate tests: curved banked road.

M4-road gates:

  G1  road_queried         — road.query() called for both wheels each step
  G2  flat_mu1_no_change   — FlatPlane(mu=1.0) gives same roll behaviour as M3
  G3  low_mu_reduces_fy    — FlatPlane(mu=0.5) peak |Fy| significantly below mu=1.0
  G4  curved_flat_normal   — CurvedRoad straight section returns (0, 0, 1) normal
  G5  curved_banked_normal — CurvedRoad corner returns banked normal (nz < 1.0)
  G6  road_in_telemetry    — step() dict contains 'road' key with mu + normal fields
  G7  m3_regression        — all 8 M3 gates pass

Run from moto_sim/ root:
    pytest tests/motorcycle/test_m4_validation.py -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

import pychrono as chrono

from core.chassis.motorcycle import Motorcycle
from core.rider.rider_input  import ConstantInputSource
from core.road.flat_plane    import FlatPlane, RoadSample
from core.road.curved_road   import CurvedRoad

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

class RecordingRoad:
    """Thin wrapper that counts road.query() calls (for G1)."""

    def __init__(self, inner=None):
        self._inner   = inner or FlatPlane()
        self.call_count = 0
        self.last_queries: list[tuple[float, float]] = []

    def query(self, x: float, y: float) -> RoadSample:
        self.call_count += 1
        self.last_queries.append((x, y))
        return self._inner.query(x, y)

    @property
    def mu_dry(self) -> float:
        return self._inner.mu_dry


def _make_moto(road=None):
    """Create motorcycle with default straight-ride inputs."""
    rider = ConstantInputSource()
    return Motorcycle.from_yaml(
        BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
        road or FlatPlane(), rider, dt=DT,
    )


def _peak_fy(mu: float, n_steps: int = 300) -> float:
    """
    Run n_steps starting from a 5° roll perturbation at v=6 m/s.
    Returns the maximum |Fy_front| observed.
    """
    road  = FlatPlane(mu_dry=mu)
    moto  = _make_moto(road)
    moto.build()
    moto.reset(v0=6.0)
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))
    peak = 0.0
    for k in range(n_steps):
        telem = moto.step(k * DT)
        peak  = max(peak, abs(telem["front"]["Fy"]))
    return peak


# ─────────────────────────────────────────────────────────────────────────────
# G1: road.query() called for both wheels each step
# ─────────────────────────────────────────────────────────────────────────────

def test_g1_road_queried():
    """road.query(x, y) is called at least twice per step (front + rear wheel)."""
    rec  = RecordingRoad()
    moto = _make_moto(rec)
    moto.build()
    moto.reset(v0=6.0)

    N_STEPS = 5
    for k in range(N_STEPS):
        moto.step(k * DT)

    assert rec.call_count >= 2 * N_STEPS, (
        f"road.query() called {rec.call_count} times over {N_STEPS} steps; "
        f"expected ≥ {2 * N_STEPS} (front + rear per step)"
    )

    # Front and rear queries should differ in x (front wheel is ahead of rear)
    if len(rec.last_queries) >= 2:
        xs = [q[0] for q in rec.last_queries[-2:]]
        assert xs[0] != xs[1], (
            "Front and rear wheel queries returned same x — "
            "only one wheel is being queried"
        )


# ─────────────────────────────────────────────────────────────────────────────
# G2: FlatPlane(mu=1.0) preserves existing roll behaviour
# ─────────────────────────────────────────────────────────────────────────────

def test_g2_flat_mu1_no_change():
    """
    With FlatPlane(mu_dry=1.0) the roll perturbation still decays to < 3° in 2 s
    at v=6 m/s (same condition as M2 G4).

    This verifies that the road-query plumbing does not regress stability.
    """
    moto = _make_moto(FlatPlane(mu_dry=1.0))
    moto.build()
    moto.reset(v0=6.0)
    moto.main_frame.SetRot(chrono.QuatFromAngleX(math.radians(5.0)))

    n_steps = round(2.0 / DT)
    for k in range(n_steps):
        telem = moto.step(k * DT)

    final_roll = abs(math.degrees(telem["chassis"]["roll"]))
    assert final_roll < 3.0, (
        f"FlatPlane(mu=1.0): roll did not decay; "
        f"final |roll|={final_roll:.2f}° (limit 3°)"
    )


# ─────────────────────────────────────────────────────────────────────────────
# G3: low mu_dry reduces lateral force
# ─────────────────────────────────────────────────────────────────────────────

def test_g3_low_mu_reduces_fy():
    """
    FlatPlane(mu=0.5) produces a lower peak |Fy| than mu=1.0 during roll recovery.

    Physical interpretation: low-friction surface provides less restoring lateral
    force, so the tire cannot supply the same cornering force for the same slip.
    """
    fy_full = _peak_fy(mu=1.0, n_steps=300)
    fy_half = _peak_fy(mu=0.5, n_steps=300)

    assert fy_half < fy_full, (
        f"mu=0.5 peak Fy={fy_half:.1f} N is NOT less than mu=1.0 Fy={fy_full:.1f} N"
    )

    ratio = fy_half / max(fy_full, 1.0)
    assert ratio < 0.80, (
        f"mu=0.5 Fy ratio = {ratio:.3f}; expected < 0.80 (should be near 0.50). "
        "Road mu_dry scaling may not be wired to tire forces."
    )


# ─────────────────────────────────────────────────────────────────────────────
# G4: CurvedRoad straight section returns flat normal
# ─────────────────────────────────────────────────────────────────────────────

def test_g4_curved_flat_normal():
    """CurvedRoad.query() on the straight section returns (0, 0, 1)."""
    road = CurvedRoad(straight_length=180.0, corner_radius=10.0, corner_bank=0.3)

    # Centre of straight 1 is at (90, 0)
    s = road.query(90.0, 0.0)
    nx, ny, nz = s.normal
    assert abs(nx) < 1e-9 and abs(ny) < 1e-9 and abs(nz - 1.0) < 1e-9, (
        f"Straight section normal = {s.normal}; expected (0, 0, 1)"
    )


# ─────────────────────────────────────────────────────────────────────────────
# G5: CurvedRoad corner section returns banked normal
# ─────────────────────────────────────────────────────────────────────────────

def test_g5_curved_banked_normal():
    """CurvedRoad.query() on a corner section returns nz < 1 (banked road)."""
    bank = 0.20   # rad (~11.5°)
    road = CurvedRoad(straight_length=180.0, corner_radius=10.0, corner_bank=bank)

    # Point on corner 1 at its rightmost position: (L+R, R) = (190, 10)
    s = road.query(190.0, 10.0)
    nx, ny, nz = s.normal

    # Normal must be a unit vector
    length = math.sqrt(nx**2 + ny**2 + nz**2)
    assert abs(length - 1.0) < 1e-6, (
        f"Road normal is not a unit vector: {s.normal}, length={length:.6f}"
    )

    # Normal must be tilted (nz < 1 for bank > 0)
    assert nz < 0.999, (
        f"Corner normal nz = {nz:.6f} is not banked (expected nz < 0.999)"
    )

    # The Z component should match cos(bank)
    assert abs(nz - math.cos(bank)) < 0.01, (
        f"Corner normal nz = {nz:.4f}; expected cos({bank:.2f}) = {math.cos(bank):.4f}"
    )

    # Straight section must still be flat after querying the corner
    s2 = road.query(90.0, 0.0)
    assert abs(s2.normal[2] - 1.0) < 1e-9, (
        f"Straight normal changed after corner query: {s2.normal}"
    )


# ─────────────────────────────────────────────────────────────────────────────
# G6: step() telemetry contains 'road' dict
# ─────────────────────────────────────────────────────────────────────────────

def test_g6_road_in_telemetry():
    """step() returns a 'road' dict with front_mu, front_normal, rear_mu, rear_normal."""
    moto = _make_moto(FlatPlane(mu_dry=0.8))
    moto.build()
    moto.reset(v0=6.0)
    telem = moto.step(0.0)

    assert "road" in telem, "step() telemetry missing 'road' key"
    r = telem["road"]

    for key in ("front_mu", "front_normal", "front_height",
                "rear_mu",  "rear_normal",  "rear_height"):
        assert key in r, f"road telemetry missing '{key}'"

    # Flat road: mu = 0.8 as configured
    assert abs(r["front_mu"] - 0.8) < 1e-9, (
        f"front_mu = {r['front_mu']} in telemetry; expected 0.8"
    )
    # Normal should be (0, 0, 1) for FlatPlane
    assert abs(r["front_normal"][2] - 1.0) < 1e-9, (
        f"front_normal = {r['front_normal']}; expected (0, 0, 1)"
    )


# ─────────────────────────────────────────────────────────────────────────────
# G7: M3 regression
# ─────────────────────────────────────────────────────────────────────────────

def test_g7_m3_regression():
    """All 8 M3 gates pass (FlatPlane road, same conditions as M3 test suite)."""
    import tests.motorcycle.test_m3_validation as m3
    m3.test_g1_suspension_present()
    m3.test_g2_fork_travel_static()
    m3.test_g3_rear_travel_static()
    m3.test_g4_fork_compresses_braking()
    m3.test_g5_rear_compresses_accel()
    m3.test_g6_travel_finite()
    m3.test_g7_m1_regression()
    m3.test_g8_m2_regression()
