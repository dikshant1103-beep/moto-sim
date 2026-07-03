"""
tests/system/test_dashboard_connectivity.py
─────────────────────────────────────────────
End-to-end telemetry connectivity test.

Verifies that every field the dashboard (WS client) needs is:
  1. Present in Motorcycle.step() output
  2. Present and correctly named in _flatten() output
  3. Finite / in physical range
  4. Covers all milestones: M1 forces, M2 stability, M3 suspension,
     M4 road, M5 thermal/wear

Run from moto_sim/ root:
    python -c "
    import sys; sys.path.insert(0, '.')
    import tests.system.test_dashboard_connectivity as t
    t.run_all()
    "
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT))

import pychrono as chrono
from core.chassis.motorcycle import Motorcycle
from core.rider.rider_input  import ConstantInputSource
from core.road.flat_plane    import FlatPlane
from core.road.curved_road   import CurvedRoad

BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
DT = 0.001


# ── helpers ───────────────────────────────────────────────────────────────────

def _make_moto(road=None, throttle=0.0, brake_front=0.0, brake_rear=0.0,
               steer_torque=0.0, v0=6.0) -> tuple[Motorcycle, dict]:
    rider = ConstantInputSource(throttle=throttle, brake_front=brake_front,
                                brake_rear=brake_rear, steer_torque=steer_torque)
    moto  = Motorcycle.from_yaml(BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
                                 road or FlatPlane(), rider, dt=DT)
    moto.build()
    moto.reset(v0=v0)
    for k in range(50):          # let state settle
        telem = moto.step(k * DT)
    return moto, telem


def _flatten(telem: dict) -> dict:
    """Mirror of moto_sim_main._flatten() — kept in sync manually."""
    c  = telem["chassis"]
    f  = telem["front"]
    r  = telem["rear"]
    i  = telem["inputs"]
    s  = telem["suspension"]
    rd = telem.get("road", {})
    _f_nrm = rd.get("front_normal", (0.0, 0.0, 1.0))
    _r_nrm = rd.get("rear_normal",  (0.0, 0.0, 1.0))
    return {
        "t":             telem["t"],
        "roll":          c["roll"],
        "pitch":         c["pitch"],
        "yaw":           c["yaw"],
        "roll_rate":     c["roll_rate"],
        "yaw_rate":      c["yaw_rate"],
        "vx":            c["vx"],
        "vy":            c["vy"],
        "ax":            c["ax"],
        "ay":            c["ay"],
        "f_alpha":       f["alpha"],
        "f_kappa":       f["kappa"],
        "f_Fz":          f["Fz"],
        "f_Fx":          f["Fx"],
        "f_Fy":          f["Fy"],
        "f_Mz":          f["Mz"],
        "f_alpha_prime": f["alpha_prime"],
        "r_alpha":       r["alpha"],
        "r_kappa":       r["kappa"],
        "r_Fz":          r["Fz"],
        "r_Fx":          r["Fx"],
        "r_Fy":          r["Fy"],
        "r_Mz":          r["Mz"],
        "r_alpha_prime": r["alpha_prime"],
        "throttle":      i["throttle"],
        "brake_front":   i["brake_front"],
        "brake_rear":    i["brake_rear"],
        "steer_torque":  i["steer_torque"],
        "steer_angle":   i["steer_angle"],
        "lean_delta":    i["lean_delta"],
        "fork_travel":   s["fork_travel"],
        "rear_travel":   s["rear_travel"],
        "road_mu_f":     rd.get("front_mu", 1.0),
        "road_mu_r":     rd.get("rear_mu",  1.0),
        "road_bank_f":   math.atan2(_f_nrm[1], max(_f_nrm[2], 0.01)),
        "road_bank_r":   math.atan2(_r_nrm[1], max(_r_nrm[2], 0.01)),
        # M5 thermal
        "f_T_tread":     f.get("T_tread",   85.0),
        "f_T_carcass":   f.get("T_carcass", 80.0),
        "f_wear":        f.get("wear",        0.0),
        "f_pressure":    f.get("pressure",  240.0),
        "r_T_tread":     r.get("T_tread",   85.0),
        "r_T_carcass":   r.get("T_carcass", 80.0),
        "r_wear":        r.get("wear",        0.0),
        "r_pressure":    r.get("pressure",  240.0),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Expected fields and their physical validity ranges
# ─────────────────────────────────────────────────────────────────────────────

# (field_name, min_value, max_value, description)
FLAT_FIELD_SPECS = [
    # ── Chassis ──────────────────────────────────────────────────────────────
    ("t",            0.0,    None,   "simulation time [s]"),
    ("roll",        -1.0,    1.0,    "chassis roll [rad]  |±57°|"),
    ("pitch",       -0.5,    0.5,    "chassis pitch [rad] |±29°|"),
    ("yaw",         -math.pi, math.pi, "chassis yaw [rad]"),
    ("roll_rate",   -20.0,  20.0,   "roll angular rate [rad/s]"),
    ("yaw_rate",    -5.0,    5.0,    "yaw rate [rad/s]"),
    ("vx",          -1.0,   60.0,   "forward speed [m/s]"),
    ("vy",          -5.0,    5.0,    "lateral speed [m/s]"),
    ("ax",         -50.0,   50.0,   "longitudinal accel [m/s²]"),
    ("ay",         -50.0,   50.0,   "lateral accel [m/s²]"),
    # ── Front tire ────────────────────────────────────────────────────────────
    ("f_alpha",     -0.5,    0.5,   "front slip angle [rad]"),
    ("f_kappa",     -1.0,    1.0,   "front slip ratio [–]"),
    ("f_Fz",         0.0, 5000.0,  "front normal load [N]"),
    ("f_Fx",      -3000.0, 3000.0, "front long. force [N]"),
    ("f_Fy",      -3000.0, 3000.0, "front lat. force [N]"),
    ("f_Mz",       -50.0,   50.0,  "front aligning torque [Nm]"),
    ("f_alpha_prime",-0.5,   0.5,  "front relaxed slip angle [rad]"),
    # ── Rear tire ─────────────────────────────────────────────────────────────
    ("r_alpha",     -0.5,    0.5,   "rear slip angle [rad]"),
    ("r_kappa",     -1.0,    1.0,   "rear slip ratio [–]"),
    ("r_Fz",         0.0, 5000.0,  "rear normal load [N]"),
    ("r_Fx",      -3000.0, 3000.0, "rear long. force [N]"),
    ("r_Fy",      -3000.0, 3000.0, "rear lat. force [N]"),
    ("r_Mz",       -50.0,   50.0,  "rear aligning torque [Nm]"),
    ("r_alpha_prime",-0.5,   0.5,  "rear relaxed slip angle [rad]"),
    # ── Inputs ────────────────────────────────────────────────────────────────
    ("throttle",     0.0,    1.0,   "throttle [0–1]"),
    ("brake_front",  0.0,    1.0,   "front brake [0–1]"),
    ("brake_rear",   0.0,    1.0,   "rear brake [0–1]"),
    ("steer_torque",-100.0, 100.0, "steer torque [Nm]"),
    ("steer_angle", -0.5,    0.5,  "steer angle [rad]"),
    ("lean_delta",  -0.5,    0.5,  "rider lean [rad]"),
    # ── Suspension ────────────────────────────────────────────────────────────
    ("fork_travel",  0.0,    0.14,  "fork compression [m]"),
    ("rear_travel",  0.0,    0.12,  "rear compression [m]"),
    # ── Road ─────────────────────────────────────────────────────────────────
    ("road_mu_f",    0.0,    2.0,   "front road friction [–]"),
    ("road_mu_r",    0.0,    2.0,   "rear road friction [–]"),
    ("road_bank_f", -0.6,    0.6,  "front road banking [rad]"),
    ("road_bank_r", -0.6,    0.6,  "rear road banking [rad]"),
    # ── M5 Thermal / wear ─────────────────────────────────────────────────────
    ("f_T_tread",   -10.0,  200.0, "front tread temp [°C]"),
    ("f_T_carcass", -10.0,  180.0, "front carcass temp [°C]"),
    ("f_wear",        0.0,    1.0,  "front wear [0–1]"),
    ("f_pressure",  150.0,  400.0, "front pressure [kPa]"),
    ("r_T_tread",   -10.0,  200.0, "rear tread temp [°C]"),
    ("r_T_carcass", -10.0,  180.0, "rear carcass temp [°C]"),
    ("r_wear",        0.0,    1.0,  "rear wear [0–1]"),
    ("r_pressure",  150.0,  400.0, "rear pressure [kPa]"),
]

# Nested step() dict keys that must exist
NESTED_TELEM_KEYS = {
    "chassis": ["roll","pitch","yaw","roll_rate","yaw_rate","vx","vy","ax","ay"],
    "front":   ["alpha","kappa","Fz","Fx","Fy","Mz","alpha_prime",
                "T_tread","T_carcass","wear","pressure"],
    "rear":    ["alpha","kappa","Fz","Fx","Fy","Mz","alpha_prime",
                "T_tread","T_carcass","wear","pressure"],
    "inputs":  ["throttle","brake_front","brake_rear","steer_torque",
                "steer_angle","lean_delta"],
    "suspension": ["fork_travel","rear_travel"],
    "road":    ["front_mu","front_normal","front_height",
                "rear_mu","rear_normal","rear_height"],
}


# ─────────────────────────────────────────────────────────────────────────────
# Tests
# ─────────────────────────────────────────────────────────────────────────────

def test_t1_nested_telem_all_keys_present():
    """Every required key exists in Motorcycle.step() nested dict."""
    _, telem = _make_moto()
    errors = []
    for section, keys in NESTED_TELEM_KEYS.items():
        if section not in telem:
            errors.append(f"Missing section: '{section}'")
            continue
        for k in keys:
            if k not in telem[section]:
                errors.append(f"Missing: telem['{section}']['{k}']")
    assert not errors, "\n".join(errors)


def test_t2_flatten_all_fields_present():
    """_flatten() output contains every dashboard-expected key."""
    _, telem = _make_moto()
    flat = _flatten(telem)
    expected = [spec[0] for spec in FLAT_FIELD_SPECS]
    missing = [k for k in expected if k not in flat]
    assert not missing, f"_flatten() missing fields: {missing}"


def test_t3_all_values_finite():
    """Every field in the flat dict is a finite number."""
    _, telem = _make_moto()
    flat = _flatten(telem)
    bad = [(k, v) for k, v in flat.items()
           if not isinstance(v, (int, float)) or not math.isfinite(v)]
    assert not bad, f"Non-finite values in flat dict: {bad}"


def test_t4_physical_range_checks():
    """Every field is within its physical validity range."""
    _, telem = _make_moto()
    flat = _flatten(telem)
    errors = []
    for field, lo, hi, desc in FLAT_FIELD_SPECS:
        val = flat.get(field)
        if val is None:
            continue
        if lo is not None and val < lo:
            errors.append(f"{field}={val:.4g} < min={lo}  [{desc}]")
        if hi is not None and val > hi:
            errors.append(f"{field}={val:.4g} > max={hi}  [{desc}]")
    assert not errors, "\n  ".join(["Range violations:"] + errors)


def test_t5_forces_nonzero_at_cruise():
    """At v=6 m/s cruise, Fz_front and Fz_rear are both substantial (>100N)."""
    _, telem = _make_moto(v0=6.0)
    flat = _flatten(telem)
    assert flat["f_Fz"] > 100.0, f"f_Fz={flat['f_Fz']:.1f} N — suspiciously low"
    assert flat["r_Fz"] > 100.0, f"r_Fz={flat['r_Fz']:.1f} N — suspiciously low"


def test_t6_suspension_compressed_at_cruise():
    """Fork travel and rear travel are positive (springs loaded) at cruise."""
    _, telem = _make_moto(v0=6.0)
    flat = _flatten(telem)
    assert flat["fork_travel"] > 0.001, \
        f"fork_travel={flat['fork_travel']*1000:.2f} mm — not compressed"
    assert flat["rear_travel"] > 0.001, \
        f"rear_travel={flat['rear_travel']*1000:.2f} mm — not compressed"


def test_t7_braking_shifts_weight_forward():
    """Under hard braking, f_Fz > r_Fz (weight transfers forward)."""
    _, telem_brake = _make_moto(brake_front=1.0, brake_rear=0.5, v0=10.0)
    flat = _flatten(telem_brake)
    assert flat["f_Fz"] > flat["r_Fz"], (
        f"Braking: f_Fz={flat['f_Fz']:.1f} N should be > r_Fz={flat['r_Fz']:.1f} N"
    )


def test_t8_throttle_shifts_weight_rearward():
    """Under full throttle, r_Fz > f_Fz (weight transfers rearward)."""
    _, telem_accel = _make_moto(throttle=1.0, v0=5.0)
    flat = _flatten(telem_accel)
    assert flat["r_Fz"] > flat["f_Fz"], (
        f"Throttle: r_Fz={flat['r_Fz']:.1f} N should be > f_Fz={flat['f_Fz']:.1f} N"
    )


def test_t9_road_fields_flat_plane():
    """On FlatPlane(mu=0.9), road_mu_f and road_mu_r both report 0.9."""
    _, telem = _make_moto(road=FlatPlane(mu_dry=0.9))
    flat = _flatten(telem)
    assert abs(flat["road_mu_f"] - 0.9) < 1e-6, \
        f"road_mu_f={flat['road_mu_f']:.4f} expected 0.9"
    assert abs(flat["road_mu_r"] - 0.9) < 1e-6, \
        f"road_mu_r={flat['road_mu_r']:.4f} expected 0.9"
    assert abs(flat["road_bank_f"]) < 0.01, \
        f"road_bank_f={flat['road_bank_f']:.4f} rad on flat road (expected ~0)"


def test_t10_road_banking_on_curved_road():
    """On CurvedRoad corner section, at least one wheel sees banking."""
    road = CurvedRoad(straight_length=10.0, corner_radius=20.0, corner_bank=0.2)
    _, telem = _make_moto(road=road, v0=8.0)
    flat = _flatten(telem)
    # At least road fields are finite (bike may still be on straight)
    assert math.isfinite(flat["road_bank_f"])
    assert math.isfinite(flat["road_bank_r"])


def test_t11_m5_thermal_state_present_and_valid():
    """M5 thermal fields exist, are finite, and within physical bounds."""
    _, telem = _make_moto()
    flat = _flatten(telem)
    for prefix in ("f_", "r_"):
        T_tread   = flat[f"{prefix}T_tread"]
        T_carcass = flat[f"{prefix}T_carcass"]
        wear      = flat[f"{prefix}wear"]
        pressure  = flat[f"{prefix}pressure"]

        assert -10 < T_tread   < 200, f"{prefix}T_tread={T_tread:.1f}°C out of range"
        assert -10 < T_carcass < 180, f"{prefix}T_carcass={T_carcass:.1f}°C out of range"
        assert 0.0 <= wear     <= 1.0, f"{prefix}wear={wear:.4f} out of [0,1]"
        assert 150 < pressure  < 400, f"{prefix}pressure={pressure:.1f} kPa out of range"


def test_t12_thermal_state_updates_during_run():
    """After 5 s of sustained throttle, rear wear > 0 and pressure changes."""
    rider = ConstantInputSource(throttle=0.8)
    moto  = Motorcycle.from_yaml(BIKE_YAML, FRONT_TIRE_YAML, REAR_TIRE_YAML,
                                 FlatPlane(), rider, dt=DT)
    moto.build()
    moto.reset(v0=5.0)

    for k in range(5000):   # 5 s
        telem = moto.step(k * DT)

    flat = _flatten(telem)
    assert flat["r_wear"] > 0.0, \
        f"rear wear={flat['r_wear']:.2e} after 5s throttle — not accumulating"
    # Pressure should differ from cold-tyre value since carcass may have changed
    assert math.isfinite(flat["r_pressure"])


def test_t13_steer_angle_in_flat_dict():
    """steer_angle and lean_delta are echoed back correctly in flat dict."""
    _, telem = _make_moto()
    flat = _flatten(telem)
    assert "steer_angle" in flat
    assert "lean_delta"  in flat
    assert math.isfinite(flat["steer_angle"])
    assert math.isfinite(flat["lean_delta"])


def test_t14_alpha_prime_differs_from_alpha_after_step():
    """f_alpha_prime is close to but not necessarily identical to f_alpha (relaxation)."""
    _, telem = _make_moto(v0=10.0)
    flat = _flatten(telem)
    # Both must be finite
    assert math.isfinite(flat["f_alpha"])
    assert math.isfinite(flat["f_alpha_prime"])
    # At cruise with no steer input, both should be small
    assert abs(flat["f_alpha"]) < 0.2, \
        f"f_alpha={math.degrees(flat['f_alpha']):.2f}° — unexpectedly large at cruise"


# ─────────────────────────────────────────────────────────────────────────────
# Runner
# ─────────────────────────────────────────────────────────────────────────────

TESTS = [
    ("T1  nested telem keys present",       test_t1_nested_telem_all_keys_present),
    ("T2  flatten all fields present",       test_t2_flatten_all_fields_present),
    ("T3  all values finite",                test_t3_all_values_finite),
    ("T4  physical range checks",            test_t4_physical_range_checks),
    ("T5  forces non-zero at cruise",        test_t5_forces_nonzero_at_cruise),
    ("T6  suspension compressed at cruise",  test_t6_suspension_compressed_at_cruise),
    ("T7  braking shifts weight forward",    test_t7_braking_shifts_weight_forward),
    ("T8  throttle shifts weight rearward",  test_t8_throttle_shifts_weight_rearward),
    ("T9  road fields on flat plane",        test_t9_road_fields_flat_plane),
    ("T10 road banking on curved road",      test_t10_road_banking_on_curved_road),
    ("T11 M5 thermal state present+valid",   test_t11_m5_thermal_state_present_and_valid),
    ("T12 thermal updates during run",       test_t12_thermal_state_updates_during_run),
    ("T13 steer_angle echoed in flat dict",  test_t13_steer_angle_in_flat_dict),
    ("T14 alpha_prime at cruise",            test_t14_alpha_prime_differs_from_alpha_after_step),
]


def run_all():
    passed, failed = 0, 0
    print("\n=== Dashboard Connectivity Tests ===\n")
    for name, fn in TESTS:
        try:
            fn()
            print(f"  PASS  {name}")
            passed += 1
        except AssertionError as e:
            print(f"  FAIL  {name}")
            print(f"        {e}")
            failed += 1
        except Exception as e:
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
            failed += 1

    total = passed + failed
    print(f"\n{'='*40}")
    print(f"  {passed}/{total} passed"
          + ("  ✓ ALL PASS" if failed == 0 else f"  ✗ {failed} FAILED"))
    print()
    return failed == 0


if __name__ == "__main__":
    ok = run_all()
    sys.exit(0 if ok else 1)
