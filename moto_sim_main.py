"""
moto_sim_main.py — Motorcycle simulator entry point.

Run from moto_sim/ root:
    python moto_sim_main.py
    python moto_sim_main.py --headless          # no WS server
    python moto_sim_main.py --v0 10.0           # initial speed [m/s]
    python moto_sim_main.py --steps 5000        # fixed-length run (then exit)

Simulation loop: 1 kHz physics, 50 Hz WS telemetry.
    Rider inputs  ← WS params dict (dashboard sliders / gamepad)
    Telemetry     → WS frame (flat dict, ~50 Hz)
    Reset signal  ← WS cmd "reset"
    Pause/Resume  ← WS cmd "pause" / "resume"
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

# ── Path setup ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))

from core.chassis.motorcycle import Motorcycle
from core.chassis.motorcycle import BikeParams
from core.tire.tire_interface import TireModel
from core.tire.onnx_tire      import OnnxTireModel
from core.rider.rider_input  import DashboardInputSource, ScenarioInputSource, SpeedControlSource
from core.road.flat_plane    import FlatPlane
from core.road.curved_road   import CurvedRoad
import transport.ws_server as ws_server

# ── Constants ─────────────────────────────────────────────────────────────────
BIKE_YAML       = ROOT / "core/chassis/bike_params/cbr600rr.yaml"
FRONT_TIRE_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"
REAR_TIRE_YAML  = ROOT / "core/tire/pacejka_coeffs/medium.yaml"

DT         = 0.001   # physics timestep [s]
WS_SEND_HZ = 50      # telemetry rate [Hz]
WS_EVERY   = max(1, round(1.0 / (DT * WS_SEND_HZ)))   # steps per WS frame = 20

BASE_BIKE_DICT = None


def _load_base_bike_dict() -> dict:
    global BASE_BIKE_DICT
    if BASE_BIKE_DICT is None:
        import yaml
        with open(BIKE_YAML) as f:
            BASE_BIKE_DICT = yaml.safe_load(f)
    return dict(BASE_BIKE_DICT)


def _build_runtime_setup(params: dict) -> tuple[dict, FlatPlane]:
    bike = _load_base_bike_dict()
    front_radius = max(0.1, float(params.get("frontWheelDiameter", bike["front_wheel_radius"] * 2.0)) / 2.0)
    rear_radius  = max(0.1, float(params.get("rearWheelDiameter", bike["rear_wheel_radius"] * 2.0)) / 2.0)
    wheelbase    = max(0.8, float(params.get("wheelbase", bike["wheelbase"])))
    front_frac   = min(0.8, max(0.2, float(params.get("frontWeightFraction", bike["cg_x"] / bike["wheelbase"]))))

    bike.update({
        "wheelbase": wheelbase,
        "front_wheel_radius": front_radius,
        "rear_wheel_radius": rear_radius,
        "rake": math.radians(float(params.get("rakeDegrees", math.degrees(bike["rake"])))),
        "trail": max(0.03, float(params.get("trail", bike["trail"]))),
        "cg_height": max(0.2, float(params.get("centreOfGravityHeight", bike["cg_height"]))),
        "total_mass": max(80.0, float(params.get("totalMass", bike["total_mass"]))),
        "front_spring_k": max(1000.0, float(params.get("frontSpringRate", bike["front_spring_k"]))),
        "rear_spring_k": max(1000.0, float(params.get("rearSpringRate", bike["rear_spring_k"]))),
        "front_damper_c": max(50.0, float(params.get("frontDamping", bike["front_damper_c"]))),
        "rear_damper_c": max(50.0, float(params.get("rearDamping", bike["rear_damper_c"]))),
        "engine_torque_max": max(0.0, float(params.get("engineTorqueMaximum", bike.get("engine_torque_max", 80.0)))),
        "brake_torque_max": max(50.0, float(params.get("brakeTorqueMaximum", bike.get("brake_torque_max", 250.0)))),
        "rider_mass": max(40.0, float(params.get("riderMass", bike.get("rider_mass", 75.0)))),
        "rider_lean_arm": max(0.1, float(params.get("riderLeanArm", bike.get("rider_lean_arm", 0.55)))),
        "cg_x": wheelbase * front_frac,
    })
    mu_dry      = max(0.2, float(params.get("frictionCoefficient", 1.0)))
    track_mode  = params.get("trackMode", "flat")

    if track_mode == "curved":
        road = CurvedRoad(
            straight_length = max(20.0, float(params.get("straightLength", 50.0))),
            corner_radius   = max(5.0,  float(params.get("cornerRadius", 20.0))),
            corner_bank     = float(params.get("cornerBankAngle", 0.15)),
            mu_dry          = mu_dry,
        )
    else:
        road = FlatPlane(mu_dry=mu_dry)

    return bike, road


_USE_PINN = False   # set True via --pinn flag

def _make_tire(compound: str = "medium"):
    """Return an OnnxTireModel if --pinn flag is set, else analytical TireModel."""
    if _USE_PINN:
        try:
            return OnnxTireModel.from_compound(compound)
        except Exception as e:
            print(f"[moto_sim] PINN load failed ({e}), falling back to Pacejka")
    return TireModel.from_yaml(ROOT / f"core/tire/pacejka_coeffs/{compound}.yaml")


def _make_motorcycle(params: dict, rider_source) -> Motorcycle:
    bike_dict, road = _build_runtime_setup(params)
    return Motorcycle(
        BikeParams(bike_dict),
        _make_tire(),
        _make_tire(),
        road,
        rider_source,
        dt=DT,
    )


# ── Telemetry flattening ──────────────────────────────────────────────────────

def _flatten(telem: dict) -> dict:
    """Flatten nested Motorcycle.step() telemetry to a flat JSON-serialisable dict."""
    c  = telem["chassis"]
    f  = telem["front"]
    r  = telem["rear"]
    i  = telem["inputs"]
    s  = telem["suspension"]
    rd = telem.get("road", {})
    _f_nrm = rd.get("front_normal", (0.0, 0.0, 1.0))
    _r_nrm = rd.get("rear_normal",  (0.0, 0.0, 1.0))
    return {
        "t":            telem["t"],
        # Chassis — angles in radians; JS components multiply by R2D for display
        "roll":         c["roll"],
        "pitch":        c["pitch"],
        "yaw":          c["yaw"],
        "roll_rate":    c["roll_rate"],
        "yaw_rate":     c["yaw_rate"],
        "vx":           c["vx"],
        "vy":           c["vy"],
        "ax":           c["ax"],
        "ay":           c["ay"],
        # Front tire — slip angles in radians
        "f_alpha":      f["alpha"],
        "f_kappa":      f["kappa"],
        "f_Fz":         f["Fz"],
        "f_Fx":         f["Fx"],
        "f_Fy":         f["Fy"],
        "f_Mz":         f["Mz"],
        "f_alpha_prime": f["alpha_prime"],
        # Rear tire — slip angles in radians
        "r_alpha":      r["alpha"],
        "r_kappa":      r["kappa"],
        "r_Fz":         r["Fz"],
        "r_Fx":         r["Fx"],
        "r_Fy":         r["Fy"],
        "r_Mz":         r["Mz"],
        "r_alpha_prime": r["alpha_prime"],
        # Rider inputs (echo back to dashboard)
        "throttle":     i["throttle"],
        "brake_front":  i["brake_front"],
        "brake_rear":   i["brake_rear"],
        "steer_torque": i["steer_torque"],
        "steer_angle":  i["steer_angle"],
        "lean_delta":   i["lean_delta"],
        # M3: Suspension travel [m], positive = compressed
        "fork_travel":  s["fork_travel"],
        "rear_travel":  s["rear_travel"],
        # M4-road: road surface at each wheel
        "road_mu_f":    rd.get("front_mu", 1.0),
        "road_mu_r":    rd.get("rear_mu",  1.0),
        "road_bank_f":  math.atan2(_f_nrm[1], max(_f_nrm[2], 0.01)),  # banking [rad]
        "road_bank_r":  math.atan2(_r_nrm[1], max(_r_nrm[2], 0.01)),
        # M5: Thermal / wear state — front tire
        "f_T_tread":    f.get("T_tread",   85.0),
        "f_T_carcass":  f.get("T_carcass", 80.0),
        "f_wear":       f.get("wear",       0.0),
        "f_pressure":   f.get("pressure",  240.0),
        # M5: Thermal / wear state — rear tire
        "r_T_tread":    r.get("T_tread",   85.0),
        "r_T_carcass":  r.get("T_carcass", 80.0),
        "r_wear":       r.get("wear",       0.0),
        "r_pressure":   r.get("pressure",  240.0),
    }


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="moto_sim motorcycle simulator")
    parser.add_argument("--headless", action="store_true",
                        help="Disable WS server (no dashboard)")
    parser.add_argument("--v0",      type=float, default=5.0,
                        help="Initial speed [m/s]  (default 5.0)")
    parser.add_argument("--steps",   type=int,   default=0,
                        help="Run N steps then exit (0 = run forever)")
    parser.add_argument("--port",    type=int,   default=8765)
    parser.add_argument("--pinn",   action="store_true",
                        help="Use ONNX PINN tire model instead of analytical Pacejka")
    parser.add_argument("--scenario", type=str,  default=None,
                        help="Path to scenario YAML for open-loop playback")
    args = parser.parse_args()

    global _USE_PINN
    _USE_PINN = args.pinn
    tire_backend = "ONNX PINN" if _USE_PINN else "Pacejka analytical"

    # ── WS server ─────────────────────────────────────────────────────────────
    if not args.headless:
        ws_server.init_params({
            "throttle":     0.0,    # [0, 1]
            "brake_front":  0.0,    # [0, 1]
            "brake_rear":   0.0,    # [0, 1]
            "steer_torque": 0.0,    # [Nm]  −50 … +50
            "lean_delta":   0.0,    # [rad] rider upper-body lean  −0.3 … +0.3
            "steer_target": 0.0,    # [rad] commanded steady-turn steer angle  −0.4 … +0.4
            "target_speed": 0.0,    # [m/s] cruise-control target; 0 = disabled
            "v0":           args.v0, # reset speed [m/s]
            "wheelbase": 1.37,
            "frontWheelDiameter": 0.598,
            "rearWheelDiameter": 0.636,
            "frontTyreWidth": 0.12,
            "rearTyreWidth": 0.18,
            "rakeDegrees": 24.0,
            "trail": 0.097,
            "centreOfGravityHeight": 0.58,
            "totalMass": 189.0,
            "frontWeightFraction": 0.46,
            "engineTorqueMaximum": 80.0,
            "brakeTorqueMaximum": 250.0,
            "frontSpringRate": 18500.0,
            "rearSpringRate": 42000.0,
            "frontDamping": 900.0,
            "rearDamping": 1800.0,
            "trackLength": 400.0,
            "cornerRadius": 80.0,
            "bumpHeight": 0.015,
            "potholeDepth": 0.03,
            "frictionCoefficient": 0.9,
            "simulationSpeed": 12.0,
            "trackMode": "flat",
            "straightLength": 50.0,
            "cornerBankAngle": 0.15,
            "riderMass": 75.0,
            "riderLeanArm": 0.55,
        })
        ws_server.start(port=args.port)

    # ── Motorcycle ────────────────────────────────────────────────────────────
    # Shared mutable cell — updated after every physics step so SpeedControlSource
    # can read the most-recent forward speed without changing the RiderInput API.
    _vx_shared: list[float] = [args.v0]

    if args.scenario:
        # Open-loop: play back the YAML file, ignore dashboard sliders
        rider_source = ScenarioInputSource(args.scenario)
        print(f"[moto_sim] Scenario mode: {args.scenario}")
    elif not args.headless:
        _base = DashboardInputSource(lambda: ws_server.get_params())
        rider_source = SpeedControlSource(
            _base,
            vx_fn    = lambda: _vx_shared[0],
            params_fn= lambda: ws_server.get_params(),
            dt       = DT,
        )
    else:
        rider_source = _ConstZeroSource()

    setup_params = ws_server.get_params() if not args.headless else {"v0": args.v0}
    moto = _make_motorcycle(setup_params, rider_source)
    moto.build()
    moto.reset(v0=args.v0)

    print(f"[moto_sim] Starting at v0={args.v0} m/s  dt={DT*1000:.1f} ms  "
          f"tire={tire_backend}  "
          f"WS={'disabled' if args.headless else f'ws://localhost:{args.port}'}")
    # Capability banner — confirms this sim build has the steer_target turning fix.
    # If you DON'T see this line in the log, you're running an OLD sim → restart it.
    print("[moto_sim] CAPABILITY: steer_target turning ENABLED (v2 cornering build)")

    # ── Simulation loop ───────────────────────────────────────────────────────
    step       = 0
    t_wall_0   = time.perf_counter()
    log_every  = round(1.0 / DT)    # console log once per simulated second
    run_forever = args.steps == 0

    while run_forever or step < args.steps:

        t = step * DT

        # ── Reset ─────────────────────────────────────────────────────────
        if not args.headless and ws_server.is_reset_requested():
            current_params = ws_server.get_params()
            v0_new = current_params.get("v0", args.v0)
            moto = _make_motorcycle(current_params, rider_source)
            moto.build()
            moto.reset(v0=v0_new)
            step      = 0
            t_wall_0  = time.perf_counter()
            print(f"[moto_sim] Reset — v0={v0_new:.1f} m/s")
            continue

        # ── Pause ─────────────────────────────────────────────────────────
        if not args.headless and ws_server.is_paused():
            time.sleep(DT)
            continue

        # ── Physics step ──────────────────────────────────────────────────
        telem = moto.step(t)

        # ── NaN/divergence guard: auto-recover instead of freezing ────────
        # If an over-aggressive command pushes the simplified stability model
        # out of its valid range the PyChrono state can blow up to NaN. Detect
        # it and reset the bike (keeps the sim/telemetry alive instead of
        # "everything stopped"). The roam controller also commands gently to
        # avoid triggering this in the first place.
        _c = telem["chassis"]
        if not (math.isfinite(_c["roll"]) and math.isfinite(_c["vx"])
                and math.isfinite(_c["yaw_rate"])):
            print("[moto_sim] ⚠ state diverged (NaN) — auto-resetting bike", flush=True)
            moto.reset(v0=args.v0)
            step += 1
            continue

        _vx_shared[0] = telem["chassis"]["vx"]

        # ── Telemetry push ────────────────────────────────────────────────
        if not args.headless and step % WS_EVERY == 0:
            ws_server.push_frame(_flatten(telem))

        # ── Console log (every simulated second) ──────────────────────────
        if step % log_every == 0:
            c = telem["chassis"]
            f = telem["front"]
            r = telem["rear"]
            print(
                f"t={t:6.2f}s  vx={c['vx']:5.2f}  "
                f"roll={math.degrees(c['roll']):+6.2f}°  "
                f"yaw_rate={math.degrees(c['yaw_rate']):+6.2f}°/s  "
                f"Fz_f={f['Fz']:.0f}N  Fz_r={r['Fz']:.0f}N",
                flush=True,
            )

        step += 1

        # ── Real-time pacing ──────────────────────────────────────────────
        # Only pace when running interactively (not in fixed-step mode).
        if run_forever:
            t_sim     = step * DT
            t_elapsed = time.perf_counter() - t_wall_0
            slack     = t_sim - t_elapsed
            if slack > 1e-4:
                time.sleep(slack)

    print(f"[moto_sim] Done — {step} steps  ({step*DT:.3f} s simulated)")


# ── Headless zero-input rider ─────────────────────────────────────────────────

class _ConstZeroSource:
    """Zero rider inputs — used in --headless mode."""
    def get(self, t: float):
        from core.rider.rider_input import RiderInput
        return RiderInput()


if __name__ == "__main__":
    main()
