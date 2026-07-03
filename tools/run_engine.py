#!/usr/bin/env python3
"""
tools/run_engine.py  —  C++ 13-DOF Engine → WebSocket dashboard bridge
──────────────────────────────────────────────────────────────────────
Launches the compiled C++ engine in streaming mode, reads its JSON
telemetry from stdout, and pushes frames to the WebSocket server so
the Electron dashboard can display C++ engine data live.

Also forwards dashboard commands (pause/resume/reset/rider inputs)
back to the C++ engine via stdin.

Usage:
    python tools/run_engine.py
    python tools/run_engine.py --pinn                 # ONNX PINN tires
    python tools/run_engine.py --v0 15.0 --port 8765
    python tools/run_engine.py --headless             # no WS, print JSON only
    python tools/run_engine.py --bike cbr600rr.yaml   # load specific bike YAML
    python tools/run_engine.py --road wet_asphalt.yaml  # road conditions
    python tools/run_engine.py --list-bikes
    python tools/run_engine.py --list-roads

Then open the Electron dashboard and switch to "Live Simulation" tab.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT       = Path(__file__).resolve().parent.parent
ENGINE     = ROOT / "engine/build/moto_engine"
ORT_LIB    = Path("/home/dikshant/.local/lib/onnxruntime-1.24.4/lib")
DATASET    = ROOT / "dataset"
BIKE_DIR   = ROOT / "core/chassis/bike_params"
ROAD_DIR   = ROOT / "scenarios/roads"

sys.path.insert(0, str(ROOT))
import transport.ws_server as ws_server


def _find_engine() -> Path:
    if ENGINE.exists():
        return ENGINE
    # Try alternate build paths
    for alt in [ROOT / "engine/moto_engine",
                ROOT / "build/moto_engine"]:
        if alt.exists():
            return alt
    raise FileNotFoundError(
        f"C++ engine binary not found at {ENGINE}\n"
        "Build it first: cd engine/build && make -j4"
    )


def _engine_env() -> dict:
    env = os.environ.copy()
    ort_lib_str = str(ORT_LIB)
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{ort_lib_str}:{existing}" if existing else ort_lib_str
    return env


def _stdin_forwarder(proc: subprocess.Popen,
                     ws_params_fn,
                     stop_event: threading.Event) -> None:
    """
    Polls WebSocket params and commands, writes JSON to engine stdin.
    Runs at ~20 Hz (50 ms polling).
    """
    last_params: dict = {}

    while not stop_event.is_set():
        try:
            params  = ws_params_fn()
            reset   = ws_server.is_reset_requested()
            paused  = ws_server.is_paused()

            if reset:
                proc.stdin.write(json.dumps({"cmd": "reset"}) + "\n")
                proc.stdin.flush()
            elif paused:
                proc.stdin.write(json.dumps({"cmd": "pause"}) + "\n")
                proc.stdin.flush()
            else:
                proc.stdin.write(json.dumps({"cmd": "resume"}) + "\n")
                proc.stdin.flush()

            # Forward rider inputs if they changed
            rider = {
                "throttle":     float(params.get("throttle",     0.0)),
                "brake_front":  float(params.get("brake_front",  0.0)),
                "brake_rear":   float(params.get("brake_rear",   0.0)),
                "steer_torque": float(params.get("steer_torque", 0.0)),
                "lean_delta":   float(params.get("lean_delta",   0.0)),
            }
            if rider != last_params:
                proc.stdin.write(json.dumps(rider) + "\n")
                proc.stdin.flush()
                last_params = rider

        except (BrokenPipeError, OSError):
            break
        except Exception as e:
            print(f"[bridge] stdin forward error: {e}", file=sys.stderr)

        time.sleep(0.05)


def _resolve_bike(name: str) -> Path:
    """Resolve bike YAML path. Looks up relative to BIKE_DIR if not absolute."""
    p = Path(name)
    if p.is_absolute() and p.exists():
        return p
    # Try as-is
    if p.exists():
        return p.resolve()
    # Try relative to BIKE_DIR
    candidate = BIKE_DIR / name
    if candidate.exists():
        return candidate
    # Try with .yaml extension
    candidate2 = BIKE_DIR / (name + ".yaml")
    if candidate2.exists():
        return candidate2
    raise FileNotFoundError(f"Bike YAML not found: {name} (looked in {BIKE_DIR})")


def _resolve_road(name: str) -> Path:
    """Resolve road YAML path. Looks up relative to ROAD_DIR if not absolute."""
    p = Path(name)
    if p.is_absolute() and p.exists():
        return p
    if p.exists():
        return p.resolve()
    candidate = ROAD_DIR / name
    if candidate.exists():
        return candidate
    candidate2 = ROAD_DIR / (name + ".yaml")
    if candidate2.exists():
        return candidate2
    raise FileNotFoundError(f"Road YAML not found: {name} (looked in {ROAD_DIR})")


def main() -> None:
    parser = argparse.ArgumentParser(description="C++ engine → dashboard bridge")
    parser.add_argument("--pinn",     action="store_true",
                        help="Use ONNX PINN tire model (Phase 1)")
    parser.add_argument("--full-mf",  action="store_true",
                        help="Use full motorcycle MF tire (Phase 2.1 — large camber, gyro Mz, turn-slip)")
    parser.add_argument("--phase2",    action="store_true",
                        help="Use Phase 2 tire (FullMF + LuGre bristle + Hertzian contact patch)")
    parser.add_argument("--coeff-fit", action="store_true",
                        help="Enable Phase 3.1 PINN coefficient fitter (updates Pacejka B,C,D,E at 10 Hz)")
    parser.add_argument("--v0",       type=float, default=8.0,
                        help="Initial speed [m/s] (default 8.0)")
    parser.add_argument("--port",     type=int, default=8765)
    parser.add_argument("--headless", action="store_true",
                        help="Print JSON to stdout, no WebSocket")
    parser.add_argument("--bike",     type=str, default="",
                        help="Bike YAML file (name or path; resolved in core/chassis/bike_params/)")
    parser.add_argument("--road",     type=str, default="",
                        help="Road YAML file (name or path; resolved in scenarios/roads/)")
    parser.add_argument("--list-bikes", action="store_true",
                        help="List available bike YAML files and exit")
    parser.add_argument("--list-roads", action="store_true",
                        help="List available road YAML files and exit")
    args = parser.parse_args()

    if args.list_bikes:
        yamls = sorted(BIKE_DIR.glob("*.yaml"))
        if yamls:
            print("Available bikes:")
            for y in yamls:
                print(f"  {y.name}")
        else:
            print(f"No YAML files found in {BIKE_DIR}")
        return

    if args.list_roads:
        yamls = sorted(ROAD_DIR.glob("*.yaml"))
        if yamls:
            print("Available roads:")
            for y in yamls:
                print(f"  {y.name}")
        else:
            print(f"No YAML files found in {ROAD_DIR}")
        return

    engine_path = _find_engine()
    onnx_path   = DATASET / "tire_pinn_medium.onnx"

    # ── Build engine command ───────────────────────────────────────────────────
    cmd = [str(engine_path), "--stream", f"--v0", str(args.v0)]
    if args.pinn:
        if not onnx_path.exists():
            print(f"[bridge] ONNX model not found: {onnx_path}", file=sys.stderr)
            print("[bridge] Run 'python rig/train_pinn.py' first", file=sys.stderr)
            sys.exit(1)
        cmd += ["--pinn", str(onnx_path)]
    elif getattr(args, "phase2", False):
        cmd += ["--phase2"]
    elif getattr(args, "full_mf", False):
        cmd += ["--full-mf"]

    if args.bike:
        try:
            bike_path = _resolve_bike(args.bike)
            cmd += ["--bike", str(bike_path)]
            print(f"[bridge] Bike: {bike_path.name}")
        except FileNotFoundError as e:
            print(f"[bridge] ERROR: {e}", file=sys.stderr)
            sys.exit(1)

    if args.road:
        try:
            road_path = _resolve_road(args.road)
            cmd += ["--road", str(road_path)]
            print(f"[bridge] Road: {road_path.name}")
        except FileNotFoundError as e:
            print(f"[bridge] ERROR: {e}", file=sys.stderr)
            sys.exit(1)

    if args.pinn:
        tire_label = "ONNX PINN"
    elif getattr(args, "phase2", False):
        tire_label = "Phase2 (FullMF+LuGre+Hertz)"
    elif getattr(args, "full_mf", False):
        tire_label = "FullMF (large camber+gyro)"
    else:
        tire_label = "Pacejka"
    print(f"[bridge] C++ engine: {engine_path.name}  tires={tire_label}  v0={args.v0} m/s")

    # ── WebSocket server ───────────────────────────────────────────────────────
    if not args.headless:
        ws_server.init_params({
            "throttle":     0.0,
            "brake_front":  0.0,
            "brake_rear":   0.0,
            "steer_torque": 0.0,
            "lean_delta":   0.0,
            "v0":           args.v0,
        })
        ws_server.start(port=args.port)
        print(f"[bridge] WebSocket: ws://localhost:{args.port}")
        print(f"[bridge] Open the Electron dashboard and connect → Live Simulation tab")

    # ── Start C++ engine subprocess ────────────────────────────────────────────
    proc = subprocess.Popen(
        cmd,
        stdin =subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text  =True,
        bufsize=1,  # line-buffered
        env   =_engine_env(),
    )
    print(f"[bridge] Engine PID {proc.pid} started", flush=True)

    # ── Forward engine stderr to our stderr ────────────────────────────────────
    def _stderr_fwd():
        for line in proc.stderr:
            print(f"[engine] {line.rstrip()}", file=sys.stderr, flush=True)
    threading.Thread(target=_stderr_fwd, daemon=True).start()

    # ── stdin forwarder thread ─────────────────────────────────────────────────
    stop_event = threading.Event()
    if not args.headless:
        fwd = threading.Thread(
            target=_stdin_forwarder,
            args=(proc, ws_server.get_params, stop_event),
            daemon=True,
        )
        fwd.start()

    # ── Main loop: read JSON from engine, push to WebSocket ────────────────────
    frames_sent = 0
    t_last_log  = time.perf_counter()

    try:
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                continue

            if args.headless:
                print(line)   # pass through to stdout
            else:
                ws_server.push_frame(frame)

            frames_sent += 1
            now = time.perf_counter()
            if now - t_last_log >= 2.0:
                t = frame.get("t", 0)
                vx = frame.get("vx", 0)
                roll = frame.get("roll", 0) * 180 / 3.14159
                print(f"[bridge] t={t:.2f}s  vx={vx:.2f}m/s  roll={roll:.2f}°  "
                      f"frames_sent={frames_sent}", flush=True)
                t_last_log = now

    except KeyboardInterrupt:
        print("\n[bridge] Interrupted — shutting down", flush=True)
    finally:
        stop_event.set()
        proc.terminate()
        proc.wait(timeout=2)
        print(f"[bridge] Engine stopped.  Total frames sent: {frames_sent}", flush=True)


if __name__ == "__main__":
    main()
