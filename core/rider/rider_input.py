"""
core/rider/rider_input.py — Rider input layer for the motorcycle simulator.

Milestone 1 (v1): rigid rider, inputs come from dashboard sliders or gamepad.
Milestone 3 (M3): scenario YAML playback, lean_delta physics, speed control.

Inputs
──────
throttle    [0, 1]   → rear wheel drive torque = throttle · T_max
brake_front [0, 1]   → front wheel brake torque
brake_rear  [0, 1]   → rear wheel brake torque
steer_torque [Nm]    → torque applied about steering axis
lean_delta   [rad]   → rider upper-body lean; shifts roll equilibrium by this angle
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class RiderInput:
    """Instantaneous rider demand (all values normalised or in SI units)."""
    throttle:     float = 0.0   # [0, 1]
    brake_front:  float = 0.0   # [0, 1]
    brake_rear:   float = 0.0   # [0, 1]
    steer_torque: float = 0.0   # [Nm]   positive = right steer
    lean_delta:   float = 0.0   # [rad]  upper-body lean; positive = lean right
    steer_target: float = 0.0   # [rad]  commanded steady-turn steer angle (0 = straight)

    def clamp(self) -> "RiderInput":
        """Return a copy with all values clamped to physical bounds."""
        return RiderInput(
            throttle     = max(0.0, min(1.0, self.throttle)),
            brake_front  = max(0.0, min(1.0, self.brake_front)),
            brake_rear   = max(0.0, min(1.0, self.brake_rear)),
            steer_torque = max(-50.0, min(50.0, self.steer_torque)),
            lean_delta   = max(-0.3,  min(0.3,  self.lean_delta)),
            steer_target = max(-0.4,  min(0.4,  self.steer_target)),
        )


class RiderInputSource:
    """
    Base class for rider input providers.

    Concrete implementations:
    - ConstantInputSource:  fixed inputs (tests / trim)
    - DashboardInputSource: reads from ws_server param dict (M1)
    - ScenarioInputSource:  plays back time-series from YAML (M3)
    - SpeedControlSource:   wraps any source and overrides throttle/brake (M3)
    """

    def get(self, t: float) -> RiderInput:
        """Return rider input at simulation time t [s]."""
        raise NotImplementedError


class ConstantInputSource(RiderInputSource):
    """Fixed inputs — useful for trim and straight-line tests."""

    def __init__(self, **kwargs) -> None:
        self._input = RiderInput(**kwargs)

    def get(self, t: float) -> RiderInput:
        return self._input


class DashboardInputSource(RiderInputSource):
    """
    Reads live inputs from the WebSocket parameter dict.

    Expected keys in params dict (from dashboard sliders):
      throttle, brake_front, brake_rear, steer_torque, lean_delta
    """

    def __init__(self, params_fn: Callable[[], dict]) -> None:
        self._params_fn = params_fn

    def get(self, t: float) -> RiderInput:
        p = self._params_fn()
        return RiderInput(
            throttle     = float(p.get("throttle",     0.0)),
            brake_front  = float(p.get("brake_front",  0.0)),
            brake_rear   = float(p.get("brake_rear",   0.0)),
            steer_torque = float(p.get("steer_torque", 0.0)),
            lean_delta   = float(p.get("lean_delta",   0.0)),
            steer_target = float(p.get("steer_target", 0.0)),
        ).clamp()


class ScenarioInputSource(RiderInputSource):
    """
    Plays back a time-series YAML scenario file (open-loop control).

    YAML format — columnar, all arrays must have equal length:

        t:             [0.0, 1.0, 3.0, 5.0]
        throttle:      [0.0, 0.8, 0.8, 0.0]
        brake_front:   [0.0, 0.0, 0.0, 1.0]
        brake_rear:    [0.0, 0.0, 0.0, 0.5]
        steer_torque:  [0.0, 0.0, 5.0, 0.0]
        lean_delta:    [0.0, 0.0, 0.05, 0.0]   # optional

    Values are linearly interpolated between time steps.
    Before t[0] the first entry is held; after t[-1] the last entry is held.
    """

    def __init__(self, yaml_path: str) -> None:
        import yaml as _yaml
        with open(yaml_path) as f:
            d = _yaml.safe_load(f)

        n = len(d["t"])
        self._t           = [float(v) for v in d["t"]]
        self._throttle    = [float(v) for v in d.get("throttle",    [0.0] * n)]
        self._brake_front = [float(v) for v in d.get("brake_front", [0.0] * n)]
        self._brake_rear  = [float(v) for v in d.get("brake_rear",  [0.0] * n)]
        self._steer       = [float(v) for v in d.get("steer_torque",[0.0] * n)]
        self._lean        = [float(v) for v in d.get("lean_delta",  [0.0] * n)]

        if n < 2:
            raise ValueError(f"Scenario '{yaml_path}' must have at least 2 time entries")
        if not all(len(lst) == n for lst in [
            self._throttle, self._brake_front, self._brake_rear,
            self._steer, self._lean,
        ]):
            raise ValueError(f"Scenario '{yaml_path}': all columns must have the same length as t")

    @staticmethod
    def _interp(ts: list, vals: list, t: float) -> float:
        if t <= ts[0]:
            return vals[0]
        if t >= ts[-1]:
            return vals[-1]
        for i in range(len(ts) - 1):
            if ts[i] <= t < ts[i + 1]:
                frac = (t - ts[i]) / (ts[i + 1] - ts[i])
                return vals[i] + frac * (vals[i + 1] - vals[i])
        return vals[-1]

    def get(self, t: float) -> RiderInput:
        _i = self._interp
        return RiderInput(
            throttle     = _i(self._t, self._throttle,    t),
            brake_front  = _i(self._t, self._brake_front, t),
            brake_rear   = _i(self._t, self._brake_rear,  t),
            steer_torque = _i(self._t, self._steer,       t),
            lean_delta   = _i(self._t, self._lean,        t),
        ).clamp()


class SpeedControlSource(RiderInputSource):
    """
    Wraps any base input source and overrides throttle/brake for cruise control.

    When params_fn() returns target_speed > 0, a PI controller maintains that
    speed by adjusting throttle (when below target) or gentle rear brake (above).
    Manual throttle/brake from the base source pass through when target_speed=0.

    PI gains are tuned for a ~189 kg motorcycle with engine_torque_max=80 Nm:
        Kp=0.15 [throttle/(m/s)]  — 1 m/s error → +0.15 throttle
        Ki=0.08 [throttle/(m/s·s)] — integral holds steady-state offset
    """

    _Kp = 0.15
    _Ki = 0.08

    def __init__(
        self,
        base_source: RiderInputSource,
        vx_fn:      Callable[[], float],
        params_fn:  Callable[[], dict],
        dt:         float = 0.001,
    ) -> None:
        self._base      = base_source
        self._vx_fn     = vx_fn      # returns current forward speed [m/s]
        self._params_fn = params_fn
        self._dt        = dt
        self._integral  = 0.0

    def get(self, t: float) -> RiderInput:
        inp    = self._base.get(t)
        target = float(self._params_fn().get("target_speed", 0.0))

        if target <= 0.0:
            self._integral = 0.0
            return inp

        err            = target - self._vx_fn()
        self._integral = max(-5.0, min(5.0, self._integral + err * self._dt))
        demand         = self._Kp * err + self._Ki * self._integral

        if demand >= 0.0:
            return RiderInput(
                throttle     = max(0.0, min(1.0, demand)),
                brake_front  = 0.0,
                brake_rear   = 0.0,
                steer_torque = inp.steer_torque,
                lean_delta   = inp.lean_delta,
                steer_target = inp.steer_target,
            )
        else:
            # Gentle braking — limit to 40% demand to avoid lock-up during CC
            brk = max(0.0, min(0.4, -demand))
            return RiderInput(
                throttle     = 0.0,
                brake_front  = brk * 0.4,
                brake_rear   = brk,
                steer_torque = inp.steer_torque,
                lean_delta   = inp.lean_delta,
                steer_target = inp.steer_target,
            )
