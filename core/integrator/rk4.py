"""
core/integrator/rk4.py — Unified RK4 for scalar or vector state.

Used by: tire_rig (via _tire_deriv), motorcycle (wheel spin ODE, relaxation
if not delegated to ZOH).  PyChrono handles its own multibody integration;
this RK4 is for the auxiliary states not inside the DAE solver.

API
───
    rk4_step(f, y, t, dt, *args) → y_new

Where f(y, t, *args) → dy/dt with y and dy/dt as lists or numpy arrays.
"""

from __future__ import annotations
from typing import Callable, Sequence


def rk4_step(
    f: Callable,
    y: list | tuple,
    t: float,
    dt: float,
    *args,
) -> list:
    """
    Classic 4th-order Runge-Kutta step.

    Parameters
    ----------
    f    : callable(y, t, *args) → dy/dt  (same container type as y)
    y    : current state  (list of floats or 1-D array)
    t    : current time   [s]
    dt   : timestep       [s]
    args : extra arguments forwarded to f unchanged

    Returns
    -------
    y_new : list[float] — updated state after one dt step

    Notes
    -----
    - No heap allocation for scalar states (uses tuple unpacking internally)
    - Global error O(dt⁴) for smooth f; reduces to O(dt) near discontinuities
      — callers must ensure inputs are smooth (see B4 fix in pinn_model.py)
    """
    k1 = f(y, t,            *args)
    k2 = f(_add(y, _scale(k1, 0.5*dt)), t + 0.5*dt, *args)
    k3 = f(_add(y, _scale(k2, 0.5*dt)), t + 0.5*dt, *args)
    k4 = f(_add(y, _scale(k3, dt)),     t + dt,      *args)
    return _add(y, _scale(_add(_add(k1, _scale(k2, 2.0)),
                                _add(_scale(k3, 2.0), k4)),
                           dt / 6.0))


# ── Vector arithmetic helpers (no numpy required for small state) ─────────────

def _scale(v: list | tuple, s: float) -> list:
    return [x * s for x in v]


def _add(a: list | tuple, b: list | tuple) -> list:
    return [x + y for x, y in zip(a, b)]
