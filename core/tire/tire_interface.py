"""
core/tire/tire_interface.py — Stable tire API for the moto_sim product.

This is the ONLY entry point callers (rig, motorcycle, scenarios) should use.
They must never call DummyTirePINN directly.

API contract (frozen for all milestones):

    model = TireModel.from_yaml("pacejka_coeffs/medium.yaml")
    model.reset()
    Fx, Fy, Mz = model.compute_forces(alpha, kappa, Fz, gamma, vx, dt)
    alpha_prime = model.alpha_prime          # current relaxed slip angle

compute_forces() is:
- Deterministic for identical inputs
- Stateful only through alpha_prime (first-order relaxation)
- O(1) per call — no heap allocation
- Thread-safe IF each wheel owns a separate TireModel instance

Future state additions (temperature, wear, pressure — Milestone 5) will be
added to TireState without changing the compute_forces() signature.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

from .pinn_model import DummyTirePINN, TireCoeffs


# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TireState:
    """
    All per-tire state variables that persist between timesteps.

    Milestone 0: alpha_prime (relaxation).
    Milestone 5: T_tread, T_carcass, wear, pressure — active thermal/wear model.

    Defaults represent a pre-heated race tire at optimal temperature so that
    existing simulation tests run at full grip without requiring state override.
    Cold-tyre scenarios require explicit TireState(T_tread=20.0, T_carcass=20.0).
    """
    alpha_prime: float = 0.0    # relaxed slip angle [rad]
    # ── M5 thermal / wear fields ─────────────────────────────────────────────
    T_tread:    float = 85.0    # tread temperature  [°C]  (optimal = full grip)
    T_carcass:  float = 80.0    # carcass temperature [°C] (slightly below tread)
    wear:       float = 0.0     # wear level [0=new, 1=bald]
    pressure:   float = 240.0   # inflation pressure [kPa]  (p_optimal = nominal)


# ─────────────────────────────────────────────────────────────────────────────
class TireModel:
    """
    Stateful tire model: Pacejka MF + ZOH relaxation + (M5) thermal/wear.

    One instance per wheel.  Not thread-safe across instances.

    Parameters
    ----------
    coeffs : TireCoeffs
        Pacejka + combined-slip + geometry coefficients.

    Example
    -------
    >>> model = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
    >>> model.reset()
    >>> Fx, Fy, Mz = model.compute_forces(
    ...     alpha=-0.05, kappa=0.02, Fz=1470.0, gamma=0.0, vx=10.0, dt=0.001
    ... )
    """

    def __init__(self, coeffs: TireCoeffs) -> None:
        self._pinn   = DummyTirePINN.from_coeffs(coeffs)
        self._sigma  = coeffs.sigma
        self._coeffs = coeffs
        self._state  = TireState()

    # ── Factory ───────────────────────────────────────────────────────────────
    @classmethod
    def from_yaml(cls, path: str | Path) -> "TireModel":
        """Load coefficients from a YAML compound file."""
        return cls(TireCoeffs.from_yaml(path))

    @classmethod
    def from_coeffs(cls, coeffs: TireCoeffs) -> "TireModel":
        return cls(coeffs)

    # ── State management ──────────────────────────────────────────────────────
    def reset(self, state: TireState | None = None) -> None:
        """Reset tire state (call before each new simulation episode)."""
        self._state = state if state is not None else TireState()

    @property
    def alpha_prime(self) -> float:
        """Current relaxed slip angle [rad]."""
        return self._state.alpha_prime

    @property
    def state(self) -> TireState:
        """Full tire state snapshot (read-only copy)."""
        import copy
        return copy.copy(self._state)

    # ── Core API ──────────────────────────────────────────────────────────────
    def compute_forces(
        self,
        alpha: float,    # kinematic slip angle  [rad]   — atan2(vy, vx) in tire frame
        kappa: float,    # longitudinal slip      [–]    — (R·ω − vx) / (|vx| + ε)
        Fz:    float,    # normal load            [N]    — positive = compression
        gamma: float,    # camber angle           [rad]  — lean from vertical
        vx:    float,    # longitudinal speed     [m/s]  — at contact patch
        dt:    float,    # timestep               [s]
    ) -> tuple[float, float, float]:
        """
        Advance relaxation ODE and return combined-slip tire forces.

        Returns
        -------
        (Fx, Fy, Mz)
            Fx  [N]    longitudinal force (traction positive)
            Fy  [N]    lateral force
            Mz  [N·m]  self-aligning torque (stabilising negative)

        Notes
        -----
        Relaxation: ZOH (A-stable, exact for first-order linear ODE):
            α'[k+1] = α + (α'[k]−α)·exp(−|vx|·dt/σ)
        Combined slip: Pacejka Gxα/Gyκ weighting (C∞ smooth).
        Mz uses combined Fy — physically consistent at all κ.
        """
        # ── ZOH relaxation ────────────────────────────────────────────────────
        vx_safe = max(abs(vx), 1e-3)
        decay   = math.exp(-vx_safe * dt / self._sigma)
        self._state.alpha_prime = alpha + (self._state.alpha_prime - alpha) * decay

        # ── Combined forces (nominal mu, full grip) ───────────────────────────
        Fx, Fy, Mz = self._pinn.predict_combined(
            Fz=Fz,
            slip_angle=self._state.alpha_prime,
            kappa=kappa,
            camber=gamma,
        )

        # ── M5: Thermal / wear state update ───────────────────────────────────
        c = self._coeffs

        # Contact-patch slip power [W]: longitudinal + lateral dissipation
        P_slip = (abs(Fx * vx_safe * abs(kappa))
                  + abs(Fy * vx_safe * abs(self._state.alpha_prime)))

        # Tread temperature (lumped first-order ODE, Euler)
        self._state.T_tread += (
            c.heat_scale * P_slip
            - (self._state.T_tread - c.T_ambient) / c.tau_tread
        ) * dt

        # Carcass follows tread with longer thermal lag
        self._state.T_carcass += (
            (self._state.T_tread - self._state.T_carcass) / c.tau_carcass
        ) * dt

        # Tyre pressure via ideal-gas law from carcass temperature
        self._state.pressure = (
            c.p_cold * (273.15 + self._state.T_carcass) / (273.15 + c.T_ambient)
        )

        # Wear accumulation (power-law, capped at 1 = bald)
        self._state.wear = min(
            1.0,
            self._state.wear + c.k_wear * (P_slip ** c.wear_exp) * dt,
        )

        # ── M5: Grip scaling ──────────────────────────────────────────────────
        # Temperature: Gaussian bell centred at T_optimal. Clamp the exponent so a
        # transient (e.g. a hard validation impact) can't overflow exp() / crash.
        _z = (self._state.T_tread - c.T_optimal) / c.T_width
        _z = max(-30.0, min(30.0, _z))
        g_T = c.T_cold_grip + (1.0 - c.T_cold_grip) * math.exp(-_z * _z)
        # Wear: linear grip reduction
        g_W = 1.0 - c.grip_wear_max * self._state.wear
        mu_scale = g_T * g_W

        return Fx * mu_scale, Fy * mu_scale, Mz * mu_scale

    # ── Convenience: pure-slip query (no state advance) ───────────────────────
    def query_pure(
        self, Fz: float, alpha: float, camber: float = 0.0,
    ) -> tuple[float, float]:
        """
        Pure-slip (Fy, Mz) at given α.  Does NOT advance state.
        Useful for tire characterisation plots and validation.
        """
        return self._pinn.predict(Fz=Fz, slip_angle=alpha, camber=camber)

    # ── Analytical properties ─────────────────────────────────────────────────
    def peak_slip_angle_deg(self, Fz: float = 1470.0) -> float:
        return self._pinn.peak_slip_angle_deg(Fz)

    def cornering_stiffness(self, Fz: float = 1470.0) -> float:
        """dFy/dα at α=0 [N/rad]."""
        return self._pinn.cornering_stiffness(Fz)

    def trail_reversal_deg(self) -> float:
        return self._pinn.trail_reversal_deg()
