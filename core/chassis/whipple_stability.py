"""
core/chassis/whipple_stability.py — Sharp/Whipple linearized stability supplement.

Physical basis
──────────────
This class supplies a rider-balance model: a roll restoring torque (rider lean
plus a quasi-static projection of the centripetal path) and a steer-centering
torque.  PyChrono handles gravity, gyroscopic wheel precession, suspension and
tyre contact forces on its own.

CORRECTION (2026-08-08).  Earlier versions of this docstring claimed PyChrono's
constraint chain (wheel → front_axle revolute → fork_lower → prismatic + TSDA →
fork_upper → steer motor → frame) fails to propagate the tyre aligning moment to
the steer axis and the lateral tyre force to main-frame roll.  Both claims were
tested by injecting a known wrench at the front contact patch and comparing the
response against the same wrench applied directly to the target DOF:

    aligning moment  → steer axis     : |η| = 0.96   (routed)
    lateral force    → main-frame roll:  η  = 0.69   (routed; the deficit is
                                                     genuine steer-DOF
                                                     compliance via the trail)

Neither path is missing.  The behaviour that motivated the "compensation" was a
sign inversion in the front-tyre slip kinematics, now fixed — see
Motorcycle.steer_kin_sign and validation/probes/STEER_SIGN_AUDIT.md.

Note also that tau_roll below contains no steer-angle term: the centripetal
coupling is folded onto the DIAGONAL of the roll equation by the quasi-static
elimination δ_eq = (K_lean/K_ctr_phys)·φ.  This is a 1-DOF roll oscillator, not
a 2-DOF Whipple weave mode, and should not be described as restoring Meijaard's
off-diagonal K₂ term.

Reference: Sharp (1971), Meijaard et al. (2007), Cossalter Ch.4.
"""

from __future__ import annotations
import math


class WhippleStabilityModel:
    """
    Compute physically-derived supplemental roll and steer torques.

    All parameters are read from a BikeParams instance.  The model is
    stateless — call torques() each step with the current state.
    """

    def __init__(self, params, K_Mz_missing: float = 0.0,
                 centripetal_factor: float = 2.5,
                 C_roll: float = 200.0) -> None:
        p = self.p = params
        # Exposed as attributes so the probes in validation/probes can sweep and
        # ablate them without editing this file.  K_Mz_missing defaulted to
        # 2000.0 before the steer-sign fix; see the note in torques() below.
        self.K_Mz_missing = K_Mz_missing
        self.centripetal_factor = centripetal_factor
        self.C_roll = C_roll
        # Effective roll inertia used for C_roll scaling:
        #   Ixx (frame) + m_frame * h² (parallel-axis, frame mass at CG height)
        self._I_roll_eff = p.frame_Ixx + p.main_frame_mass * p.cg_height ** 2

    # ─────────────────────────────────────────────────────────────────────────
    def torques(
        self,
        vx:          float,   # [m/s]  forward speed
        roll:        float,   # [rad]  chassis roll angle
        steer_angle: float,   # [rad]  front wheel steer angle (motor position)
        roll_rate:   float,   # [rad/s] roll angular velocity (body-frame X)
        Fz_f:        float,   # [N]   front wheel normal load
        lean_delta:  float,   # [rad]  rider upper-body lean command
        steer_target: float = 0.0,  # [rad] commanded steer angle for a steady turn
    ) -> tuple[float, float]:
        """
        Returns (tau_steer_supplement, tau_roll_supplement) [N·m].

        tau_steer_supplement: added to the steer motor torque command.
        tau_roll_supplement:  accumulated on main_frame about its body-X axis.

        The caller is still responsible for:
          - Rider steer torque input (subtracted from tau_steer)
          - C_steer viscous steer damping
          - K_pitch_damp, K_yaw_damp (separate compensation terms)
        """
        p = self.p

        # ── Trail self-alignment: lean → steer (physical) ─────────────────────
        # Lateral tire force from lean creates a steering torque proportional to
        # Fz_f × trail × cos(rake).  Cossalter §4.2, Sharp 1971 Eq.(14).
        K_lean = Fz_f * p.trail * math.cos(p.rake)          # ~82 N·m/rad at cruise

        # ── Gyroscopic steer centering (physical, v²-dependent) ───────────────
        # Front wheel angular momentum If·(v/Rf) precesses about steer axis when
        # yaw rate ψ̇ = v·δ/L; resulting torque:
        #   τ = If · ω_front · ψ̇ = If · v/Rf · v·δ/L = If·v²/(Rf·L) · δ
        # Source: Sharp 1971, gyroscopic matrix term.
        K_gyro_steer = (p.front_wheel_Iyy
                        * vx * vx
                        / (p.front_wheel_radius * p.wheelbase))  # ~33 N·m/rad @6m/s

        # Physical geometric steer centering (trail + gyroscopic).
        # Used ONLY for the centripetal-coupling projection in tau_roll (below).
        K_ctr_phys = K_lean + K_gyro_steer   # ~105 N·m/rad at v=5

        # ── Steer-axis self-alignment ─────────────────────────────────────────
        # HISTORICAL NOTE.  This term used to carry a fixed 2000 N·m/rad
        # "constraint compensation", on the belief that PyChrono's wheel-bearing
        # revolute did not route the Pacejka aligning moment Mz to the steer
        # axis.  That belief was wrong.  Measured transmission ratio for a
        # moment injected at the front contact patch versus the same moment
        # applied directly at the steer motor is |η| = 0.96 — the path is routed,
        # as constraint theory requires (a revolute transmits every moment
        # component except the one about its own axis, and the front axle's free
        # axis is the wheel SPIN axis).
        #
        # The real defect was a sign inversion in the front-tyre slip kinematics
        # (see Motorcycle.steer_kin_sign).  The 2000 N·m/rad spring was simply
        # stiff enough to overpower a tyre force pointing the wrong way.  With
        # the sign corrected the compensation is unnecessary: peak steer after a
        # 5° roll perturbation is 1.43°/1.36°/0.96° at v = 5/6/10 m/s with
        # K_Mz_missing = 0, against 73.7°/66.3°/54.9° before the fix.
        #
        # Default is therefore 0.0.  Non-zero values are retained only so the
        # probes in validation/probes can reproduce the pre-fix behaviour.
        K_Mz_missing = self.K_Mz_missing  # N·m/rad  (0 = no compensation)
        K_ctr_steer  = K_ctr_phys + K_Mz_missing

        # ── Centripetal roll coupling (MAIN MISSING TERM) ─────────────────────
        # When front wheel is steered δ, the tire generates lateral force which
        # acts at the contact patch.  The moment arm to the CG (height h) creates
        # a roll restoring torque:
        #   τ_centripetal ≈ m·v²·h/L · sin(δ)
        # This v²-dependent term is the centripetal path that PyChrono's prismatic
        # fork topology fails to route from front wheel to main_frame.
        # Source: Meijaard et al. 2007, K₂ matrix, off-diagonal term.
        K_centripetal = p.total_mass * vx * vx * p.cg_height / p.wheelbase

        # ── Physical rider lean restoring torque ──────────────────────────────
        # Rider shifts upper-body CoM by rider_lean_arm per radian of roll.
        # τ_rider = -m_rider · g · rider_lean_arm · sin(roll − lean_delta)
        K_rider = p.rider_mass * 9.81 * p.rider_lean_arm       # ~404 N·m/rad

        # ── Effective roll stiffness (equilibrium projection) ────────────────
        # Eliminating δ from the coupled steer-roll equations at steady state,
        # using the GEOMETRIC K_ctr_phys (not K_ctr_steer which includes the Mz
        # compensation that does NOT affect the centripetal-force moment arm):
        #   δ_eq = K_lean / K_ctr_phys · φ  →  K_centripetal_eff = K_centripetal · K_lean / K_ctr_phys
        #
        # The 2.5× factor accounts for:
        #   1. Steer settling lag (τ_steer = K_ctr_phys/C_steer ≈ 0.19 s): the steer
        #      never fully reaches δ_eq during a roll oscillation period (~1.3 s).
        #   2. Dynamic centripetal overshoot during steer build-up phase.
        #
        # Values at selected speeds (total_mass=189, cg_height=0.58, L=1.37):
        #   v=5:  K_roll_total ≈ 4312 N·m/rad   (K_net = 3237, ω_weave ≈ 7.1 rad/s)
        #   v=10: K_roll_total ≈ 9837 N·m/rad   (K_net = 8762, ω_weave ≈ 11.6 rad/s)
        # ω_weave ∝ v  →  correct Sharp 1971 speed scaling.
        K_centripetal_eff = (self.centripetal_factor * K_centripetal
                             * K_lean / max(K_ctr_phys, 1.0))
        K_roll_total = K_rider + K_centripetal_eff

        # Constant roll damping — NOT velocity-scaled.
        # Velocity-scaled C_roll (ζ=const) over-damps the v=10 oscillation because
        # PyChrono's gyroscopic effects (SetUseGyroTorque) also scale with speed.
        # Using a fixed 200 N·m·s/rad gives ζ≈0.24 at v=5 and ζ≈0.16 at v=10,
        # preserving underdamped oscillation at both speeds for G5/G6.
        C_roll = self.C_roll   # N·m·s/rad  (constant)

        # ── Steer torque supplement ───────────────────────────────────────────
        # K_ctr_steer (not K_ctr_phys) used here: includes the missing Mz routing
        # so the steer is strongly centered. Originally it centered on ZERO (→ the
        # bike resisted turning, yaw < 3 °/s, G4). It now centers on `steer_target`:
        # commanding a steady steer angle makes the bike HOLD that steer → a real
        # steady turn, while keeping the same stiff centering/stability. With
        # steer_target = 0 this is identical to the original (straight-line preserved).
        # C_steer and rider torque are handled in motorcycle.py.
        tau_steer = (K_lean * math.sin(roll - lean_delta)
                     - K_ctr_steer * (steer_angle - steer_target))

        # ── Roll torque supplement ────────────────────────────────────────────
        # τ_roll = −K_roll_total·sin(φ−lean_delta) − C_roll·φ̇
        # K_roll_total encodes both rider lean and the centripetal path projection.
        tau_roll = (-K_roll_total * math.sin(roll - lean_delta)
                    - C_roll * roll_rate)

        return tau_steer, tau_roll
