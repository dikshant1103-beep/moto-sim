"""
core/chassis/motorcycle.py — PyChrono multibody motorcycle model.

Milestone 1 implementation.

Architecture
────────────
Bodies   : main_frame, swingarm, front_fork, front_wheel, rear_wheel
Joints   : steering_axis (cylindrical — steer + fork compression),
           swingarm_pivot (revolute),
           front_axle (revolute), rear_axle (revolute + drive/brake torque),
           front_suspension (ChLinkTSDA),
           rear_suspension (ChLinkTSDA)
Tire API : two TireModel instances (front, rear) from tire_interface
Road API : one FlatPlane (or any road with .query(x,y) → RoadSample)
Rider    : RiderInputSource queried each step

Coordinate frame (PyChrono / SAE J670)
    X : forward   Y : left   Z : up   (right-hand)

See docs/decisions/004-pychrono-integration.md and
    docs/decisions/005-m1-pychrono-architecture.md for rationale.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import pychrono as chrono
import yaml

from .whipple_stability import WhippleStabilityModel


# ─────────────────────────────────────────────────────────────────────────────
class BikeParams:
    """
    Motorcycle geometry and inertia parameters loaded from YAML.

    All lengths in metres, masses in kg, inertias in kg·m², angles in radians.
    """

    _REQUIRED = {
        "wheelbase":          "m",
        "rake":               "rad",
        "trail":              "m",
        "front_wheel_radius": "m",
        "rear_wheel_radius":  "m",
        "total_mass":         "kg",
        "cg_height":          "m",
        "cg_x":               "m",
        "front_spring_k":     "N/m",
        "front_damper_c":     "N·s/m",
        "rear_spring_k":      "N/m",
        "rear_damper_c":      "N·s/m",
    }

    def __init__(self, d: dict) -> None:
        for key in self._REQUIRED:
            if key not in d:
                raise ValueError(f"BikeParams: missing required key '{key}'")
            setattr(self, key, d[key])

        self.rider_mass      = d.get("rider_mass",       75.0)   # [kg]
        self.rider_lean_arm  = d.get("rider_lean_arm",   0.55)   # [m] effective lean arm
        self.steering_friction  = d.get("steering_friction",   0.5)
        self.front_wheel_mass   = d.get("front_wheel_mass",    4.5)
        self.rear_wheel_mass    = d.get("rear_wheel_mass",     7.0)
        self.front_wheel_Iyy   = d.get("front_wheel_Iyy",    0.4)
        self.rear_wheel_Iyy    = d.get("rear_wheel_Iyy",     0.6)
        self.main_frame_mass    = d.get("main_frame_mass",   165.0)
        self.swingarm_mass      = d.get("swingarm_mass",       5.0)
        self.front_fork_mass    = d.get("front_fork_mass",     7.5)
        self.frame_Ixx          = d.get("frame_Ixx",           9.5)
        self.frame_Iyy          = d.get("frame_Iyy",          58.0)
        self.frame_Izz          = d.get("frame_Izz",          50.0)
        self.frame_Ixz          = d.get("frame_Ixz",          -2.5)
        self.swingarm_Iyy       = d.get("swingarm_Iyy",        0.25)
        self.swingarm_length    = d.get("swingarm_length",     0.540)
        self.swingarm_pivot_x   = d.get("swingarm_pivot_x",   0.420)
        self.swingarm_pivot_z   = d.get("swingarm_pivot_z",   0.330)
        self.front_ride_height  = d.get("front_ride_height",  0.115)
        self.rear_ride_height   = d.get("rear_ride_height",   0.100)
        self.rear_spring_preload = d.get("rear_spring_preload", 0.005)
        # Engine/drive
        self.engine_torque_max  = d.get("engine_torque_max",  80.0)   # N·m at rear wheel
        self.brake_torque_max   = d.get("brake_torque_max",  150.0)   # N·m (combined)
        self._raw = d

    @classmethod
    def from_yaml(cls, path: str | Path) -> "BikeParams":
        with open(path) as f:
            return cls(yaml.safe_load(f))


# ─────────────────────────────────────────────────────────────────────────────
# Small helpers
# ─────────────────────────────────────────────────────────────────────────────

def _quat_from_axis_angle(axis: chrono.ChVector3d, angle: float) -> chrono.ChQuaterniond:
    """Rotation quaternion from axis+angle."""
    ax = axis.GetNormalized()
    s = math.sin(angle / 2)
    return chrono.ChQuaterniond(math.cos(angle / 2), ax.x * s, ax.y * s, ax.z * s)


def _revolute_frame_y(pos: chrono.ChVector3d) -> chrono.ChFramed:
    """ChFramed at pos whose Z axis aligns with world +Y (for Y-spin revolutes)."""
    q = chrono.QuatFromAngleX(-math.pi / 2)
    return chrono.ChFramed(pos, q)


def _steering_frame(pos: chrono.ChVector3d, rake: float) -> chrono.ChFramed:
    """
    ChFramed at pos whose Z axis aligns with the steering axis direction.

    The steering axis leans rearward from vertical by rake rad.
    Direction (upward along axis): (-sin(rake), 0, cos(rake)).
    Revolute joint Z → steering axis direction.

    Build by rotating world Z by rake about Y:
        Z_new = cos(rake)*Z - sin(rake)*X  → matches (-sin(rake),0,cos(rake))
        but we need to rotate Z toward -X, which is a rotation about +Y by -rake.
    """
    q = chrono.QuatFromAngleY(-rake)
    return chrono.ChFramed(pos, q)


def _make_simple_body(
    mass: float,
    pos: chrono.ChVector3d,
    Ixx: float = 1.0, Iyy: float = 1.0, Izz: float = 1.0,
) -> chrono.ChBody:
    b = chrono.ChBody()
    b.SetMass(mass)
    b.SetPos(pos)
    b.SetInertiaXX(chrono.ChVector3d(Ixx, Iyy, Izz))
    b.SetFixed(False)
    return b


# ─────────────────────────────────────────────────────────────────────────────
class Motorcycle:
    """
    PyChrono multibody motorcycle — Milestone 1.

    Usage
    ─────
        moto = Motorcycle.from_yaml(bike_yaml, front_tire_yaml, rear_tire_yaml,
                                    road, rider_source)
        moto.build()
        moto.reset()
        for k in range(n_steps):
            telemetry = moto.step(k * moto.dt)
    """

    # Crown radius of tyre toroid — camber-dependent lateral contact patch offset
    R_CROWN = 0.040   # [m]

    def __init__(
        self,
        params: BikeParams,
        front_tire,     # TireModel
        rear_tire,      # TireModel
        road,           # has .query(x,y) → RoadSample
        rider_source,   # RiderInputSource
        dt: float = 0.001,
    ) -> None:
        self.params       = params
        self.front_tire   = front_tire
        self.rear_tire    = rear_tire
        self.road         = road
        self.rider_source = rider_source
        self.dt           = dt
        self._built       = False

        # Bodies (populated by build())
        self.system:       Optional[chrono.ChSystemSMC] = None
        self.main_frame:   Optional[chrono.ChBody] = None
        self.swingarm:     Optional[chrono.ChBody] = None
        self.front_fork:   Optional[chrono.ChBody] = None   # upper fork tubes
        self.fork_lower:   Optional[chrono.ChBody] = None   # lower fork legs (M3)
        self.front_wheel:  Optional[chrono.ChBody] = None
        self.rear_wheel:   Optional[chrono.ChBody] = None

        # Joints
        self._steer_motor:  Optional[chrono.ChLinkMotorRotationTorque] = None
        # M3 suspension links (populated by build())
        self._front_susp:             Optional[chrono.ChLinkTSDA] = None
        self._rear_susp:              Optional[chrono.ChLinkTSDA] = None
        self._front_susp_rest_length: float = 0.0
        self._rear_susp_rest_length:  float = 0.0

        # Saved initial body transforms for reset()
        self._init_pos: dict = {}
        self._init_rot: dict = {}

        # OPT-IN bumpy validation ground: when set to a road object (has .query(x,y)
        # → RoadSample with .height), build() replaces the flat ground box with a
        # collision TRIANGLE MESH that follows the road profile, so the wheels
        # physically ride bumps / potholes / cross-slope (real suspension + load
        # response). Default None → flat ground (unchanged straight-line behaviour).
        self.profile_ground = None

        # Pre-allocated scratch vectors for hot loop
        self._F_front  = chrono.ChVector3d()
        self._F_rear   = chrono.ChVector3d()
        self._cp_front = chrono.ChVector3d()
        self._cp_rear  = chrono.ChVector3d()

        # Force accumulators (idx into AddAccumulator)
        self._acc_front: int = 0
        self._acc_rear:  int = 0
        self._acc_frame: int = 0

        # Current rider inputs (updated each step)
        self._steer_fn   = chrono.ChFunctionConst(0.0)
        self._drive_fn   = chrono.ChFunctionConst(0.0)
        self._fbrake_fn  = chrono.ChFunctionConst(0.0)

        # Initial velocities saved for reset
        self._v0 = 5.0   # m/s

        # Longitudinal slip relaxation state (prevents kappa instability).
        # Without this the wheel (small I_yy) decelerates faster than the frame
        # (large mass) each step, causing kappa to diverge.
        # ZOH: κ'[k+1] = κ_kin + (κ'[k] - κ_kin)·exp(-vx·dt/σ_kappa)
        self._sigma_kappa: float = 0.30   # [m] longitudinal relaxation length
        self._kappa_front: float = 0.0
        self._kappa_rear:  float = 0.0

        # ── M2: Dynamic Fz (longitudinal weight transfer) ─────────────────────
        self._ax_smooth: float = 0.0
        self._vx_prev:   float = 0.0

        # ── M3: Lateral acceleration EMA for Fz correction during cornering ──
        # ΔFz_lat = m·ay·h·sin(roll)/L  (small but non-zero at large lean angles)
        self._ay_smooth: float = 0.0
        self._vy_prev:   float = 0.0

        # ── M4: Whipple/Sharp parallel stability model ────────────────────────
        # Replaces the empirical compensation constants (K_ctr=800, K_roll_comp=5596,
        # C_roll=1200) with physically-derived velocity-dependent terms computed
        # each step by WhippleStabilityModel.  See whipple_stability.py for derivation.
        self._whipple = WhippleStabilityModel(params)

        # K_yaw_damp: artificial yaw-rate damping, now DISABLED (0.0).
        #
        # This was 2000 N·m·s/rad against a frame yaw inertia of Izz = 50 kg·m²
        # — a 40:1 override of the vehicle's own yaw dynamics.  Its stated
        # justification was that "PyChrono's constraint chain does not route
        # steer reaction to yaw on main_frame", and its value was tuned to leave
        # "enough response for countersteering (G6)".  Both premises were wrong:
        # the constraint chain routes the tyre aligning moment to the steer axis
        # at |η| = 0.96, and the G6 gate was itself inverted (see decision 006).
        # The yaw instability it suppressed was the front-tyre steer-sign
        # inversion acting as positive feedback, fixed by steer_kin_sign below.
        #
        # Measured over 30 s of straight running with steer_kin_sign = -1.0 and
        # no damping at all, peak yaw rate is 0.0011 °/s at 10 m/s and 0.0060 °/s
        # at 20 m/s — thousandths of a degree per second.  The "~14 rad/s yaw
        # instability eigenvalue" does not appear.  What the damper did cost was
        # steady-turn fidelity against v·tan(δ)·cos(ε)/L:
        #
        #     K_yaw_damp    0     50     200     500    2000
        #     turn error  -2.4%  -6.6%  -17.1%  -32.4%  -64.9%
        #
        # Evidence: validation/probes/s7_yaw_damping.py.
        #
        # CAVEAT if you ever re-enable it: the torque below is applied about
        # WORLD Z, unlike K_pitch_damp which correctly uses the body Y axis.  On
        # a leaned bike that also bleeds off part of the lean rate — almost
        # certainly the "couples back into roll" effect the old comment noted.
        # Fix the axis before using a non-zero value.
        self.K_yaw_damp   = 0.0     # N·m·s/rad  (disabled; see decision 006)
        #
        # K_pitch_damp: TSDA provides ≈1200 N·m·s/rad pitch damping; 300 is a small
        # supplement to handle HHT integrator noise at low speed (<3 m/s).
        self.K_pitch_damp = 300.0   # N·m·s/rad  (supplemental)
        #
        self.C_steer      = 20.0    # N·m·s/rad  steer viscous friction
        #
        # steer_kin_sign: sign mapping between ChLinkMotorRotationTorque's
        # GetMotorAngle() and the TRUE geometric steer angle of the front wheel,
        # used when building the front-wheel heading for the tyre slip model
        # (see _psi_front below).
        #
        # GetMotorAngle() returns the NEGATIVE of the geometric steer angle.
        # Measured directly from body orientations: a +10 N·m steer torque held
        # for 0.2 s gives GetMotorAngle() = +30.954° while the wheel physically
        # points −28.317° (= −GetMotorAngle()·cos(rake)).  Feeding the raw motor
        # angle into _psi_front therefore evaluated the front tyre at a slip
        # angle of the wrong sign, so its lateral force AMPLIFIED steer
        # perturbations instead of opposing them: peak steer excursion reached
        # 55–74° after a 5° roll perturbation once the artificial centering
        # spring (K_Mz_missing) was removed, and the vehicle yawed opposite to
        # its own front wheel in a steady turn.
        #
        # With -1.0 the steady-turn yaw rate matches the single-track kinematic
        # law v·tan(δ)·cos(ε)/L to 0.5 %, and peak steer stays under 1.5 % of a
        # degree per degree of roll with no compensation term at all.
        #
        # Set to +1.0 only to reproduce pre-fix behaviour.
        # Audit + reproduction scripts: validation/probes/ (see
        # validation/probes/STEER_SIGN_AUDIT.md).
        self.steer_kin_sign = -1.0
        # Smoothed steer-target the controller chases. Ramping it (instead of
        # stepping) avoids the torque snap that diverges the model, while keeping
        # full steer-centering authority so the steer holds the small intended angle
        # (no runaway → no speed-scrub → no low-speed divergence). See step().
        self._steer_tgt_smooth = 0.0
        self._STEER_TGT_RATE   = 0.25   # rad/s max change in chased steer target

    # ─────────────────────────────────────────────────────────────────────────
    def build(self) -> None:
        """Construct PyChrono bodies, joints, and forces.  Call once before reset()."""
        if self._built:
            return
        p = self.params

        # ── 1. System + Bullet collision + HHT timestepper ──────────────────
        sys = chrono.ChSystemSMC()
        sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
        sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
        hht = chrono.CastToChTimestepperHHT(sys.GetTimestepper())
        hht.SetAlpha(-0.3)
        hht.SetMaxIters(10)
        hht.SetAbsTolerances(1e-4, 1e-3)
        hht.SetStepControl(False)
        hht.SetJacobianUpdateMethod(hht.JacobianUpdate_EVERY_STEP)
        self.system = sys

        # ── 1b. Contact material (zero friction — Pacejka handles it) ────────
        # SMC contact provides the wheel normal force; tangential forces come
        # from AccumulateForce calls with the Pacejka model.
        _cmat = chrono.ChContactMaterialSMC()
        _cmat.SetFriction(0.0)
        _cmat.SetRestitution(0.0)
        # E=5e9: equilibrium penetration ~0.08mm, settles in ~15ms — small enough
        # that the initial free-fall pitch transient is < 0.1° before contact loads.
        _cmat.SetYoungModulus(5e9)
        _cmat.SetPoissonRatio(0.3)
        self._contact_mat = _cmat

        # ── 1c. Ground plane ─────────────────────────────────────────────────
        # Always the proven flat box (stable contact). For validation, raised
        # obstacle boxes are ADDED on top (the wheel rolls on stable flat ground,
        # then physically rides over each obstacle → real suspension + load jolt).
        ground = chrono.ChBodyEasyBox(2000, 200, 0.02, 1000, True, True, _cmat)
        ground.SetFixed(True)
        ground.SetPos(chrono.ChVector3d(0.0, 0.0, -0.01))
        sys.AddBody(ground)
        if self.profile_ground is not None:
            for b in self._build_obstacle_bodies(_cmat):
                sys.AddBody(b)

        # ── 2. Geometry ───────────────────────────────────────────────────────
        Rr   = p.rear_wheel_radius
        Rf   = p.front_wheel_radius
        L    = p.wheelbase
        rake = p.rake
        sin_r = math.sin(rake); cos_r = math.cos(rake)

        # Absolute world positions of key points
        p_rear_axle  = chrono.ChVector3d(0.0, 0.0, Rr)
        p_front_axle = chrono.ChVector3d(L,   0.0, Rf)
        p_frame_cg   = chrono.ChVector3d(p.cg_x, 0.0, p.cg_height)
        p_sa_pivot   = chrono.ChVector3d(p.swingarm_pivot_x, 0.0, p.swingarm_pivot_z)

        # Steering head: position such that the steer axis extended to the
        # ground lands at (wheelbase + trail, 0, 0), giving the correct
        # fork offset (≈33mm).  Without this the front axle falls on the
        # steer axis → zero geometric trail → lean-steer coupling inverts.
        #
        # Condition: psh_x + psh_z * tan(rake) = L + trail
        # Keep z = front_axle_z + fork_length*cos(rake) (same height).
        fork_length = 0.55   # m (governs steer-head height, not x-position)
        _psh_z = p_front_axle.z + fork_length * cos_r
        _psh_x = L + p.trail - _psh_z * (sin_r / cos_r)
        p_steer_head = chrono.ChVector3d(_psh_x, 0.0, _psh_z)

        # Fork split: midpoint between steering head and front axle
        p_fork_mid = chrono.ChVector3d(
            (p_steer_head.x + p_front_axle.x) / 2.0,
            0.0,
            (p_steer_head.z + p_front_axle.z) / 2.0,
        )
        # Kept for any legacy reference (same as p_fork_mid)
        p_front_fork_cg = p_fork_mid
        # Fork-axis unit vector (upward along steering axis)
        fork_ax_x = -sin_r
        fork_ax_z =  cos_r
        p_swingarm_cg = chrono.ChVector3d(
            (p_sa_pivot.x + p_rear_axle.x) / 2.0,
            0.0,
            (p_sa_pivot.z + p_rear_axle.z) / 2.0,
        )

        # ── 3. Bodies ─────────────────────────────────────────────────────────
        # Main frame: engine + rider + tank lumped at measured CG
        frame_body = chrono.ChBody()
        frame_body.SetMass(p.main_frame_mass)
        frame_body.SetPos(p_frame_cg)
        Imat = chrono.ChMatrix33d(0)
        Imat[0, 0] = p.frame_Ixx
        Imat[1, 1] = p.frame_Iyy
        Imat[2, 2] = p.frame_Izz
        Imat[0, 2] = p.frame_Ixz
        Imat[2, 0] = p.frame_Ixz
        frame_body.SetInertia(Imat)
        frame_body.SetUseGyroTorque(True)
        sys.AddBody(frame_body)
        self.main_frame = frame_body

        # Swingarm: pivots about the swingarm pivot, carries rear axle
        sw_r = _make_simple_body(
            p.swingarm_mass, p_swingarm_cg,
            Ixx=0.05, Iyy=p.swingarm_Iyy, Izz=0.05,
        )
        sw_r.SetUseGyroTorque(True)
        sys.AddBody(sw_r)
        self.swingarm = sw_r

        # ── M3: Two-body telescopic fork ──────────────────────────────────────
        # fork_upper (fu): inner tubes fixed to steering head, steers with frame
        # fork_lower (fl): outer legs, slides along fork axis, carries front wheel
        fork_half = fork_length / 2.0
        fu_mass = p.front_fork_mass * 0.40
        fl_mass = p.front_fork_mass * 0.60
        fu_I = fu_mass * fork_half**2 / 12.0
        fl_I = fl_mass * fork_half**2 / 12.0

        p_fu_cg = chrono.ChVector3d(
            (p_steer_head.x + p_fork_mid.x) / 2.0, 0.0,
            (p_steer_head.z + p_fork_mid.z) / 2.0,
        )
        p_fl_cg = chrono.ChVector3d(
            (p_fork_mid.x + p_front_axle.x) / 2.0, 0.0,
            (p_fork_mid.z + p_front_axle.z) / 2.0,
        )

        fu = _make_simple_body(fu_mass, p_fu_cg, Ixx=fu_I, Iyy=fu_I, Izz=0.01)
        fu.SetUseGyroTorque(True)
        fu.SetRot(_steering_frame(p_fu_cg, rake).GetRot())
        sys.AddBody(fu)
        self.front_fork = fu

        fl = _make_simple_body(fl_mass, p_fl_cg, Ixx=fl_I, Iyy=fl_I, Izz=0.01)
        fl.SetUseGyroTorque(True)
        fl.SetRot(_steering_frame(p_fl_cg, rake).GetRot())
        sys.AddBody(fl)
        self.fork_lower = fl

        # Front wheel: sphere collision shape (radius=Rf) for ground contact;
        # mass/inertia overridden to match YAML values.
        fw = chrono.ChBodyEasySphere(Rf, 1000, True, True, _cmat)
        fw.SetMass(p.front_wheel_mass)
        fw.SetInertiaXX(chrono.ChVector3d(
            p.front_wheel_Iyy / 2.0, p.front_wheel_Iyy, p.front_wheel_Iyy / 2.0))
        fw.SetPos(p_front_axle)
        fw.SetUseGyroTorque(True)
        sys.AddBody(fw)
        self.front_wheel = fw

        # Rear wheel: sphere collision shape (radius=Rr).
        rw = chrono.ChBodyEasySphere(Rr, 1000, True, True, _cmat)
        rw.SetMass(p.rear_wheel_mass)
        rw.SetInertiaXX(chrono.ChVector3d(
            p.rear_wheel_Iyy / 2.0, p.rear_wheel_Iyy, p.rear_wheel_Iyy / 2.0))
        rw.SetPos(p_rear_axle)
        rw.SetUseGyroTorque(True)
        sys.AddBody(rw)
        self.rear_wheel = rw

        # ── 4. Joints ─────────────────────────────────────────────────────────

        # Swingarm pivot: revolute about Y (allows swingarm to adjust for lean/pitch).
        # No spring in M1 — the pivot revolute alone keeps the rear wheel on the ground
        # and allows geometric accommodation of frame roll.  M2 adds TSDA spring with
        # a corrected attachment point for proper moment arm.
        sa_pivot_joint = chrono.ChLinkLockRevolute()
        sa_pivot_joint.Initialize(frame_body, sw_r, _revolute_frame_y(p_sa_pivot))
        sys.AddLink(sa_pivot_joint)

        # Rear axle: revolute about Y (rear wheel spin, torque applied externally)
        rear_axle_joint = chrono.ChLinkLockRevolute()
        rear_axle_joint.Initialize(sw_r, rw, _revolute_frame_y(p_rear_axle))
        sys.AddLink(rear_axle_joint)

        # Steering motor: frame → fork_upper (steer DOF about steering axis)
        steer_motor = chrono.ChLinkMotorRotationTorque()
        steer_motor.Initialize(frame_body, fu, _steering_frame(p_steer_head, rake))
        self._steer_fn = chrono.ChFunctionConst(0.0)
        steer_motor.SetTorqueFunction(self._steer_fn)
        sys.AddLink(steer_motor)
        self._steer_motor = steer_motor

        # ── M3: Prismatic joint: fork_upper → fork_lower (fork compression DOF)
        # Z axis of joint frame = fork axis → translation along fork axis only.
        prism_joint = chrono.ChLinkLockPrismatic()
        prism_joint.Initialize(fu, fl, _steering_frame(p_fork_mid, rake))
        sys.AddLink(prism_joint)

        # ── M3: Front suspension TSDA (spring + damper along fork axis) ──────
        # Attachment points straddle p_fork_mid at ±fs_arm along the fork axis.
        fs_arm = 0.04   # [m] half-distance between TSDA attachment points
        p_fs_upper = chrono.ChVector3d(
            p_fork_mid.x + fork_ax_x * fs_arm, 0.0,
            p_fork_mid.z + fork_ax_z * fs_arm,
        )
        p_fs_lower = chrono.ChVector3d(
            p_fork_mid.x - fork_ax_x * fs_arm, 0.0,
            p_fork_mid.z - fork_ax_z * fs_arm,
        )
        fs_design_length = 2.0 * fs_arm   # initial distance at design (compressed) position

        # Total vehicle weight — used for both front and rear suspension preload.
        W_tot = (p.main_frame_mass + p.swingarm_mass + p.front_fork_mass +
                 p.front_wheel_mass + p.rear_wheel_mass) * 9.81

        # Static preload: spring force along fork axis must support the front load
        # minus the weight of bodies below the spring (fork_lower + front wheel).
        fz_f_static = W_tot * (p.cg_x / p.wheelbase)
        f_spring_front = max(0.0, fz_f_static - (p.front_wheel_mass + fl_mass) * 9.81) / cos_r
        delta_front = f_spring_front / p.front_spring_k
        fs_rest_len = fs_design_length + delta_front   # spring compressed at design

        front_susp = chrono.ChLinkTSDA()
        front_susp.Initialize(fu, fl, False, p_fs_upper, p_fs_lower)
        front_susp.SetRestLength(fs_rest_len)
        front_susp.SetSpringCoefficient(p.front_spring_k)
        front_susp.SetDampingCoefficient(p.front_damper_c)
        sys.AddLink(front_susp)
        self._front_susp             = front_susp
        self._front_susp_rest_length = fs_rest_len

        # Front axle: fork_lower → front_wheel (spin DOF)
        front_axle_joint = chrono.ChLinkLockRevolute()
        front_axle_joint.Initialize(fl, fw, _revolute_frame_y(p_front_axle))
        sys.AddLink(front_axle_joint)

        # ── 5. Rear suspension TSDA ──────────────────────────────────────────
        # Swingarm attachment at the midpoint between pivot and rear axle.
        # Frame attachment directly above the swingarm attachment.
        # Spring is pre-compressed at the design position so its CW torque
        # balances the CCW torque that the ground reaction creates about the
        # swingarm pivot (otherwise the free revolute has no restoring torque
        # and the swingarm rotates uncontrolled).
        #
        # Torque balance about swingarm pivot (Y-axis):
        #   τ_spring (CW) = r_axle_x*(Fz_r − m_rw·g) − r_cg_x·m_sw·g
        #   F_spring = τ_spring / arm_x  where arm_x = |x_mid − x_pivot|
        x_mid     = (p_sa_pivot.x + p_rear_axle.x) / 2.0   # 0.210 m
        z_mid     = (p_sa_pivot.z + p_rear_axle.z) / 2.0   # 0.324 m
        rs_dist   = 0.156                                    # spring design length [m]

        p_rs_swing = chrono.ChVector3d(x_mid, 0.0, z_mid)
        p_rs_frame = chrono.ChVector3d(x_mid, 0.0, z_mid + rs_dist)

        fz_r_static  = W_tot * (1.0 - p.cg_x / p.wheelbase)
        arm_x        = abs(p_sa_pivot.x - x_mid)                # 0.210 m
        tau_cw_req   = (abs(p_rear_axle.x - p_sa_pivot.x) * (fz_r_static - p.rear_wheel_mass * 9.81)
                        - arm_x * p.swingarm_mass * 9.81)
        delta_compr  = max(0.0, tau_cw_req) / (arm_x * p.rear_spring_k)
        rs_rest_len  = rs_dist + delta_compr    # > rs_dist → compressed at design

        rear_susp = chrono.ChLinkTSDA()
        rear_susp.Initialize(frame_body, sw_r, False, p_rs_frame, p_rs_swing)
        rear_susp.SetRestLength(rs_rest_len)
        rear_susp.SetSpringCoefficient(p.rear_spring_k)
        rear_susp.SetDampingCoefficient(p.rear_damper_c)
        sys.AddLink(rear_susp)
        self._rear_susp             = rear_susp
        self._rear_susp_rest_length = rs_rest_len

        # ── 6. Force accumulators for tire forces + frame roll control ───────────
        self._acc_front = fw.AddAccumulator()
        self._acc_rear  = rw.AddAccumulator()
        self._acc_frame = frame_body.AddAccumulator()

        # ── 7. Save initial transforms for reset() ────────────────────────────
        for name, body in [
            ("main_frame", frame_body), ("swingarm", sw_r),
            ("front_fork", fu), ("fork_lower", fl),
            ("front_wheel", fw), ("rear_wheel", rw),
        ]:
            self._init_pos[name] = chrono.ChVector3d(body.GetPos())
            self._init_rot[name] = chrono.ChQuaterniond(body.GetRot())

        self._built = True

    # ─────────────────────────────────────────────────────────────────────────
    def _build_obstacle_bodies(self, cmat):
        """Discretize the validation road's RAISED features into fixed collision
        boxes the wheels ride over. Samples z = profile_ground.query(x, 0).height at
        2 cm along the path; each contiguous raised run becomes a box of that height
        (top at z), 2.5 m wide. (Depressions can't be boxes — pothole far-edge impact
        is captured by its rising edge; cross-slope is handled via the road normal.)"""
        road = self.profile_ground
        V = chrono.ChVector3d
        x0, x1, seg = -2.0, 200.0, 0.04   # 4 cm staircase steps follow the profile shape
        n = int((x1 - x0) / seg)
        bodies = []
        for i in range(n):
            cx = x0 + i * seg + seg / 2
            try:
                h = road.query(cx, 0.0).height
            except Exception:
                h = 0.0
            if h > 0.002:
                # one short box per segment at the local height → a STAIRCASE that
                # follows the shape (gradual for a hump, sharp for a cleat). Slight
                # length overlap so adjacent steps touch.
                box = chrono.ChBodyEasyBox(seg * 1.2, 2.5, h, 1000, True, True, cmat)
                box.SetFixed(True)
                box.SetPos(V(cx, 0.0, h / 2))   # top at z=h
                bodies.append(box)
        return bodies

    # ─────────────────────────────────────────────────────────────────────────
    def reset(self, v0: float = 5.0) -> None:
        """Reset to straight-line initial condition at v0 m/s."""
        if not self._built:
            raise RuntimeError("Call build() before reset().")
        self._v0 = v0
        self._steer_tgt_smooth = 0.0   # straighten the chased steer target on reset
        p = self.params

        bodies = {
            "main_frame":  self.main_frame,
            "swingarm":    self.swingarm,
            "front_fork":  self.front_fork,
            "fork_lower":  self.fork_lower,
            "front_wheel": self.front_wheel,
            "rear_wheel":  self.rear_wheel,
        }

        # Restore initial positions and rotations
        for name, body in bodies.items():
            body.SetPos(self._init_pos[name])
            body.SetRot(self._init_rot[name])
            body.SetPosDt(chrono.ChVector3d(0, 0, 0))
            body.SetAngVelParent(chrono.ChVector3d(0, 0, 0))
            body.SetPosDt2(chrono.ChVector3d(0, 0, 0))
            body.SetAngAccParent(chrono.ChVector3d(0, 0, 0))

        # Zero steer torque before applying velocity
        self._steer_fn.SetConstant(0.0)

        # Apply initial forward velocity to all bodies
        vel_fwd = chrono.ChVector3d(v0, 0.0, 0.0)
        for body in bodies.values():
            body.SetPosDt(vel_fwd)

        # Wheel spin: rolling without slip.
        # Forward rolling about +Y: contact patch velocity = v_cm + ω×r_contact,
        # r_contact = (0,0,-R) → ω×r = (-R*ω_y, 0, 0).
        # No-slip: v_cm.x - R*ω_y = 0 → ω_y = +v0/R > 0.
        omega_front = +v0 / p.front_wheel_radius
        omega_rear  = +v0 / p.rear_wheel_radius
        self.front_wheel.SetAngVelParent(chrono.ChVector3d(0.0, omega_front, 0.0))
        self.rear_wheel.SetAngVelParent(chrono.ChVector3d(0.0, omega_rear,  0.0))

        # Reset system time
        self.system.SetChTime(0.0)

        # Reset tire relaxation state
        self.front_tire.reset()
        self.rear_tire.reset()

        # Reset longitudinal slip relaxation
        self._kappa_front = 0.0
        self._kappa_rear  = 0.0

        # Reset M2/M3 dynamic-Fz state
        self._ax_smooth = 0.0
        self._vx_prev   = v0
        self._ay_smooth = 0.0
        self._vy_prev   = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    def step(self, t: float) -> dict:
        """
        Advance physics by one dt.

        Returns telemetry dict:
            chassis: {roll, pitch, yaw, roll_rate, yaw_rate, vx, vy, ax, ay}
            front:   {alpha, kappa, Fz, Fx, Fy, Mz, alpha_prime}
            rear:    {alpha, kappa, Fz, Fx, Fy, Mz, alpha_prime}
            inputs:  {throttle, brake_front, brake_rear, steer_torque}
        """
        p = self.params

        # ── 1. Rider inputs ───────────────────────────────────────────────────
        inp = self.rider_source.get(t).clamp()

        # Ramp the chased steer target toward the commanded one (no step → no snap).
        _dmax = self._STEER_TGT_RATE * self.dt
        self._steer_tgt_smooth += max(-_dmax, min(_dmax,
                                                  inp.steer_target - self._steer_tgt_smooth))

        # Axle torques (computed again in step 6a with kappa logic; kept here
        # as intermediate values for drive/brake application in step 7)
        drive_torque = inp.throttle * p.engine_torque_max
        brake_front  = inp.brake_front * p.brake_torque_max * 0.4
        brake_rear   = inp.brake_rear  * p.brake_torque_max * 0.6

        # ── 2. Read wheel state ───────────────────────────────────────────────
        pos_f = self.front_wheel.GetPos()
        rot_f = self.front_wheel.GetRot()
        vel_f = self.front_wheel.GetPosDt()
        omega_f = self.front_wheel.GetAngVelParent()

        pos_r = self.rear_wheel.GetPos()
        rot_r = self.rear_wheel.GetRot()
        vel_r = self.rear_wheel.GetPosDt()
        omega_r = self.rear_wheel.GetAngVelParent()

        frame_rot = self.main_frame.GetRot()
        frame_vel = self.main_frame.GetPosDt()

        # ── 3. Chassis angles (from main frame quaternion) ────────────────────
        q = frame_rot
        e0, e1, e2, e3 = q.e0, q.e1, q.e2, q.e3
        roll  = math.atan2(2*(e0*e1 + e2*e3), 1 - 2*(e1*e1 + e2*e2))
        pitch = math.asin(max(-1.0, min(1.0, 2*(e0*e2 - e3*e1))))
        yaw   = math.atan2(2*(e0*e3 + e1*e2), 1 - 2*(e2*e2 + e3*e3))

        frame_omega = self.main_frame.GetAngVelParent()
        cos_yaw = math.cos(yaw); sin_yaw = math.sin(yaw)

        # ── 3b. M2/M3: EMA-filtered longitudinal + lateral acceleration ────────
        vx_body = frame_vel.x * cos_yaw + frame_vel.y * sin_yaw
        vy_body = -frame_vel.x * sin_yaw + frame_vel.y * cos_yaw
        _alpha_ema = self.dt / (0.1 + self.dt)   # T=0.1 s EMA for both axes
        self._ax_smooth = ((1.0 - _alpha_ema) * self._ax_smooth
                           + _alpha_ema * (vx_body - self._vx_prev) / self.dt)
        self._ay_smooth = ((1.0 - _alpha_ema) * self._ay_smooth
                           + _alpha_ema * (vy_body - self._vy_prev) / self.dt)
        self._vx_prev = vx_body
        self._vy_prev = vy_body

        # ── 4. Normal loads for tire model ────────────────────────────────────
        Fz_f, Fz_r = self._static_Fz(roll)

        # ── 4b/4c. M4: Whipple/Sharp physical stability torques ─────────────────
        #
        # WhippleStabilityModel replaces the empirical K_ctr=800/K_roll_comp=5596/
        # C_roll=1200 constants with physically-derived velocity-dependent terms:
        #   tau_steer = K_lean(Fz_f)·sin(φ) − K_ctr_phys(v,Fz_f)·δ
        #   tau_roll  = −K_rider·sin(φ) − K_centripetal(v)·sin(δ) − C_roll(v)·φ̇
        # where K_centripetal = m·v²·h/L is the centripetal path previously missing
        # from PyChrono's constraint chain.  See whipple_stability.py for derivation.
        _steer_ang_raw = self._steer_motor.GetMotorAngle()
        _steer_rate    = self._steer_motor.GetMotorAngleDt()

        _body_x_world = frame_rot.Rotate(chrono.ChVector3d(1.0, 0.0, 0.0))
        _roll_rate = (
            frame_omega.x * _body_x_world.x +
            frame_omega.y * _body_x_world.y +
            frame_omega.z * _body_x_world.z
        )

        _tau_steer_w, _tau_roll_w = self._whipple.torques(
            vx          = vx_body,
            roll        = roll,
            steer_angle = _steer_ang_raw,
            roll_rate   = _roll_rate,
            Fz_f        = Fz_f,
            lean_delta  = inp.lean_delta,
            steer_target = self._steer_tgt_smooth,
        )

        # Steer motor: Whipple supplement + rider torque input + viscous friction.
        # Sign convention: positive motor torque steers LEFT in PyChrono
        # (QuatFromAngleY(-rake) frame), so rider +steer_torque → steer RIGHT
        # needs negation.
        _tau = -inp.steer_torque + _tau_steer_w - self.C_steer * _steer_rate
        # High safety clamp only (prevent extreme spikes); the snap is avoided by
        # rate-limiting the steer_target the model chases (see _steer_tgt_smooth),
        # NOT by limiting torque — a low torque clamp lets the steer run away under
        # cornering constraint forces, which scrubs speed → low-speed divergence.
        _tau = max(-400.0, min(400.0, _tau))
        self._steer_fn.SetConstant(_tau)

        # Roll: Whipple supplement applied about body-frame X axis.
        self.main_frame.AccumulateTorque(
            self._acc_frame,
            chrono.ChVector3d(
                _tau_roll_w * _body_x_world.x,
                _tau_roll_w * _body_x_world.y,
                _tau_roll_w * _body_x_world.z,
            ),
            False,  # world frame
        )

        # Artificial yaw-rate damping — disabled by default (K_yaw_damp = 0.0).
        # See the K_yaw_damp comment in __init__ for why, and for the world-Z
        # axis caveat that must be fixed before re-enabling it.
        if self.K_yaw_damp:
            self.main_frame.AccumulateTorque(
                self._acc_frame,
                chrono.ChVector3d(0.0, 0.0, -self.K_yaw_damp * frame_omega.z),
                False,  # world frame
            )

        # Pitch rate damping: the model has no pitch stabiliser; at low speed
        # (<3 m/s) gyroscopic effects vanish and HHT numerical noise accumulates
        # into a slow nose-dive over tens of seconds. Damping the body-frame
        # pitch rate (Y axis) prevents divergence without fighting the suspension
        # springs that provide the physical pitch equilibrium at normal speed.
        # Must act about the body Y axis (not world Y) so it stays correct when
        # the bike yaws.
        _body_y_world = frame_rot.Rotate(chrono.ChVector3d(0.0, 1.0, 0.0))
        _pitch_rate = (
            frame_omega.x * _body_y_world.x +
            frame_omega.y * _body_y_world.y +
            frame_omega.z * _body_y_world.z
        )
        self.main_frame.AccumulateTorque(
            self._acc_frame,
            chrono.ChVector3d(
                -self.K_pitch_damp * _pitch_rate * _body_y_world.x,
                -self.K_pitch_damp * _pitch_rate * _body_y_world.y,
                -self.K_pitch_damp * _pitch_rate * _body_y_world.z,
            ),
            False,  # world frame
        )

        # ── 4d. M4-road: query road surface at wheel contact positions ────────
        # road.query() is called each step for both wheels; returns height,
        # normal, and mu_dry.  Height is stored in telemetry (PyChrono's flat
        # ground box remains the physical contact surface for M1–M4).
        # Normal is used to correct camber (gamma) for road banking.
        # mu_dry scales Pacejka output forces (1.0 = dry asphalt, no change).
        road_f = self.road.query(pos_f.x, pos_f.y)
        road_r = self.road.query(pos_r.x, pos_r.y)

        # ── 5. Tire kinematics ────────────────────────────────────────────────
        # Front wheel heading includes steer angle projected onto horizontal.
        _psi_front = yaw + self.steer_kin_sign * _steer_ang_raw * math.cos(p.rake)

        alpha_f, kappa_f, gamma_f, vx_f = self._tire_kinematics(
            pos_f, rot_f, vel_f, omega_f, _psi_front, p.front_wheel_radius, Fz_f,
            road_normal=road_f.normal,
        )
        alpha_r, kappa_r, gamma_r, vx_r = self._tire_kinematics(
            pos_r, rot_r, vel_r, omega_r, yaw, p.rear_wheel_radius, Fz_r,
            road_normal=road_r.normal,
        )

        # ── 6a. Kappa: free-roll clamp then ZOH relaxation ───────────────────
        # PyChrono's SMC contact + HHT creates a small persistent angular
        # velocity bias on free-rolling wheels (~0.15% slip).  This is
        # numerical noise, not real slip.  Clamp it to zero here so the
        # Pacejka model doesn't accumulate a growing braking force.
        # kappa is only meaningful when an axle torque is actively applied.
        #
        # When |net_torque| > KAPPA_TORQUE_THRESHOLD the kinematic kappa is
        # used (drive/brake genuinely changes the slip ratio).
        KAPPA_TORQUE_THRESHOLD = 2.0   # [N·m]
        net_rear_torque_raw  = inp.throttle * p.engine_torque_max - inp.brake_rear  * p.brake_torque_max * 0.6
        net_front_torque_raw = -inp.brake_front * p.brake_torque_max * 0.4

        kappa_f_kin = kappa_f if abs(net_front_torque_raw) > KAPPA_TORQUE_THRESHOLD else 0.0
        kappa_r_kin = kappa_r if abs(net_rear_torque_raw)  > KAPPA_TORQUE_THRESHOLD else 0.0

        # ZOH: κ'[k+1] = κ_kin + (κ'[k] - κ_kin)·exp(-vx·dt/σ)
        _exp_f = math.exp(-vx_f * self.dt / self._sigma_kappa)
        _exp_r = math.exp(-vx_r * self.dt / self._sigma_kappa)
        self._kappa_front = kappa_f_kin + (self._kappa_front - kappa_f_kin) * _exp_f
        self._kappa_rear  = kappa_r_kin + (self._kappa_rear  - kappa_r_kin) * _exp_r

        # ── 6b. Tire forces ───────────────────────────────────────────────────
        Fx_tf, Fy_tf, Mz_f = self.front_tire.compute_forces(
            alpha_f, self._kappa_front, Fz_f, gamma_f, vx_f, self.dt,
        )
        Fx_tr, Fy_tr, Mz_r = self.rear_tire.compute_forces(
            alpha_r, self._kappa_rear,  Fz_r, gamma_r, vx_r, self.dt,
        )

        # ── 6c. M4-road: scale forces by road surface friction ────────────────
        # road.mu_dry = 1.0 for dry asphalt (FlatPlane default) → no-op.
        # For other surfaces (mu<1): all Pacejka outputs scale proportionally.
        Fx_tf *= road_f.mu_dry; Fy_tf *= road_f.mu_dry; Mz_f *= road_f.mu_dry
        Fx_tr *= road_r.mu_dry; Fy_tr *= road_r.mu_dry; Mz_r *= road_r.mu_dry

        # ── 7. Apply drive/brake torques and tire forces to wheels ───────────
        # Forward rolling: ω_y > 0.  Positive Y torque accelerates forward spin.
        # Drive torque increases ω_y (traction); brake torque decreases ω_y.
        net_rear_torque  = inp.throttle * p.engine_torque_max - inp.brake_rear  * p.brake_torque_max * 0.6
        net_front_torque = -inp.brake_front * p.brake_torque_max * 0.4

        t_rear_vec  = chrono.ChVector3d(0.0, net_rear_torque,  0.0)
        t_front_vec = chrono.ChVector3d(0.0, net_front_torque, 0.0)
        self.rear_wheel.AccumulateTorque(self._acc_rear,  t_rear_vec,  False)
        self.front_wheel.AccumulateTorque(self._acc_front, t_front_vec, False)

        # ── 8. Apply tire forces at contact patches ───────────────────────────
        self._apply_tire_force(
            self.front_wheel, self._acc_front,
            pos_f, Fx_tf, Fy_tf, Mz_f,
            _psi_front, p.front_wheel_radius, gamma_f,
        )
        self._apply_tire_force(
            self.rear_wheel, self._acc_rear,
            pos_r, Fx_tr, Fy_tr, Mz_r,
            yaw, p.rear_wheel_radius, gamma_r,
        )

        # ── 9. Advance simulation ─────────────────────────────────────────────
        self.system.DoStepDynamics(self.dt)

        # Accumulators are cleared manually each step (not auto-cleared by SMC)
        self.front_wheel.EmptyAccumulator(self._acc_front)
        self.rear_wheel.EmptyAccumulator(self._acc_rear)
        self.main_frame.EmptyAccumulator(self._acc_frame)

        # ── 10. Telemetry ─────────────────────────────────────────────────────
        # M1 is not a reversing model. Near standstill under braking the tire
        # relaxation state can numerically drive the chassis into reverse, which
        # then makes the dashboard show large negative speeds and confusing wheel
        # attitudes. Clamp the vehicle to a stop instead of allowing reverse.
        new_frame_rot = self.main_frame.GetRot()
        qn = new_frame_rot
        e0n, e1n, e2n, e3n = qn.e0, qn.e1, qn.e2, qn.e3
        yaw_n = math.atan2(2*(e0n*e3n + e1n*e2n), 1 - 2*(e2n*e2n + e3n*e3n))
        fwd_xy = chrono.ChVector3d(math.cos(yaw_n), math.sin(yaw_n), 0.0)
        new_frame_vel = self.main_frame.GetPosDt()
        v_forward = new_frame_vel.x * fwd_xy.x + new_frame_vel.y * fwd_xy.y
        braking_cmd = max(inp.brake_front, inp.brake_rear)

        if braking_cmd > 0.05 and v_forward < 0.35:
            for body in (
                self.main_frame,
                self.swingarm,
                self.front_fork,
                self.front_wheel,
                self.rear_wheel,
            ):
                v = body.GetPosDt()
                v_long = v.x * fwd_xy.x + v.y * fwd_xy.y
                v_corr = chrono.ChVector3d(
                    v.x - v_long * fwd_xy.x,
                    v.y - v_long * fwd_xy.y,
                    v.z,
                )
                body.SetPosDt(v_corr)
            self.front_wheel.SetAngVelParent(chrono.ChVector3d(0.0, max(0.0, self.front_wheel.GetAngVelParent().y), 0.0))
            self.rear_wheel.SetAngVelParent(chrono.ChVector3d(0.0, max(0.0, self.rear_wheel.GetAngVelParent().y), 0.0))
            self._kappa_front = 0.0
            self._kappa_rear  = 0.0
            new_frame_vel = self.main_frame.GetPosDt()

        ax = (new_frame_vel.x - frame_vel.x) / self.dt
        ay = (new_frame_vel.y - frame_vel.y) / self.dt

        # ── M3: Suspension travel ─────────────────────────────────────────────
        # Positive = compressed (fork_lower moved toward fork_upper).
        fork_travel = self._front_susp_rest_length - self._front_susp.GetLength()
        rear_travel = self._rear_susp_rest_length  - self._rear_susp.GetLength()

        return {
            "t": t,
            "chassis": {
                "roll":      roll,
                "pitch":     pitch,
                "yaw":       yaw,
                "roll_rate": frame_omega.x,
                "yaw_rate":  frame_omega.z,
                "vx":        frame_vel.x * cos_yaw + frame_vel.y * sin_yaw,
                "vy":       -frame_vel.x * sin_yaw + frame_vel.y * cos_yaw,
                "ax":        ax,
                "ay":        ay,
            },
            "front": {
                "alpha":       alpha_f,
                "kappa":       self._kappa_front,
                "Fz":          Fz_f,
                "Fx":          Fx_tf,
                "Fy":          Fy_tf,
                "Mz":          Mz_f,
                "alpha_prime": self.front_tire.alpha_prime,
                "T_tread":     self.front_tire.state.T_tread,
                "T_carcass":   self.front_tire.state.T_carcass,
                "wear":        self.front_tire.state.wear,
                "pressure":    self.front_tire.state.pressure,
            },
            "rear": {
                "alpha":       alpha_r,
                "kappa":       self._kappa_rear,
                "Fz":          Fz_r,
                "Fx":          Fx_tr,
                "Fy":          Fy_tr,
                "Mz":          Mz_r,
                "alpha_prime": self.rear_tire.alpha_prime,
                "T_tread":     self.rear_tire.state.T_tread,
                "T_carcass":   self.rear_tire.state.T_carcass,
                "wear":        self.rear_tire.state.wear,
                "pressure":    self.rear_tire.state.pressure,
            },
            "inputs": {
                "throttle":    inp.throttle,
                "brake_front": inp.brake_front,
                "brake_rear":  inp.brake_rear,
                "steer_torque": inp.steer_torque,
                "steer_angle": _steer_ang_raw,
                "lean_delta":  inp.lean_delta,
            },
            "suspension": {
                "fork_travel": fork_travel,   # [m]  + = compressed
                "rear_travel": rear_travel,   # [m]  + = compressed
            },
            "road": {
                "front_mu":     road_f.mu_dry,
                "front_normal": road_f.normal,
                "front_height": road_f.height,
                "rear_mu":      road_r.mu_dry,
                "rear_normal":  road_r.normal,
                "rear_height":  road_r.height,
            },
        }

    def _static_Fz(self, roll: float = 0.0) -> tuple[float, float]:
        """
        Normal load split — M2 longitudinal + M3 lateral weight transfer.

        Returns (Fz_front, Fz_rear).

        Longitudinal: ΔFz_long = m·ax·h/L  (braking → forward, accel → rearward)
        Lateral:      ΔFz_lat  = m·ay·h·sin(roll)/L
            During cornering the leaned inertial force has a pitch component
            proportional to sin(roll) that redistributes load front/rear.
        Both ax and ay are T=0.1 s EMA-filtered to suppress HHT noise.
        """
        p = self.params
        total_mass = (
            p.main_frame_mass + p.swingarm_mass + p.front_fork_mass +
            p.front_wheel_mass + p.rear_wheel_mass
        )
        total_weight = total_mass * 9.81
        frac_front = p.cg_x / p.wheelbase
        frac_rear  = 1.0 - frac_front
        dFz_long = total_mass * self._ax_smooth * p.cg_height / p.wheelbase
        dFz_lat  = total_mass * self._ay_smooth * p.cg_height * math.sin(roll) / p.wheelbase
        dFz = dFz_long + dFz_lat
        return (
            max(10.0, total_weight * frac_front - dFz),
            max(10.0, total_weight * frac_rear  + dFz),
        )

    def _tire_kinematics(
        self,
        pos:         chrono.ChVector3d,
        rot:         chrono.ChQuaterniond,
        vel:         chrono.ChVector3d,
        omega:       chrono.ChVector3d,
        yaw:         float,
        R:           float,
        Fz:          float,
        road_normal: tuple = (0.0, 0.0, 1.0),
    ):
        """
        Compute (alpha, kappa, gamma, vx) for one wheel.
        Fz is passed in from _static_Fz() (not from ground spring).

        road_normal: road surface unit normal at this wheel's contact patch.
            Flat road: (0, 0, 1) — no correction.
            Banked road: tilted normal → adjusts effective camber (gamma).
        """
        # Caller passes the correct heading: yaw for rear, yaw+δ·cos(rake) for front.
        psi = yaw
        cos_psi = math.cos(psi); sin_psi = math.sin(psi)

        # Velocity in tire frame
        vx_t =  vel.x * cos_psi + vel.y * sin_psi
        vy_t = -vel.x * sin_psi + vel.y * cos_psi

        vx_t_safe = max(abs(vx_t), 1e-3)

        # Slip angle — Pacejka convention: α = -atan(Vy/Vx) where Vy is
        # lateral velocity in the tire frame.  Our vy_t = v projected onto
        # the tire's Y axis has the opposite sign to the Pacejka lateral
        # velocity, so negate to match: positive α = wheel heading LEFT of
        # velocity → Fy > 0 (centripetal leftward force for left steer).
        alpha = -math.atan2(vy_t, vx_t_safe)

        # Kinematic longitudinal slip ratio (positive = traction, negative = braking).
        # Caller applies ZOH longitudinal relaxation before passing to tire model.
        kappa = (R * omega.y - vx_t_safe) / (vx_t_safe + 1e-3)
        kappa = max(-1.0, min(1.0, kappa))

        # Camber = chassis roll − road banking angle.
        # On flat ground: bank = 0, gamma = roll (unchanged from M1–M3).
        # On banked road: road normal tilts laterally → subtract bank from roll.
        # Banking angle about the wheel heading axis:
        #   lat_dir = (−sin(ψ), cos(ψ)) in XY plane
        #   lat_component = nx·lat_x + ny·lat_y
        #   bank = atan2(lat_component, nz)
        frame_q = self.main_frame.GetRot()
        e0f, e1f, e2f, e3f = frame_q.e0, frame_q.e1, frame_q.e2, frame_q.e3
        roll_frame = math.atan2(2*(e0f*e1f + e2f*e3f), 1 - 2*(e1f*e1f + e2f*e2f))

        nx, ny, nz = road_normal
        lat_n  = nx * (-sin_psi) + ny * cos_psi
        bank   = math.atan2(lat_n, max(nz, 0.01))
        gamma  = roll_frame - bank

        return alpha, kappa, gamma, vx_t_safe

    # ─────────────────────────────────────────────────────────────────────────
    def _apply_tire_force(
        self,
        wheel_body:  chrono.ChBody,
        acc_idx:     int,
        wheel_pos:   chrono.ChVector3d,
        Fx_tire:     float,
        Fy_tire:     float,
        Mz_tire:     float,
        yaw:         float,
        R:           float,
        gamma:       float,
    ) -> None:
        """
        Transform tire-frame forces to world frame and apply at contact patch.

        Tire frame: x = wheel heading, y = lateral, z = up
        World frame: x = forward, y = left, z = up
        """
        cos_psi = math.cos(yaw); sin_psi = math.sin(yaw)

        # Rotate tire forces to world frame
        Fx_w =  Fx_tire * cos_psi - Fy_tire * sin_psi
        Fy_w =  Fx_tire * sin_psi + Fy_tire * cos_psi

        # Contact patch world position: camber-dependent lateral offset
        y_cp = self.R_CROWN * math.sin(gamma)
        cp = chrono.ChVector3d(wheel_pos.x, wheel_pos.y + y_cp, 0.0)

        # Apply force at contact patch (local=False: world-frame vectors)
        F_world = chrono.ChVector3d(Fx_w, Fy_w, 0.0)
        wheel_body.AccumulateForce(acc_idx, F_world, cp, False)

        # Aligning torque about world Z (local=False: world frame)
        Mz_world = chrono.ChVector3d(0.0, 0.0, Mz_tire)
        wheel_body.AccumulateTorque(acc_idx, Mz_world, False)

    # ─────────────────────────────────────────────────────────────────────────
    @classmethod
    def from_yaml(
        cls,
        bike_yaml:       str | Path,
        front_tire_yaml: str | Path,
        rear_tire_yaml:  str | Path,
        road,
        rider_source,
        dt: float = 0.001,
    ) -> "Motorcycle":
        from ..tire.tire_interface import TireModel
        params     = BikeParams.from_yaml(bike_yaml)
        front_tire = TireModel.from_yaml(front_tire_yaml)
        rear_tire  = TireModel.from_yaml(rear_tire_yaml)
        return cls(params, front_tire, rear_tire, road, rider_source, dt)
