# Decision 005 — M1 PyChrono Architecture

**Date**: Pre-Milestone 1  
**Status**: Decided  
**Files affected**: `core/chassis/motorcycle.py` (Milestone 1 implementation)

---

## 1. Contact System: ChSystemSMC vs ChSystemNSC

### Decision: **ChSystemSMC** (Smooth contact, penalty method)

### Reasoning

| Criterion | ChSystemNSC (Non-smooth) | ChSystemSMC (Smooth) |
|-----------|--------------------------|----------------------|
| Contact model | Complementarity / impulse | Penalty (Hertz springs) |
| Solver | Iterative (APGD / SOR) | Direct (sparse LU) |
| Real-time @ 1 kHz | ✓ for low body count | ✓ for low body count |
| Stiff springs | Degrades (requires tiny dt) | Handles well |
| Tire force application | Either works | **Preferred** — SMC is designed for force-based contact |
| Suspension springs | Either works | **Better** — SMC preserves energy in stiff systems |
| Current rig code | Uses SMC | Consistent |

For a 6-body motorcycle with stiff suspension springs (18,500–42,000 N/m),
SMC is more numerically robust.  NSC would require careful tuning of the
iterative solver tolerance and could exhibit constraint chatter at 1 kHz.

The existing `tire_rig.py` already uses `ChSystemSMC` — consistency is
an additional reason to keep it.

---

## 2. Timestepper: HHT vs Alternatives

### Decision: **ChTimestepperHHT** (Hilber-Hughes-Taylor α-method)

Available timesteppers (from PyChrono probe): HHT, EulerImplicit,
EulerImplicitLinearized, EulerImplicitProjected, Newmark,
Trapezoidal, TrapezoidalLinearized.

### Analysis

| Timestepper | Order | A-stable? | Algorithmic damping | Notes |
|-------------|-------|-----------|---------------------|-------|
| EulerImplicitLinearized | 1 | Yes | High (first-order) | Fast but dissipative; kills high-freq dynamics |
| Trapezoidal | 2 | Yes | None | Exact energy, but no damping of spurious modes |
| **HHT** | **2** | **Yes** | **Controllable (α)** | Standard for structural/vehicle dynamics |
| Newmark | 2 | Conditionally | With γ>0.5 | HHT is a special case of Newmark |

HHT with α = −0.3:
- 2nd-order accurate (captures weave and wobble dynamics correctly)
- Unconditionally A-stable (no CFL condition on dt)
- Algorithmic dissipation removes unphysical high-frequency noise from
  numerical stiffness — critical for stiff suspension springs at 1 kHz
- α = −0.3 is the "standard" motorcycle/vehicle setting — positive ρ∞ = 0.5
  means 50% amplitude decay of spurious modes per step

`EulerImplicitLinearized` was used in the tire rig because it's the fastest
and the rig has only 2 DOF.  For the 6-body motorcycle DAE, HHT is required
to get the mode frequencies right (validates against Sharp 1971 in M2).

### HHT configuration

```python
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht = chrono.CastToChTimestepperHHT(system.GetTimestepper())
hht.SetAlpha(-0.3)
hht.SetMaxiters(10)       # Newton iterations per step
hht.SetAbsTolerances(1e-4, 1e-3)  # position, velocity
hht.SetStepControl(False)  # fixed dt — essential for real-time
hht.SetModifiedNewton(True)  # reuse Jacobian — 2–4× faster per step
```

`SetStepControl(False)` is critical: adaptive step control would break the
deterministic 1 kHz loop.

---

## 3. Tire Force Application: Contact Patch Position, Not Body Origin

### Decision: Apply tire forces at the **contact patch world position**, not at the wheel body centre of mass.

### Physics

The wheel body centre of mass is at the wheel axle (height = R_wheel above
ground).  Tire forces (Fx, Fy) act at the contact patch (ground level, offset
by the camber-dependent lateral shift y_cp = R_crown·sin(γ)).

Applying forces at the body origin instead of the contact patch:
- Introduces a spurious moment arm of R_wheel in X and Z
- Creates unphysical coupling between Fx and pitch rate
- Shifts the aligning torque pivot point

### Implementation

```python
# Contact patch world position
cp_world = chrono.ChVector3d(
    wheel_pos.x,
    wheel_pos.y + y_cp,      # lateral camber offset
    wheel_pos.z - R_wheel,   # ground level
)

# Applied force vector in world frame
F_world = chrono.ChVector3d(Fx_world, Fy_world, 0.0)

# Torque about contact patch, expressed as moment on wheel body
tau_contact = cp_world - wheel_body.GetPos()  # arm from CoM to patch
moment = tau_contact.Cross(F_world) + chrono.ChVector3d(0.0, Mz_world, 0.0)

wheel_body.AccumulateForce(F_world, cp_world, worldcoords=True)
# OR equivalently:
wheel_body.AccumulateForce(F_world, wheel_body.GetPos(), worldcoords=True)
wheel_body.AccumulateTorque(moment, worldcoords=True)
```

Using `AccumulateForce(F, application_point, worldcoords=True)` is the
preferred PyChrono API — it handles the moment arm internally.

Forces are accumulated before `system.DoStepDynamics(dt)`, cleared automatically
by PyChrono at the end of each step.  No manual reset needed.

### Reference frame for tire forces

Tire forces (Fx, Fy, Mz) are computed in the **tire frame** (x = wheel heading,
y = lateral, z = up), following ISO 8855 / Pacejka convention.

Before applying to PyChrono (world frame, Z-up, X-forward):
```python
# Wheel heading angle in world frame (yaw + steer)
cos_psi = cos(heading)
sin_psi = sin(heading)

Fx_world =  Fx * cos_psi - Fy * sin_psi
Fy_world =  Fx * sin_psi + Fy * cos_psi
# Fz = normal reaction; already in PyChrono's contact normal direction
# Mz stays about Z axis (same in tire and world frames for flat road)
```

---

## 4. No-Allocation Hot Loop Design

### Decision: Zero heap allocation per step in the tire + PyChrono loop.

### Sources of allocation identified and mitigated

| Source | Allocation | Mitigation |
|--------|-----------|------------|
| `TireModel.compute_forces()` | 12 bytes (dataclass write) | Acceptable — Python is updating existing object fields, not allocating new objects |
| `DummyTirePINN._predict_scalar()` | 0 | Pure math, no containers |
| `chrono.ChVector3d()` | ~80 bytes each | Pre-allocate reusable vectors in `__init__`; mutate with `.Set(x,y,z)` in loop |
| `chrono.ChQuaterniond()` | ~80 bytes | Pre-allocate one per body in `__init__` |
| Frame telemetry dict | ~400 bytes | Build once per WS send (every 20 steps); not in inner loop |
| `ws_server.push_frame(dict)` | ~400 bytes | Amortised: 1 frame per 20 physics steps (50 Hz send, 1 kHz physics) |

### Hot loop structure

```python
# Pre-allocate in Motorcycle.__init__
self._F_front  = chrono.ChVector3d()
self._F_rear   = chrono.ChVector3d()
self._cp_front = chrono.ChVector3d()
self._cp_rear  = chrono.ChVector3d()

def step(self, t):
    # 1. Read wheel state from PyChrono (no allocation — returns existing objects)
    pos_f   = self.front_wheel.GetPos()      # returns internal reference
    rot_f   = self.front_wheel.GetRot()
    vel_f   = self.front_wheel.GetPosDt()

    # 2. Compute tire kinematics (pure Python math, no allocation)
    alpha_f, kappa_f, gamma_f, vx_f, Fz_f = self._tire_kinematics(
        pos_f, rot_f, vel_f, self.front_axle_omega, self.road
    )

    # 3. Tire forces (1.4 µs, 12 bytes)
    Fx_f, Fy_f, Mz_f = self.front_tire.compute_forces(
        alpha_f, kappa_f, Fz_f, gamma_f, vx_f, self.dt
    )

    # 4. Transform to world frame and apply (reuse pre-allocated vectors)
    self._F_front.Set(Fx_world_f, Fy_world_f, 0.0)
    self._cp_front.Set(cpx_f, cpy_f, 0.0)
    self.front_wheel.AccumulateForce(self._F_front, self._cp_front, True)
    # ... same for rear ...

    # 5. PyChrono step (~200–500 µs, internal allocation managed by PyChrono)
    self.system.DoStepDynamics(self.dt)

    # 6. Telemetry (every 20 steps)
    if self._ws_counter >= 20:
        self._ws_counter = 0
        ws_server.push_frame(self._build_frame())   # one dict per 20 steps
    self._ws_counter += 1
```

The only unavoidable allocation is inside `system.DoStepDynamics()` (PyChrono
C++ side) — this is outside our control and is typically < 1 KB per step for
a 6-body system with sparse LU.

---

## 5. Initial Condition for M1 Tests

Per the product spec:
- **Straight-line stability at 5 m/s** is the first sign-of-life test.
- **Not** "sit stably at rest" (requires active balancing; a passive motorcycle
  falls over below ~2 m/s).

Initial condition:
```python
# Main frame: position at origin, upright (identity quaternion)
# Both wheels: omega = v / R_wheel (rolling without slip)
# All joints: at design position (suspension at static sag)
# Tire states: alpha_prime = 0, computed from v=5 m/s, alpha=0
system.GetBodies()[frame_idx].SetPosDt(chrono.ChVector3d(5.0, 0.0, 0.0))
front_wheel.SetAngVelParent(chrono.ChVector3d(0, -5.0/R_front, 0))
rear_wheel.SetAngVelParent(chrono.ChVector3d(0, -5.0/R_rear, 0))
```

At 5 m/s with correct geometry, the motorcycle is above the capsize speed and
should exhibit mild directional stability (capsize mode is stable above ~3 m/s
for typical motorcycles).

---

## 6. Steering: Torque, Not Angle

Confirmed: steer input is `steer_torque [N·m]`, applied as a torque about the
steering axis joint.

```python
# Applied each step to the steering revolute joint:
steer_joint.SetTorqueFunction(chrono.ChFunctionConst(steer_torque_Nm))
```

Gamepad right stick X → steer_torque linearly scaled to ±30 N·m.
Dashboard slider: −50 N·m to +50 N·m.

Positive steer torque = torque in the +Z direction about the steering axis
(leaning forward from steering head) = tends to turn wheel **right** initially,
then through countersteering dynamics produces **left turn**.

---

## 7. Countersteering Sign Convention

**Test (M1 gate)**: at 10 m/s steady straight line, apply +2 N·m steer torque
for 0.5 s.  Expected result: motorcycle turns **LEFT**.

Why: +2 N·m turns wheel right → gyroscopic reaction rolls bike left →
left turn (countersteering).  If the bike turns right, the sign of either the
steer torque application OR the gyroscopic coupling is wrong.

Sign check in PyChrono: ensure the steering axis revolute joint is oriented so
that +torque corresponds to rightward wheel deflection (positive steer angle).
Verify by checking that a positive steer angle command deflects the front wheel
centerline to the right in the world frame.
