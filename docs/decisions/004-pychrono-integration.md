# Decision 004 — PyChrono as Multibody Solver (Not Hand-Written ODEs)

**Date**: Milestone 0 (pre-decision for Milestone 1)  
**Status**: Decided  
**Files affected**: `core/chassis/motorcycle.py`

---

## Decision

PyChrono (`ChSystemSMC` + `ChTimestepperHHT`) solves the full constrained
multibody DAE.  Tire forces are applied as external forces before each step.

Specifically:
- 6 rigid bodies (main frame, swingarm, front fork, front wheel, rear wheel)
- 6 joints (steering axis, swingarm pivot, front axle, rear axle,
  front suspension, rear suspension)
- Tyre forces: computed outside PyChrono, applied via `ChForce` each step
- Target integration step: 1 ms (1 kHz)

## Why not hand-write chassis ODEs

Implementing a full 6-DOF constrained multibody system from scratch requires:
1. **Constraint Jacobians** for all 6 joints (steering revolute, 4 revolutes,
   2 spring-dampers) — error-prone derivation, especially for the steering
   axis with non-zero rake angle
2. **Baumgarte stabilisation** or projection to prevent constraint drift
3. **Sparse linear algebra** for the constraint force multipliers
4. **Gyroscopic terms** for spinning wheel inertia — easy to get wrong

PyChrono handles all of these correctly.  The API is:
- Build once in `Motorcycle.build()`
- Call `system.DoStepDynamics(dt)` at 1 kHz
- Read body positions, velocities, orientations via `GetPos()`, `GetRot()`

## Timestep target

1 ms (1 kHz).  `ChTimestepperHHT` (Hilber-Hughes-Taylor α-method) is
unconditionally stable for linear problems and well-behaved for the
motorcycle stiffness range.  HHT α = −0.3 (standard, slightly dissipative,
filters high-frequency noise without distorting the dynamics of interest).

Alternative considered: Euler implicit linearised.  Rejected: less accurate
for the suspension spring-damper at 1 kHz; HHT preferred for stiff systems.

## Tire force application

At each step:
1. Read wheel state (position, velocity, orientation quaternion) from PyChrono
2. Compute slip kinematics in the tire frame
3. Call `TireModel.compute_forces()` → (Fx, Fy, Mz)
4. Apply as body-fixed forces on wheel body via `ChForce`
5. Call `system.DoStepDynamics(dt)`

This sequence means tire forces are one step behind the wheel kinematics —
same explicit-explicit coupling used in all real-time vehicle simulators
(VI-CarRealTime, CarMaker, etc.) and stable at 1 kHz for the force magnitudes
involved.
