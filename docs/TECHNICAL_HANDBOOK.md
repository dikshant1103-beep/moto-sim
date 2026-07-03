# Motorcycle Simulation System — Technical Handbook

**Audience:** Mechanical engineer with no ML background.
**Purpose:** Understand the complete system, explain it confidently to any technical interviewer or collaborator.
**Scope:** Everything from physics first-principles through the neural-network integration plan.

---

## 1. System Overview

### The Full Pipeline

```
Rider Inputs
(dashboard sliders / gamepad)
        │
        ▼  WebSocket (50 Hz)
┌───────────────────┐
│   ws_server.py    │  receives {throttle, brake, steer_torque, setup params}
└───────────────────┘
        │
        ▼  1 kHz (dt = 1 ms)
┌────────────────────────────────────────────────────────────────┐
│                    moto_sim_main.py                            │
│                                                                │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │               Motorcycle.step(t)                        │  │
│  │                                                         │  │
│  │   1. Read rider inputs                                  │  │
│  │   2. Read wheel positions / velocities from PyChrono    │  │
│  │   3. Compute chassis roll, pitch, yaw (from quaternion) │  │
│  │   4. Compute Fz (static + dynamic weight transfer)      │  │
│  │   5. Compute tire kinematics (α, κ, γ, Vx per wheel)   │  │
│  │   6. ZOH relaxation update (α' and κ')                  │  │
│  │   7. Call TireModel → (Fx, Fy, Mz) per wheel           │  │
│  │   8. Apply drive/brake torques to wheel bodies          │  │
│  │   9. Apply tire forces at contact patch                 │  │
│  │  10. sys.DoStepDynamics(dt) — PyChrono integrates       │  │
│  └─────────────────────────────────────────────────────────┘  │
│                    ↕  telemetry dict                           │
│  Push to WebSocket every 20th step (50 Hz output)             │
└────────────────────────────────────────────────────────────────┘
        │
        ▼  WebSocket (50 Hz)
┌───────────────────┐
│   dashboard/      │  React app, Chart.js panels, live display
└───────────────────┘
```

### Key Design Decisions

- **1 kHz physics, 50 Hz telemetry.** The tire model and PyChrono integrator run at every step. Only every 20th step is pushed over WebSocket. This matches real ECU sampling rates and keeps the WS channel light.
- **Hybrid force application.** PyChrono handles gravity, inertia, and constraint forces. Pacejka tire forces are injected as external force accumulators — not as contact friction. This avoids PyChrono's simplistic Coulomb friction model and gives full Pacejka accuracy.
- **Zero contact friction in PyChrono.** The `ChContactMaterialSMC` has `SetFriction(0.0)`. This is intentional — all tangential tire forces come from `AccumulateForce()` calls using the Pacejka model.

---

## 2. Tire Physics Model (Detailed)

The tire model lives in `core/tire/pinn_model.py`, class `DummyTirePINN`. Despite the name "PINN", it currently runs **pure analytical Pacejka** physics. The neural network placeholder (`self.net`) exists but is never called in the production path. The entry point for all callers is `TireModel.compute_forces()` in `core/tire/tire_interface.py`.

### 2.1 Inputs to the Tire Model

At each 1 ms step, per wheel:

| Symbol | Meaning | Unit |
|--------|---------|------|
| α (alpha) | Kinematic slip angle | rad |
| κ (kappa) | Longitudinal slip ratio | — |
| Fz | Normal load on tire | N |
| γ (gamma) | Camber angle (lean) | rad |
| Vx | Longitudinal speed at contact | m/s |
| dt | Timestep | s |

The relaxed slip angle α' is the **internal state** that carries forward between steps.

---

### 2.2 Pure Lateral Force — Pacejka Magic Formula (Fy₀)

**Physical meaning:** When a tire rolls at an angle to its heading, rubber distorts in the contact patch. The lateral restoring force from this distortion is Fy. At small angles it is proportional to slip (linear region). At large angles the rubber saturates — you cannot pull any more lateral force out of the tire regardless of slip angle.

**Step-by-step calculation:**

```
Step 1 — Peak force capacity
    D = μ · Fz

    μ is friction coefficient (default 0.90 for medium compound).
    Fz is normal load. D is the maximum possible lateral force — the "plateau"
    of the slip curve. It scales linearly with load because more load = more
    rubber in the contact patch = more grip.

Step 2 — Curvature factor B (stiffness normalised by D)
    B = (k_alpha · Fz) / (C_pac · D + ε)

    k_alpha is cornering stiffness coefficient [N/(N·rad)] — a tire property.
    The product B·C·D = dFy/dα at α=0 is the cornering stiffness [N/rad].
    B·C·D ≈ 12 × 1470 × 0.90 / (1.80 × ...) ≈ 8800 N/rad for a typical sports tire.

Step 3 — Shape function φ (Pacejka's nonlinear compression)
    Ba  = B · α'            (α' is the relaxed slip angle)
    φ   = Ba − E · (Ba − atan(Ba))

    E is the curvature factor [–]. In this code E = 0 (simplified).
    When E = 0: φ = Ba. For E > 0, the curve bends at peak.

Step 4 — Pacejka lateral force
    Fy_pac = D · sin(C_pac · atan(φ))

    C_pac = 1.80. The atan() maps the real line to (−π/2, +π/2).
    sin(C_pac · atan(·)) creates the characteristic S-shaped curve:
    rises from zero, peaks, then asymptotes to a plateau.

Step 5 — Add camber thrust
    Fy_cam = k_gamma · Fz · γ
    Fy₀   = clamp(Fy_pac + Fy_cam, −D, +D)

    A leaned tire generates lateral force even at zero slip angle.
    This is camber thrust — caused by the conical contact geometry.
    The clamp prevents the combined force from exceeding friction capacity.
```

In code (`_predict_scalar`):
```python
D     = self.mu * Fz
B     = (self.k_alpha * Fz) / (self.C_pac * D + 1e-8)
Ba    = B * alpha
phi   = Ba - self.E_pac * (Ba - math.atan(Ba))
Fy_pac = D * math.sin(self.C_pac * math.atan(phi))
Fy     = max(-D, min(D, Fy_pac + self.k_gamma * Fz * gamma))
```

---

### 2.3 Pure Longitudinal Force — Simplified Pacejka (Fx₀)

**Physical meaning:** When the wheel spins faster than the road speed (traction) or slower (braking), rubber slips longitudinally. This generates a forward or braking force. The relationship is nearly linear at small slip, then saturates.

```
Fx₀ = μ · Fz · tanh(Kx · κ)

    κ (kappa): longitudinal slip ratio
        = (R·ω − Vx) / (|Vx| + ε)
        Positive = wheel spinning faster than road (traction)
        Negative = wheel slower than road (braking)
    Kx = 12.0 — longitudinal slip stiffness coefficient
    tanh replaces the full Pacejka B-C-D formula for Fx (valid approximation
    for a first-principles model where real Fx coefficients are unavailable)
```

---

### 2.4 Why Combined Slip Matters

A tire operating simultaneously with lateral slip AND longitudinal slip cannot produce full force in both directions at once. The contact patch rubber can only deform so much — it has a finite friction budget. If 80% of the budget is used for braking, only 20% remains for cornering.

This is not just a safety consideration. Ignoring it causes the simulation to generate physically impossible forces — more total force than friction allows — which leads to instability.

---

### 2.5 Combined Slip — Pacejka Weighting Factors (§4.3.2)

The Pacejka (2012) textbook solution uses smooth weighting functions that reduce each pure-slip force based on the other slip component. This code uses that exact formulation:

```
Gxα = cos(Cxα · atan(Bxα · α'))    ← how much α reduces Fx
Gyκ = cos(Cyκ · atan(Byκ · κ))     ← how much κ reduces Fy

Combined forces:
    Fx = Gxα · Fx₀
    Fy = Gyκ · Fy₀
```

**Why cos() and not something simpler?**

- When α = 0: atan(0) = 0, cos(0) = 1.0 → G = 1 → full force. Correct.
- As |α| grows: atan → π/2, with Cxα = 1.0 → cos(π/2) = 0 → G → 0. Force reduced. Correct.
- With Cxα = Cyκ = 1.0 (as coded): the argument of cos is always in (−π/2, +π/2), so cos is always ≥ 0. **This guarantees forces never flip sign due to the weighting** — a critical numerical stability property.
- The function is C∞ continuous (infinitely differentiable) at all (α, κ) values. No if/else branches, no discontinuities. This matters for both solver stability and future automatic differentiation.

In code (`predict_combined`):
```python
Gxa = math.cos(self.Cxa * math.atan(self.Bxa * slip_angle))
Gyk = math.cos(self.Cyk * math.atan(self.Byk * kappa))
Fx  = Gxa * Fx0
Fy  = Gyk * Fy0
```

**Critical ordering note:** The combined-slip weighting is applied to Fy **before** computing Mz. This is the B3 fix (bug fix) in this codebase: Mz uses the combined Fy, not the pure-slip Fy_pac. If Mz were computed from pure-slip Fy but the simulation applies combined-slip Fy, the aligning torque and lateral force become inconsistent — the self-aligning torque no longer physically matches the actual lateral force generated.

---

### 2.6 Pneumatic Trail Model

**Physical meaning:** The lateral force in the contact patch does not act at the geometric center of the tire. It acts slightly behind center — at the "pneumatic trail" distance `tp` — creating a self-aligning moment that turns the steering back toward straight-ahead. This is the primary mechanism that makes a free-steering bicycle or motorcycle steer toward center.

```
tp = tp0 · cos(Ct · atan(Bt · α'))

    tp0 = 0.030 m  — maximum pneumatic trail (at zero slip)
    Bt  = 10.0     — trail decay rate with slip
    Ct  = 1.50     — trail shape factor

Behaviour:
    α' → 0    : tp = tp0 · cos(0) = tp0 = 30 mm   (maximum self-aligning)
    α' → peak : tp → 0                              (steering "goes light" at limit)
    α' > α_reversal: tp < 0                         (Mz reverses — "trail reversal")
```

The trail reversal angle (where tp first reaches zero) is:
```
α_reversal = atan(tan(π/(2·Ct)) / Bt)   [from trail_reversal_deg() method]
```
For Ct=1.50, Bt=10.0: reversal at ~8.3°. At slip angles beyond this, the tire actually tries to steer itself further into the corner — a known phenomenon in high-speed instabilities.

---

### 2.7 Aligning Torque Mz

```
Mz = −Fy · tp + K_gamma_Mz · γ · Fz

    First term:  self-aligning torque from pneumatic trail
    Second term: camber contribution to Mz (smaller effect)

    K_gamma_Mz = 0.020 [m/rad]
```

**Physical meaning of the sign:** When the tire generates leftward force (Fy > 0), the pneumatic trail creates a moment that tries to rotate the steering back to the right (reducing the slip angle). The negative sign in `−Fy · tp` gives this restorative direction.

**Why Mz uses combined Fy:** If you are simultaneously braking (κ ≠ 0), the actual lateral force is reduced by Gyκ. If Mz still used the full pure-slip Fy_pac, the aligning torque would be larger than the actual lateral force that was applied — physically inconsistent. Using the combined Fy maintains the physical relationship: Mz ∝ actual lateral force generated.

---

### 2.8 Geometric Stand-Up Torque

When the tire is leaned and generating longitudinal force Fx, the force acts at the lateral contact patch offset:

```
y_cp = R_crown · sin(γ)    [R_crown = 40 mm, crown radius of tyre toroid]
```

The force Fx applied at lateral offset y_cp creates a moment about the wheel's steering axis that tends to stand the motorcycle upright. This is handled implicitly in the code through `_apply_tire_force()`:

```python
y_cp = self.R_CROWN * math.sin(gamma)
cp   = ChVector3d(wheel_pos.x, wheel_pos.y + y_cp, 0.0)
wheel_body.AccumulateForce(acc_idx, F_world, cp, False)
```

By applying Fx at the laterally-offset contact point rather than the wheel centerline, PyChrono automatically computes the resulting moment — including the stand-up torque from traction/braking. No explicit separate calculation is needed; it emerges from the force application geometry.

---

### 2.9 Mechanical Trail Torque

The motorcycle's steering geometry has geometric trail (the contact patch is behind the steering axis projection on the ground). When lateral tire forces act at the contact patch, they create a torque about the steering axis proportional to the trail distance. This torque is **handled automatically by PyChrono** through the joint constraint forces — it is NOT manually added. The `ChLinkMotorRotationTorque` steering joint transmits this reaction force correctly through the constraint equations.

**Critical**: Do not manually add a `trail × Fy` torque term to the steering — PyChrono already computes this from the geometry. Double-counting it would destabilize the steering dynamics.

---

## 3. Relaxation Dynamics

### 3.1 Why Instantaneous Slip Is Physically Wrong

At each timestep, the tire kinematics code computes the kinematic slip angle:

```python
alpha = -math.atan2(vy_t, vx_t_safe)
```

This is the **geometric** slip — the angle between the wheel heading and the velocity vector. If the tire model used this directly, it would mean the tire instantly generates its full lateral force the moment the steering angle changes. In reality, the rubber in the contact patch takes time to distort.

Physically: the contact patch is ~150 mm long. At 10 m/s the tire travels through its own contact patch length in 15 ms. The lateral distortion of the rubber cannot fully develop faster than this. The relevant length scale is the **relaxation length σ** — approximately equal to the half-contact-patch length (≈0.30 m for a motorcycle tire).

Without relaxation: applying a sudden 5° steering input produces an instant, full-magnitude lateral force spike. This creates an unphysical impulse, excites high-frequency oscillations, and typically destabilizes the simulation.

### 3.2 The ZOH Relaxation ODE

The relaxation is modeled as a first-order lag in slip angle space:

```
σ/Vx · dα'/dt + α' = α     (continuous time)
```

where α' is the relaxed (filtered) slip angle and α is the kinematic slip angle.

The time constant is τ = σ/Vx. At higher speeds (large Vx), τ is small — the tire responds faster. At low speed, τ is large — the tire responds sluggishly, which matches reality (a barely-moving motorcycle has very slow lateral force buildup).

**Discretization — Zero-Order Hold (ZOH):**

```
α'[k+1] = α + (α'[k] − α) · exp(−|Vx| · dt / σ)
```

In code (`tire_interface.py`):
```python
vx_safe = max(abs(vx), 1e-3)
decay   = math.exp(-vx_safe * dt / self._sigma)
self._state.alpha_prime = alpha + (self._state.alpha_prime - alpha) * decay
```

**Why ZOH and not Euler?** Forward Euler (`α'[k+1] = α'[k] + dt/τ·(α−α')`) is conditionally stable — it can go unstable if dt > 2τ. ZOH is **A-stable** (unconditionally stable regardless of dt). For the 1 kHz loop with σ=0.30 m and Vx potentially as low as 0.001 m/s, the time constant can be extremely large and Euler would need tiny timesteps. ZOH handles any ratio of dt/τ correctly.

### 3.3 Longitudinal Slip Relaxation

The same ZOH is applied to the longitudinal slip ratio κ. This prevents a separate instability: PyChrono's SMC contact + HHT integrator creates a small persistent angular velocity bias on free-rolling wheels (~0.15% slip). Without relaxation, this bias would accumulate into a growing braking force over time.

```python
# ZOH: κ'[k+1] = κ_kin + (κ'[k] − κ_kin)·exp(−Vx·dt/σ_kappa)
_exp_f = math.exp(-vx_f * self.dt / self._sigma_kappa)
self._kappa_front = kappa_f_kin + (self._kappa_front - kappa_f_kin) * _exp_f
```

Additionally, κ is clamped to zero when no significant axle torque is applied (|net torque| < 2 N·m threshold), because kinematic slip in that regime is purely numerical noise.

### 3.4 Low-Speed Behaviour

Both relaxation ODEs use `vx_safe = max(|Vx|, 1e-3)` to prevent division by zero. At standstill, the time constant τ = σ/1e-3 = 300 s, meaning α' barely moves — the tire retains its previous state. This is acceptable because at zero speed there are no significant tire forces anyway.

---

## 4. Multibody System (PyChrono)

### 4.1 Bodies

Six dynamic bodies are created in `Motorcycle.build()`:

| Body | Mass | Key role |
|------|------|---------|
| `main_frame` | 165 kg | Engine + rider + tank, carries main inertia tensor (Ixx, Iyy, Izz, Ixz) |
| `swingarm` | 5 kg | Pivots about rear of frame, carries rear axle |
| `front_fork` (upper, `fu`) | 40% of 7.5 kg | Inner fork tubes, fixed to steering head, steers with frame |
| `fork_lower` (`fl`) | 60% of 7.5 kg | Outer legs, slides along fork axis (M3), carries front wheel |
| `front_wheel` | 4.5 kg | Sphere collision shape radius Rf |
| `rear_wheel` | 7.0 kg | Sphere collision shape radius Rr |

The ground is a 7th fixed body (ChBodyEasyBox, 2000×200×0.02 m).

**Gyroscopic torques** (`SetUseGyroTorque(True)`) are enabled on all rotating bodies. This is essential for the gyroscopic precession that causes countersteering — without it the bike would not exhibit the correct lean-steer coupling.

### 4.2 Joints

| Joint | Type | Between | Purpose |
|-------|------|---------|---------|
| Swingarm pivot | `ChLinkLockRevolute` (Y-axis) | frame ↔ swingarm | Swingarm rotation for suspension travel |
| Rear axle | `ChLinkLockRevolute` (Y-axis) | swingarm ↔ rear_wheel | Wheel spin DOF |
| Steering motor | `ChLinkMotorRotationTorque` (steer axis) | frame ↔ fork_upper | Applies steer torque, reads steer angle/rate |
| Fork prismatic | `ChLinkLockPrismatic` (fork axis) | fork_upper ↔ fork_lower | Fork compression DOF (M3) |
| Front axle | `ChLinkLockRevolute` (Y-axis) | fork_lower ↔ front_wheel | Wheel spin DOF |
| Front TSDA | `ChLinkTSDA` | fork_upper ↔ fork_lower | Spring + damper along fork axis (M3) |
| Rear TSDA | `ChLinkTSDA` | frame ↔ swingarm | Rear spring + damper (M2/M3) |

### 4.3 Steering Axis Geometry

The steering axis does not point straight up — it leans rearward by the rake angle (24° for CBR600RR). This geometry is critical for generating geometric trail, which is the distance between where the steering axis projects onto the ground and where the tire actually contacts the ground.

```
Trail t: the contact patch is behind the steering axis projection.
```

The steering head position is computed geometrically so that the correct trail emerges:

```python
# Condition: psh_x + psh_z * tan(rake) = L + trail
_psh_z = p_front_axle.z + fork_length * cos_r
_psh_x = L + p.trail - _psh_z * (sin_r / cos_r)
```

If this geometry is wrong, the effective trail becomes zero or negative — the lean-steer coupling inverts and the bike is unconditionally unstable.

### 4.4 Timestepper: HHT

The simulation uses PyChrono's HHT (Hilber-Hughes-Taylor) implicit timestepper:

```python
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
hht.SetAlpha(-0.3)          # numerical damping (α ∈ [−1/3, 0])
hht.SetMaxIters(10)
hht.SetAbsTolerances(1e-4, 1e-3)
```

HHT is an implicit method — it solves a nonlinear system of equations each step. For stiff mechanical systems with suspensions and constraints, implicit methods are far more stable than explicit (e.g., Runge-Kutta) at the same timestep. The `alpha = −0.3` introduces slight numerical dissipation that suppresses high-frequency ringing from contact impacts without significantly affecting the physics at 1 kHz.

### 4.5 Force Accumulators

PyChrono's `AddAccumulator()` system lets you inject forces that are automatically cleared after `DoStepDynamics()`. The code pre-allocates accumulator slots for each body:

```python
self._acc_front = fw.AddAccumulator()   # front wheel forces
self._acc_rear  = rw.AddAccumulator()   # rear wheel forces
self._acc_frame = frame_body.AddAccumulator()  # roll/yaw control
```

After each step, the code **manually clears** the accumulators:
```python
self.front_wheel.EmptyAccumulator(self._acc_front)
```
This is required because SMC contact in PyChrono does not auto-clear them. Without the manual clear, forces would accumulate (double-add) across steps — the M1 G8 test specifically validates this.

### 4.6 TSDA Suspension (M3)

`ChLinkTSDA` is PyChrono's Translational Spring-Damper-Actuator. It:
- Connects two bodies at specified world points
- Exerts a force along the line between those points
- Force = k·(length − rest_length) + c·(d_length/dt)

**Front suspension pre-load calculation:**
```python
# The spring must be pre-compressed to support the static front load.
# Otherwise, the fork immediately extends to its free length and the
# front wheel drops — the bike falls through the ground.
f_spring_front = max(0.0, fz_f_static - (front_wheel_mass + fl_mass) * 9.81) / cos_r
delta_front    = f_spring_front / front_spring_k
fs_rest_len    = fs_design_length + delta_front
```

**Why divide by cos(rake)?** The fork spring acts along the fork axis, which is inclined at the rake angle. The vertical load component projected onto the fork axis is F_vertical / cos(rake).

**Rear suspension pre-load** is computed from a torque balance about the swingarm pivot, ensuring the swingarm is in equilibrium at the design ride height.

### 4.7 Control Gains (Roll Stabilization)

The PyChrono constraint model, while geometrically correct, does not by itself produce the proprioceptive feedback a rider uses to stay balanced. An active stabilization controller is added to represent the rider's upper-body lean response:

```python
# Roll restoring torque applied about the bike's current forward axis
K_roll     = 6000.0   # N·m/rad   (> m·g·h_cg ≈ 1075 N·m/rad for stability)
C_roll     = 1200.0   # N·m·s/rad
K_yaw_damp = 3000.0   # N·m·s/rad
K_lean     = 20.0     # N·m/rad   lean→steer coupling
K_ctr      = 800.0    # N·m/rad   steer centering
C_steer    = 20.0     # N·m·s/rad steer viscous damping
```

**Why roll control about the body axis, not world axis?** Once the motorcycle yaws, the world X-axis no longer points forward along the bike. A world-frame roll correction torque would inject the wrong moment and could eventually flip the chassis. The code uses:

```python
_body_x_world = frame_rot.Rotate(ChVector3d(1, 0, 0))  # bike's forward in world frame
_tau_roll = -K_roll * sin(roll) - C_roll * roll_rate
# Apply torque about current forward axis
```

This ensures the stabilization always acts about the physically correct axis regardless of bike heading.

---

## 5. Stability and Validation (M1 + M2)

### 5.1 Straight-Line Stability (M1 G4)

**Test:** Run 3 seconds at 5 m/s with zero inputs. Require |roll| < 5° and |yaw_rate| < 3°/s throughout.

**Physics:** A bicycle/motorcycle at speed is dynamically stable because:
1. Gyroscopic precession from spinning wheels resists lean changes.
2. Geometric trail self-aligns the steering when the bike leans.
3. The active roll controller (K_roll, C_roll) provides rider-like balance.

**Why not rely on geometry alone?** In a rigid multibody solver, constraint imprecision, numerical integration error, and the absence of tire relaxation dynamics all introduce small perturbations. Without the active controller, these accumulate and cause slow instability even when the geometry is correct. The roll gains are set to be overdamped (K_roll >> critical value) for robustness.

### 5.2 Countersteering (M1 G6)

**Test:** Apply +2 N·m steer torque at 10 m/s for 0.5 s. Verify yaw_rate < −0.05°/s (bike turns LEFT).

**Physics (countersteering mechanism):**
1. Positive steer torque steers the front wheel to the RIGHT.
2. The front wheel, now rolling at an angle to its heading, generates a leftward lateral force.
3. This leftward force, applied below the center of gravity, creates a rolling moment — the bike **leans LEFT**.
4. The lean angle creates a component of gravity that acts as centripetal force — the bike turns LEFT.
5. Simultaneously, the spinning front wheel has angular momentum pointing to the left. Turning the steering axis (yaw of the fork) creates a gyroscopic torque that also promotes left lean.

**Why +torque → LEFT turn is counterintuitive:** You push the handlebar RIGHT but the bike turns LEFT. This is correct and is the mechanism all motorcyclists use above ~15 km/h.

The sign convention in PyChrono (Z-up, SAE): left turn = negative yaw_rate. The test verifies yaw_rate < −0.05°/s.

### 5.3 Roll Perturbation Decay (M2 G4)

**Test:** Apply 5° initial roll at 6 m/s; verify roll < 3° after 2 s.

**Physics:** The roll control gain K_roll = 6000 N·m/rad creates a restoring torque of 6000 × sin(5°) ≈ 523 N·m. The vehicle's roll inertia (Ixx ≈ 9.5 kg·m²) gives a natural frequency of sqrt(K_roll/Ixx) ≈ 25 rad/s — well above the weave mode frequency. The damping C_roll = 1200 makes it overdamped at 25 rad/s.

### 5.4 Weave Mode is Oscillatory (M2 G5)

**Test:** 5° perturbation produces ≥1 sign reversal within 2 s.

**Physics:** The weave mode is a coupled roll-yaw-steering oscillation. It is oscillatory (complex eigenvalue) because:
- A rolling perturbation generates lateral tire force (through slip angle).
- Lateral force generates yaw.
- Yaw creates new slip angle (geometry).
- The loop closes with a time delay from tire relaxation and steering dynamics.
- Result: oscillation, not monotone decay.

This matches Sharp (1971), who first computed the eigenvalues of the linearized bicycle equations. The weave eigenvalue has a small positive imaginary part (oscillatory) and a negative real part (stable) for typical riding conditions.

### 5.5 Speed Stability (M2 G6)

**Test:** At 10 m/s, more roll sign reversals than at 5 m/s within 3 s.

**Physics:** Weave frequency increases with speed. Higher speed → tire relaxation time constant σ/Vx decreases → faster tire response → faster oscillation → more cycles in a fixed time window. This is a fundamental prediction of Sharp (1971) and is validated here.

### 5.6 What Was Fixed Between Early Versions and Passing

Several iterations were required to get the tests to pass:

1. **Wrong K_roll gains.** Initially K_roll was too small (< m·g·h_cg ≈ 1075). The inverted pendulum instability dominated — the bike fell over. Fixed by setting K_roll = 6000 >> 1075.

2. **World-frame roll torque.** When the bike yawed, a world-frame roll correction torque injected a moment about the wrong axis. Fixed by computing the torque about the current body forward axis.

3. **Kappa instability.** HHT + SMC contact introduced small angular velocity bias on free-rolling wheels. This appeared as growing braking force. Fixed by (a) clamping κ to zero below the torque threshold, and (b) applying ZOH relaxation to κ.

4. **Steering head geometry.** Initially the steer head was placed without accounting for the correct trail. The front axle fell on the steer axis → zero geometric trail → lean-steer coupling inverted → unstable. Fixed by computing `_psh_x` from the trail condition.

---

## 6. Synthetic Dataset Generation

### 6.1 Why Real Tire Data Is Unavailable

Real tire force data comes from tire test rigs (e.g., MTS Flat-Trac, Calspan). These:
- Cost $5,000–$20,000 per tire test campaign.
- Require physical tire samples from manufacturers.
- Take days to run comprehensive sweep measurements.
- The data is proprietary — tire manufacturers do not publish it.

For a simulation project without a tire manufacturer partnership, real data is simply inaccessible.

### 6.2 Why Use Pacejka to Generate Synthetic Data

The Pacejka coefficients (μ, k_alpha, k_gamma, C_pac, E_pac, Kx, tp0, etc.) characterize a tire's behavior in a physically consistent, validated formulation. Literature provides typical coefficient ranges for road motorcycle tires. The `medium.yaml` file in this project provides one such set.

The idea: **run the Pacejka model over a grid of inputs and record the outputs as if they were "measurements".** This gives a dataset that:
- Covers the full operating envelope (including nonlinear regions).
- Is physically consistent (satisfies friction ellipse constraint).
- Has zero measurement noise (or controllable noise).
- Is free and fast to generate.

The neural network is then trained to reproduce this dataset. When trained well, the NN approximates the Pacejka model. The value is not that the NN is "smarter" than Pacejka — it is that a trained NN can be deployed in C++ (via ONNX or LibTorch) faster than calling Python + PyTorch analytically, and can potentially be retrained on real data later.

### 6.3 Input Space Definition

The four inputs span these physically relevant ranges:

| Input | Range | Notes |
|-------|-------|-------|
| Fz | 200 – 3000 N | From 0.2·static to 3·static (includes wheelie, jump landing) |
| α (slip angle) | −20° to +20° | ±0.35 rad covers all racing conditions; peak is ~4–8° |
| κ (slip ratio) | −0.5 to +0.5 | −0.5 = heavy braking; +0.5 = hard acceleration |
| γ (camber) | −45° to +45° | ±0.79 rad covers MotoGP lean angles |

### 6.4 Sampling Methods

**Grid sampling** (uniform sweep):
```python
Fz_vals    = np.linspace(200, 3000, 20)    # 20 levels
alpha_vals = np.linspace(-0.35, 0.35, 40)  # 40 levels
kappa_vals = np.linspace(-0.5, 0.5, 20)
gamma_vals = np.linspace(-0.79, 0.79, 10)
# Total: 20 × 40 × 20 × 10 = 160,000 points
```
Advantage: covers all regions. Disadvantage: wastes points in the linear region (small α, small κ) where the function is simple.

**Random sampling** (Latin Hypercube Sampling):
Samples are drawn so that projections onto each axis are uniformly distributed. Better coverage of the 4D space than naive random. Good for initial dataset.

**Importance sampling** (near nonlinear regions):
The interesting behavior is in the saturation region, near the friction limit, and at combined-slip corners of the (α, κ) space. Over-sample these regions:
```python
# More samples where |alpha| > 0.1 rad or |kappa| > 0.1
# Fewer samples in the fully linear center
```

A practical strategy: 50% uniform grid + 50% concentrated near the nonlinear knee (α ≈ ±5°, κ ≈ ±0.1).

### 6.5 Data Generation Pipeline

```python
from core.tire.tire_interface import TireModel

model = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
rows  = []

for Fz in Fz_grid:
    for alpha in alpha_grid:
        for kappa in kappa_grid:
            for gamma in gamma_grid:
                # Reset relaxation state between independent samples
                model.reset()
                # Use large Vx (10 m/s) and small dt (0.001 s) → relaxation
                # converges in <<1 step, so α' ≈ α (effectively pure kinematic)
                Fx, Fy, Mz = model.compute_forces(
                    alpha=alpha, kappa=kappa, Fz=Fz, gamma=gamma,
                    vx=10.0, dt=0.001
                )
                rows.append([Fz, alpha, kappa, gamma, Fx, Fy, Mz])

import pandas as pd
df = pd.DataFrame(rows, columns=["Fz","alpha","kappa","gamma","Fx","Fy","Mz"])
df.to_csv("dataset/tire_synthetic.csv", index=False)
```

### 6.6 Dataset Format

Each row: `[Fz, α, κ, γ] → [Fx, Fy, Mz]`

| Column | Unit | Range |
|--------|------|-------|
| Fz | N | 200–3000 |
| alpha | rad | −0.35 to +0.35 |
| kappa | — | −0.5 to +0.5 |
| gamma | rad | −0.79 to +0.79 |
| Fx | N | −2700 to +2700 |
| Fy | N | −2700 to +2700 |
| Mz | N·m | −100 to +100 |

### 6.7 Normalization

Neural networks are sensitive to input scale. If Fz ranges over [200, 3000] and α over [−0.35, +0.35], the optimizer will treat small changes in α as unimportant relative to large changes in Fz, even though they matter equally.

**Standard normalization:**
```python
# Z-score per input column (mean=0, std=1)
X_mean = X_train.mean(axis=0)    # shape: (4,)
X_std  = X_train.std(axis=0)
X_norm = (X_train - X_mean) / X_std

# Similarly for outputs
Y_mean = Y_train.mean(axis=0)
Y_std  = Y_train.std(axis=0)
Y_norm = (Y_train - Y_mean) / Y_std
```
Store X_mean, X_std, Y_mean, Y_std — they are needed at inference time to denormalize predictions back to physical units.

### 6.8 Common Mistakes to Avoid

1. **Missing saturation region.** If all samples have |α| < 3°, the NN only learns the linear zone. At 5°+ the network will extrapolate incorrectly, producing forces well above the friction limit.

2. **Missing combined-slip interaction.** If the dataset only contains pure-slip samples (either κ=0 or α=0, never both non-zero simultaneously), the NN will not learn the Gxα/Gyκ coupling. At runtime it will predict the full pure-slip force even during combined-slip — violating the friction budget.

3. **Poor sampling near the Fz=0 boundary.** Fz can drop to near-zero during a wheelie or when the suspension bottoms out. The Pacejka model degenerates at Fz=0. Include samples at Fz=100–300 N to train the near-zero behavior.

4. **Not checking physical consistency.** After training, verify:
   - |Fx²+Fy²| ≤ (μ·Fz)² everywhere (friction ellipse not violated)
   - Fy=0 at α=0, γ=0 for any Fz (no lateral force at zero slip)
   - Mz reverses sign at expected α (pneumatic trail reversal)

---

## 7. PINN Training

### 7.1 Current State

The `DummyTirePINN` class has a neural network defined:
```python
self.net = nn.Sequential(
    nn.Linear(3, 32), nn.Tanh(),
    nn.Linear(32, 32), nn.Tanh(),
    nn.Linear(32, 2),
)
```
This network exists but **its weights have never been trained**. The `forward()` method runs pure analytical Pacejka, not `self.net`. The "PINN" name is aspirational — it describes the intended production path.

### 7.2 Network Architecture for Production

The production network needs 4 inputs (Fz, α, κ, γ) and 3 outputs (Fx, Fy, Mz). A simple MLP:

```python
net = nn.Sequential(
    nn.Linear(4, 64),   nn.Tanh(),
    nn.Linear(64, 64),  nn.Tanh(),
    nn.Linear(64, 32),  nn.Tanh(),
    nn.Linear(32, 3),
)
```

**Why Tanh activations?** The Pacejka functions (sin, atan, tanh) are all smooth and bounded. Tanh neurons can represent these behaviors naturally. ReLU would struggle because the output is smooth, not piecewise linear.

**Why this depth?** The Pacejka combined-slip function has interactions between all 4 inputs. Shallow networks cannot represent these well. 3 hidden layers of width 64 is a common starting point — total parameters: 4×64 + 64×64 + 64×32 + 32×3 = 8,928 parameters. Fast to evaluate.

### 7.3 Loss Function

**Pure data-fitting (MSE):**
```python
loss = F.mse_loss(y_pred, y_true)
```

**Optional physics constraints (PINN approach):**
```python
# Friction ellipse constraint — penalize violations
friction_violation = F.relu(Fx_pred**2 + Fy_pred**2 - (mu * Fz)**2)
loss_phys = friction_violation.mean()

# Total loss
loss = loss_mse + lambda_phys * loss_phys
```

The physics constraint is a *soft* penalty (not a hard constraint). It guides training to stay physical but does not guarantee it. Start with pure MSE; add physics penalties if the trained network shows violations at test time.

### 7.4 Training Process

```python
# Training loop (simplified)
optimizer = torch.optim.Adam(net.parameters(), lr=1e-3)
scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=1000)

for epoch in range(2000):
    for X_batch, Y_batch in dataloader:
        optimizer.zero_grad()
        Y_pred = net(X_batch)               # normalized inputs/outputs
        loss   = F.mse_loss(Y_pred, Y_batch)
        loss.backward()
        optimizer.step()
    scheduler.step()
```

**Validation:** Hold out 20% of data. Monitor validation MSE — stop when it stops decreasing (early stopping). Typical training time: <5 minutes on a laptop GPU for 160K samples.

### 7.5 Why This Is System Identification, Not Pure AI

The term "Physics-Informed Neural Network" can be misleading. This is better understood as **data-driven system identification with physical priors**:

- The dataset comes from a physics model (Pacejka), not from "big data".
- The network learns to reproduce known physics, not discover unknown patterns.
- Physical constraints (friction ellipse) are explicitly encoded.
- The model is deterministic and operates in a well-understood domain.

The "intelligence" is not in the network weights — it is in the quality of the Pacejka model that generated the training data, and in the physical constraints enforced during training. A well-trained network is a fast, differentiable approximation of Pacejka — useful for C++ deployment and future gradient-based optimization, not for discovering new physics.

---

## 8. Integration into the Simulation

### 8.1 Replacing Pacejka with Network at Runtime

The `TireModel` class is designed for this substitution without changing any caller code. The hot-path call is:

```python
# Current (Pacejka):
Fx, Fy, Mz = self._pinn.predict_combined(Fz, alpha_prime, kappa, gamma)

# Future (trained NN — same interface):
x_norm = (torch.tensor([Fz, alpha_prime, kappa, gamma]) - X_mean) / X_std
y_norm = self.net(x_norm)
Fx, Fy, Mz = (y_norm * Y_std + Y_mean).tolist()
```

The `TireModel.compute_forces()` signature is frozen — no callers need changing.

### 8.2 Runtime Constraints

The physics loop runs at **1 kHz** (1 ms budget per step). All tire computations must complete in under 0.5 ms (leaving half the budget for PyChrono integration and I/O).

**Current Pacejka (Python math):** ~2 µs per tire call. Well within budget.

**PyTorch inference:** A 4-64-64-32-3 network forward pass on CPU: ~50 µs in Python. Too slow for 1 kHz.

**Solutions for production:**
1. **ONNX Runtime** — export trained model to `.onnx`, run with `onnxruntime`. Typical: 5–15 µs.
2. **LibTorch (C++ PyTorch)** — export to TorchScript, call from C++. Typical: 3–10 µs.
3. **Batching** — note that both tires are evaluated each step. Batch them as a (2, 4) input tensor → single forward pass. Halves overhead per tire.
4. **Quantization** — reduce to FP16 or INT8 weights. Usually 2–4× speedup with <1% accuracy loss for this function class.

The analytical Pacejka (current code) will remain as the fallback if the NN violates physical limits at runtime.

### 8.3 What Does Not Change

- `TireModel.compute_forces()` signature
- `TireModel.compute_forces()` calling convention (per-step, stateful through α')
- `motorcycle.py` step loop
- Relaxation logic (ZOH is independent of the force model)
- All telemetry keys and dashboard panels

---

## 9. Current Limitations (Honest Assessment)

### 9.1 No Real Rider Model

The rider is a rigid body fused to the main frame. Real rider effects ignored:
- **Upper body lean** (most important for balance at low speed)
- **Body weight shift under braking/acceleration** (affects Fz distribution)
- **Head/neck dynamics** (affects perception latency)
- **Leg bracing** (changes effective frame stiffness)

The active roll controller (`K_roll`, `C_roll`) is a **surrogate** for rider lean. It is a proportional-derivative controller, not a biomechanical rider model. It stabilizes the simulation but does not represent how a human actually leans.

### 9.2 Limited Suspension Dynamics (M3 In Progress)

M3 adds the TSDA spring-damper, which is a significant improvement. Still missing:
- **Nonlinear spring rate** (real motorcycle springs are progressive)
- **Bump stops** (hard limit on compression)
- **Stiction and seal friction** in the fork
- **Swingarm kinematics effect on chain tension** (anti-squat geometry)
- **Front fork bending stiffness** (torsional compliance of the fork legs)
- **Tire heading correction** pending in M3 — the front wheel heading should include the steer angle projected onto the road plane correctly; current approximation uses `_psi_front = yaw + steer_angle * cos(rake)`.

### 9.3 No Real Tire Data

All tire behavior comes from a single `medium.yaml` Pacejka coefficient set estimated from literature. Missing:
- Load sensitivity variation (Fz² scaling of cornering stiffness)
- Temperature effects (Milestone 5 placeholder exists in `TireState`)
- Wear effects (also M5 placeholder)
- Speed effects (centrifugal growth at very high speed)
- Pressure effects

The PINN, even when trained, will only be as good as the Pacejka model that generated its training data. It will not "discover" behavior that Pacejka does not model.

### 9.4 Flat Road Only

The road model returns `height=0, normal=(0,0,1)` everywhere. A real motorcycle over a bump or pothole will see:
- Instantaneous Fz spike when the wheel hits a step
- Jounce/rebound oscillation afterward
- Cross-slope load transfer if the road is banked

The `RoadSample` dataclass already has `height` and `normal` fields — the interface is ready for Milestone 4.

### 9.5 Model Assumptions

- **Rigid chassis** — no frame flex. Real frames flex ~1–2 mm under load, which matters for high-frequency chatter.
- **2D camber** — camber is approximated as equal to main frame roll. Real camber varies with steer angle (Ackermann geometry) and suspension deflection.
- **No aerodynamics** — no drag, no lift, no fork-steer aerodynamic moments. Significant above 100 km/h.
- **No chain/belt drivetrain** — drive torque is applied directly at the rear axle, ignoring chain tension and anti-squat effects.

---

## 10. Roadmap Forward

### 10.1 Finish M3 — Complete Suspension Physics

Immediate items:
1. **Tire heading correction** — Fix the front tire heading calculation to correctly project the steer angle. Currently `_psi_front = yaw + steer_angle * cos(rake)` is an approximation. The exact calculation requires projecting the steer axis onto the road plane.
2. **Validate suspension telemetry** — The `SuspensionPanel` dashboard shows `fork_travel` and `rear_travel`. Verify these match expected static compression at design ride height.
3. **Add M3 gate tests** — Write validation tests for: (a) correct static travel at design load, (b) fork compresses under braking, (c) rear compresses under acceleration.

### 10.2 Train the PINN

Once M3 is passing:
1. Write `rig/generate_dataset.py` using the pipeline from Section 6.5.
2. Generate ~200K samples with the sampling strategy from Section 6.4.
3. Train the network (Section 7.3–7.4).
4. Validate against held-out Pacejka data and verify friction ellipse.
5. Swap into `TireModel` via a new `NeuralTireModel` subclass.
6. Re-run all M0–M3 tests to confirm no regression.

### 10.3 Add Rider Model

Options in order of complexity:
1. **PD lean controller** (current): extend with speed-dependent gain scheduling.
2. **Neuromuscular rider (Sharp 2004)**: explicit torso, arm, and foot DOFs. More accurate but requires many parameters.
3. **Learned rider**: train a neural net rider agent in the same simulation environment. Longest-term goal.

### 10.4 Validate Against Literature (Sharp 1971)

Sharp's 1971 paper gives linearized eigenvalues for the weave, wobble, and capsize modes as a function of speed for a benchmark bicycle. A numerical simulation should reproduce the qualitative trends:
- Weave: complex eigenvalue, imaginary part (frequency) increasing with speed.
- Wobble: higher-frequency oscillation (≈10 Hz), lightly damped.
- Capsize: real eigenvalue, crosses from stable to unstable at low speed.

The M2 G5/G6 tests partially validate weave. A proper eigenvalue comparison requires linearizing the equations of motion numerically (perturb each DOF, measure response) and comparing to Sharp's Table 1.

### 10.5 Summary Timeline

| Milestone | Status | Key Deliverable |
|-----------|--------|----------------|
| M0 — Tire model | COMPLETE | Pacejka MF + combined slip + relaxation |
| M1 — PyChrono multibody | COMPLETE | 8/8 gate tests pass |
| M2 — Weight transfer + eigenvalues | COMPLETE | 7/7 gate tests pass |
| M3 — Full suspension | IN PROGRESS | TSDA added; heading fix pending |
| M4 — Track road model | NOT STARTED | Curved road with banking |
| M5 — Thermal/wear model | NOT STARTED | TireState fields already defined |
| PINN-1 — Dataset generation | NOT STARTED | 200K synthetic samples |
| PINN-2 — Network training | NOT STARTED | Trained model, ONNX export |
| PINN-3 — C++ integration | NOT STARTED | LibTorch/ONNX in production loop |

---

## Quick Reference: Key Numbers

| Parameter | Value | Why It Matters |
|-----------|-------|---------------|
| Physics timestep dt | 1 ms | 1 kHz = real-time at 1× speed |
| Telemetry rate | 50 Hz | 20 ms latency to dashboard |
| Relaxation length σ (lateral) | 0.30 m | Half contact patch length |
| Relaxation length σ (longitudinal) | 0.30 m | Same order of magnitude |
| Cornering stiffness (Fy₀ slope) | ≈8800 N/rad | For CBR600RR at 1470 N load |
| Peak lateral force | ≈1320 N | μ·Fz = 0.90 × 1470 |
| Peak slip angle | ≈4–8° | From `peak_slip_angle_deg()` |
| Trail reversal angle | ≈8.3° | Where Mz switches sign |
| Roll stabilization K_roll | 6000 N·m/rad | >> m·g·h = 1075 for stability |
| CBR600RR total mass | 189 kg | Including rider |
| CBR600RR wheelbase | 1.37 m | Rear axle to front axle |
| Rake angle | 24° | 0.419 rad |
| Geometric trail | 0.097 m | 97 mm |
| Front spring rate | 18,500 N/m | Standard sport-touring |
| Rear spring rate | 42,000 N/m | Stiffer (single shock) |
