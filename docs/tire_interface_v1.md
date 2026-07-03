# tire_interface API — Version 1.0

**File**: `core/tire/tire_interface.py`  
**Frozen**: Milestone 0  
**Revision policy**: This document defines the v1 contract.  Breaking changes
require a new version (v2) and a migration note.  Additive changes (new read-only
properties, new factory classmethods) may be added without a version bump.

---

## Overview

`TireModel` is the sole public API for tire force evaluation in the moto_sim
product.  Both the tire rig and the motorcycle simulator use this class.
`DummyTirePINN` is an implementation detail and must not be imported by callers.

One `TireModel` instance per physical tire.  Instances are NOT thread-safe
across wheels; each wheel must own its own instance.

---

## Factory Methods

### `TireModel.from_yaml(path: str | Path) → TireModel`

Load a `TireModel` from a compound YAML file.

```python
model = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
```

| Argument | Type | Description |
|----------|------|-------------|
| `path` | `str \| Path` | Path to a compound YAML file (soft/medium/hard or custom). |

Raises `FileNotFoundError` if path is missing.  Raises `ValueError` if a
required YAML key is absent.

---

### `TireModel.from_coeffs(coeffs: TireCoeffs) → TireModel`

Construct from an already-loaded `TireCoeffs` NamedTuple.  Useful in tests.

---

## State Management

### `model.reset(state: TireState | None = None) → None`

Reset the tire to an initial state.  **Must be called before each simulation
episode** to clear α' from the previous run.

| Argument | Default | Description |
|----------|---------|-------------|
| `state` | `None` | If None, resets to `TireState()` (α'=0, cold temperatures, zero wear). |

---

## Core API — compute_forces()

```python
Fx, Fy, Mz = model.compute_forces(
    alpha: float,
    kappa: float,
    Fz:    float,
    gamma: float,
    vx:    float,
    dt:    float,
)
```

**Signature is frozen for v1.** Do not add positional arguments.  Optional
keyword-only additions are allowed without a version bump.

### Arguments

| Argument | Units | Sign convention | Typical range |
|----------|-------|----------------|---------------|
| `alpha` | rad | Kinematic slip angle in the **tire frame**. `atan2(vy_tire, vx_tire)`. Positive = left turn (velocity vector angled to the left of wheel heading). | ±0.35 rad (±20°) |
| `kappa` | — | Longitudinal slip ratio. `(R·ω − vx) / (|vx| + ε)`. Positive = traction (wheel spinning faster than ground speed). Negative = braking. Clamped to [−1, +1]. | ±0.3 typical, ±1.0 extreme |
| `Fz` | N | Normal load. **Positive = compression** (road pushes wheel up). Must be > 0; behaviour undefined at Fz ≤ 0. | 500–3000 N |
| `gamma` | rad | Camber angle. Angle between wheel plane and vertical. Positive = wheel leans to the right (SAE sign convention). | ±0.52 rad (±30°) for motorcycle |
| `vx` | m/s | Longitudinal speed at the contact patch in the tire frame. Used only to compute the relaxation time constant τ = σ/vx. Clamped internally to ≥ 1e-3 m/s. | 0.5–80 m/s |
| `dt` | s | Timestep for the ZOH relaxation update. Must be > 0. Should equal the physics loop timestep. | 0.001 s (1 kHz) |

### Return Values

| Value | Units | Sign convention |
|-------|-------|----------------|
| `Fx` | N | Longitudinal tire force in the tire frame. Positive = forward (traction). Negative = rearward (braking). |
| `Fy` | N | Lateral tire force in the tire frame. Positive = left (same direction as positive α). Negative = right. |
| `Mz` | N·m | Self-aligning torque about the contact patch vertical axis. **Negative = stabilising** (tends to reduce slip angle). Positive = destabilising (past pneumatic trail reversal). |

### Semantics

1. **Relaxation advance**: Each call advances the internal ZOH relaxation ODE:
   ```
   α'[k+1] = α + (α'[k] − α) · exp(−|vx| · dt / σ)
   ```
   The returned `(Fx, Fy, Mz)` are computed at the updated `α'[k+1]`.

2. **Combined slip**: Fy is weighted by `Gyκ = cos(Cyκ · atan(Byκ · κ))`.
   Fx is weighted by `Gxα = cos(Cxα · atan(Bxα · α'))`.  Both G-factors equal
   1.0 at their respective zero inputs.  The coupling is C∞ smooth.

3. **Mz consistency**: Mz uses the combined Fy (not pure-slip Fy_pac), so Mz
   and Fy are always physically consistent regardless of κ.

4. **Stateful**: The call modifies `model._state.alpha_prime`.  Calls at the
   same (α, κ, Fz, γ, vx, dt) with different call history may return different
   results due to the running α'.

5. **Pure math, zero torch allocation**: The hot path (`_predict_scalar`) uses
   only `math.*` functions.  No heap allocation beyond the Python call stack.

### Performance (measured, Milestone 0)

| Path | mean | p99 |
|------|------|-----|
| Cold (first call, new instance) | 260 µs | 384 µs |
| Warm (sustained, same instance) | **1.4 µs** | 2.4 µs |
| Two tires per step at 1 kHz | 2.8 µs | — |
| Remaining budget for PyChrono | **997 µs** | — |
| Bytes allocated per warm call | 12 bytes | — |

Cold cost is one-time at episode start (model construction + YAML parse).
Warm cost is what matters for real-time.

---

## Convenience Query — query_pure()

```python
Fy, Mz = model.query_pure(Fz, alpha, camber=0.0)
```

Evaluates pure-slip (Fy, Mz) **without advancing state**.  Uses the torch path
(supports autograd).  Intended for tire characterisation sweeps and validation
tests; not for the 1 kHz hot path.

---

## Read-Only Properties

| Property | Units | Description |
|----------|-------|-------------|
| `model.alpha_prime` | rad | Current relaxed slip angle α' after the last `compute_forces()` call. |
| `model.state` | `TireState` | Full state snapshot (copy). Contains `alpha_prime`, and M5 thermal/wear fields. |
| `model.peak_slip_angle_deg(Fz)` | ° | Slip angle at maximum Fy (pure slip, γ=0). Numerically scanned over [0°, 30°]. |
| `model.cornering_stiffness(Fz)` | N/rad | `dFy/dα` at α=0 (analytically = B·C·D). |
| `model.trail_reversal_deg()` | ° | Slip angle at which pneumatic trail crosses zero. `inf` if Ct ≤ 1. |

---

## TireState (Milestone 0 fields)

```python
@dataclass
class TireState:
    alpha_prime: float = 0.0    # relaxed slip angle [rad]
    T_tread:     float = 20.0   # [°C]  inactive until M5
    T_carcass:   float = 20.0   # [°C]  inactive until M5
    wear:        float = 0.0    # [0=new, 1=bald]  inactive until M5
    pressure:    float = 230.0  # [kPa]  inactive until M5
```

Milestone 5 will activate the thermal/wear fields and wire them into the
Pacejka μ scaling.  The `compute_forces()` signature does not change.

---

## Versioning Policy

| Change type | Action required |
|-------------|----------------|
| Add keyword-only argument to `compute_forces()` | Additive — no version bump. Existing callers unaffected. |
| Add new property or classmethod | Additive — no version bump. |
| Change argument units, sign convention, or semantics | **Breaking** — requires `tire_interface_v2.md` and migration guide. |
| Change return value sign convention | **Breaking** |
| Remove method or argument | **Breaking** |
| Change `TireState` field defaults in a way that alters hot-path output | **Breaking** |

---

## Usage Examples

### Rig (single tire, prescribed vx/vy)

```python
from core.tire.tire_interface import TireModel

model = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
model.reset()

for step in simulation_steps:
    alpha = math.atan2(vy, max(abs(vx), 0.01))
    kappa = (R * omega - vx) / (abs(vx) + 1e-3)
    Fx, Fy, Mz = model.compute_forces(alpha, kappa, Fz, gamma, vx, DT)
```

### Motorcycle (two tires, Fz from multibody)

```python
front_tire = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
rear_tire  = TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
front_tire.reset()
rear_tire.reset()

# each PyChrono step:
Fz_f, Fz_r = read_contact_forces_from_pychrono()
Fx_f, Fy_f, Mz_f = front_tire.compute_forces(alpha_f, kappa_f, Fz_f, gamma_f, vx_f, DT)
Fx_r, Fy_r, Mz_r = rear_tire.compute_forces(alpha_r, kappa_r, Fz_r, gamma_r, vx_r, DT)
apply_to_pychrono_wheel_bodies(...)
```
