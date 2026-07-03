# Decision 003 — tire_interface.py Stable API Design

**Date**: Milestone 0  
**Status**: Decided  
**Files affected**: `core/tire/tire_interface.py`

---

## Context

Both the tire rig and the motorcycle need to evaluate tire forces.  Two options:

A. Each caller imports `DummyTirePINN` directly and manages relaxation itself.
B. A thin adapter (`TireModel`) wraps PINN + relaxation and exposes a single
   `compute_forces()` call.

## Decision

**Option B**: `tire_interface.TireModel` is the sole public API.

```python
model = TireModel.from_yaml("pacejka_coeffs/medium.yaml")
Fx, Fy, Mz = model.compute_forces(alpha, kappa, Fz, gamma, vx, dt)
```

Callers never import `DummyTirePINN` directly.

## API contract (frozen)

- `compute_forces(alpha, kappa, Fz, gamma, vx, dt) → (Fx, Fy, Mz)`
- `model.alpha_prime` — read-only current relaxed slip
- `model.reset(state)` — reset between episodes
- `model.query_pure(Fz, alpha, camber)` — pure-slip query, no state advance
- `TireModel.from_yaml(path)` — load from compound YAML

## Milestone 5 extension (pre-planned)

`TireState` will gain `T_tread, T_carcass, wear, pressure` fields with defaults
that reproduce the Milestone 0 behaviour exactly.  `compute_forces()` signature
is unchanged — new thermal/wear outputs available via `model.state`.

## Consequences

- `DummyTirePINN` is now an implementation detail.  Tests validate behaviour
  through `TireModel`, not the PINN directly (exception: coefficient-level
  sanity checks in `TestPhysicsSanity`).
- The motorcycle simulator will own two `TireModel` instances (front, rear)
  per simulation run.  Thread-safety note: do NOT share a `TireModel` across
  threads — each wheel owns one instance.
