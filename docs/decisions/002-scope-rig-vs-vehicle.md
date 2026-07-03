# Decision 002 — Product Scope: Full Motorcycle Multibody Model

**Date**: Milestone 0  
**Status**: Decided  
**Files affected**: `core/chassis/motorcycle.py`, all of Milestone 1

---

## Context

The prototype (`tire_rig.py`) was a kinematic single-tire test rig: velocities
[vx, vy] prescribed, Fz fixed, no chassis frame, no yaw or heading dynamics.
The question was whether the product should remain a rig or evolve to a full
vehicle model.

## Decision

**Full multibody motorcycle model**, starting Milestone 1.

Rationale:
1. **Product positioning**: VI-BikeRealTime competes with VI-CarRealTime,
   CarSim, etc.  A single-tire rig cannot simulate countersteering, weave,
   wobble, capsize — the phenomena customers actually want to see.
2. **PyChrono is already installed**: the heavy DAE solver dependency is paid.
   Not using it for chassis dynamics is waste.
3. **Tire rig retained**: `rig/tire_rig.py` remains as an internal
   characterisation tool.  The product and the rig share the same
   `core/tire/pinn_model.py` — no duplication.

## What the rig still does

- Pacejka coefficient identification sweeps (α, κ, γ, Fz grids)
- Relaxation length measurement at multiple speeds
- Combined-slip surface mapping
- Thermal model calibration (Milestone 5)

These are run offline; results feed into the YAML compound files.

## Fz decision

Fz in the rig remains fixed (1470 N, prescribed).  In the motorcycle, Fz
is computed from multibody contact forces — load transfer is a natural
consequence of the chassis dynamics.  No explicit load-transfer formula is
hand-coded (PyChrono solves it).
