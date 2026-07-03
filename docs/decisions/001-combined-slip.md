# Decision 001 — Combined-Slip Formulation: Gxα/Gyκ vs. Post-Hoc Ellipse

**Date**: Milestone 0  
**Status**: Decided  
**Files affected**: `core/tire/pinn_model.py:predict_combined()`

---

## Context

The original `tire_rig.py` prototype computed combined-slip forces as:

1. Compute pure-slip Fx0 = μ·Fz·tanh(Kx·κ)
2. Compute pure-slip Fy0 from Pacejka MF
3. If sqrt(Fx0² + Fy0²) > F_max: scale both by F_max/sqrt(Fx0²+Fy0²)

This is the "post-hoc friction ellipse" approach.  Bug B4 traced to this: the
conditional `if F_vec > F_max` creates a C⁰ discontinuity in dFy/dκ at the
ellipse boundary, producing kinks in the Fy(κ) curve and numerical instability
when Fx and Fy couple through the ellipse.

## Decision

Replace with **Pacejka 2012 §4.3.2 Gxα/Gyκ weighting functions**:

```
Gxα = cos(Cxα · atan(Bxα · α'))    — Fx reduction due to lateral slip
Gyκ = cos(Cyκ · atan(Byκ · κ))     — Fy reduction due to longitudinal slip

Fx = Gxα · Fx0(κ)
Fy = Gyκ · Fy0(α', γ)
```

With Cxα = Cyκ = 1.0, the argument of cos is bounded in (−π/2, π/2) → both
G-factors are always ≥ 0 → no clamp needed → no conditional branches.

## Consequences

- Fy(κ) is now C∞ smooth: validated by `test_fy_no_spikes_across_kappa`
- The full (Fx, Fy) locus in the friction plane is not exactly an ellipse; it
  is a rounded rectangle shape (Gxα and Gyκ independently scale each axis).
  For motorcycle tyres operating well within the friction limit, this is
  indistinguishable from an ellipse.  At peak combined loading the shape
  differs by < 5% from a true ellipse — acceptable for the product's use case.
- Mz is now computed from combined Fy (B3 fix), so Mz and Fy are always
  physically consistent regardless of κ.
- Parameters Bxα, Cxα, Byκ, Cyκ added to all compound YAML files.

## Alternatives Rejected

- **Normalized combined slip (Pacejka §4.3.4)**: uses composite slip
  `σ = sqrt(σx² + σy²)` for more accurate off-peak behaviour but requires
  a different Pacejka curve formulation.  Deferred to a future tire revision
  (Milestone 5+) if PINN training data shows the current formulation is
  insufficient.
- **Post-hoc ellipse with smooth sigmoid boundary**: smoothed the ellipse
  transition but still required an if/else.  Rejected: adds parameters for
  no physical gain.
