# Decision 006 — Front-Tyre Steer-Sign Inversion, and Removal of `K_Mz_missing` and `K_yaw_damp`

**Date**: 2026-08-08
**Status**: Decided, applied
**Files affected**:
`core/chassis/motorcycle.py` (`steer_kin_sign`, `_psi_front`, `K_yaw_damp`),
`core/chassis/whipple_stability.py` (`K_Mz_missing` default),
`tests/motorcycle/test_m1_validation.py` (G6),
`tests/motorcycle/test_m2_validation.py` (G8)
**Evidence**: `validation/probes/` — eight reproducible scripts + raw JSON, and
`validation/probes/STEER_SIGN_AUDIT.md`

---

## TL;DR — if you ever need this again

The one-line fix is `Motorcycle.steer_kin_sign = -1.0`, used in

```python
_psi_front = yaw + self.steer_kin_sign * _steer_ang_raw * math.cos(p.rake)
```

`ChLinkMotorRotationTorque.GetMotorAngle()` returns the **negative** of the true
geometric steer angle. Feeding the raw motor angle into the front-tyre heading
evaluates the tyre at a slip angle of the wrong sign.

To reproduce the old behaviour exactly, set `steer_kin_sign = +1.0`,
`WhippleStabilityModel(K_Mz_missing=2000.0)` and `Motorcycle.K_yaw_damp = 2000.0`.
All three remain supported parameters; nothing was deleted.

---

## Context

`WhippleStabilityModel` carried a hard-coded constant

```python
K_Mz_missing = 2000.0   # N·m/rad
```

justified in comments and in a draft paper as *constraint compensation*: the
belief that PyChrono's constraint chain

```
front_wheel → front_axle (revolute) → fork_lower → prismatic + TSDA
            → fork_upper → steer motor → main_frame
```

fails to route (A) the Pacejka aligning moment `Mz` to the steering axis, and
(B) the front lateral force `Fy` into main-frame roll.

Neither belief survived measurement.

## Method

Inject a known generalised force at the front contact patch, measure the
response of the target DOF, and divide by the response to the *same* generalised
force applied directly to that DOF:

```
eta = (q_injected − q_null) / (q_reference − q_null)
```

All three cases share an identical settled state, timestepper and supplement
configuration, so the ratio contains no tuning parameter.
Script: `validation/probes/s1_routing_probe.py`.

## What we found

**1. Both claimed failures are false.**

| path | transmission ratio |
|---|---|
| aligning moment → steer axis | **\|η\| = 0.96** |
| lateral force → main-frame roll | **η = 0.69** |

Neither is zero. The first is what constraint theory requires: a revolute
transmits every moment component except the one about its own axis, and the
front axle's free axis is the wheel *spin* axis. The 31 % deficit in the second
is genuine physics — the contact patch is offset from the steer axis by the
mechanical trail, so part of the lateral impulse accelerates the free steer DOF
instead of rolling the chassis.

**2. The real defect was a sign inversion.** Measured geometrically from body
orientations (`s1b_sign_check.py`), a +10 N·m steer-motor torque held 0.2 s:

| quantity | value |
|---|---|
| `GetMotorAngle()` | **+30.954°** |
| true geometric steer (from wheel spin-axis orientation) | **−28.317°** |
| `GetMotorAngle() · cos(rake)` | +28.277° |

Exact magnitude match, exact sign inversion. Confirmed in an ordinary run
(`s1c_psi_front_audit.py`): Pearson correlation **−0.87**, median ratio −0.81.

Supporting checks, all passing, which localise the fault to that one line:
the steer axis in world coordinates is (−0.411, 0, +0.912) with vertical
component ≈ cos(rake); a positive moment about world +Z turns the wheel
geometrically left; and the Pacejka model is properly self-aligning (Mz opposes
slip angle at α = 1°, 2°, 4°, 6°). Tyre model and multibody model are each
internally correct — only the sign of the steer angle passed between them was
wrong.

**3. `K_Mz_missing` was masking it.** Peak geometric steer after a 5° roll
perturbation (`s2_signfix_vs_compensation.py`):

| K_Mz_missing | pre-fix, v=5/6/10 | corrected, v=5/6/10 |
|---|---|---|
| **0** | 73.7° / 66.3° / 54.9° | **1.43° / 1.36° / 0.96°** |
| 500 | 15.1° / 15.6° / 13.3° | 1.35° / 0.76° / 0.65° |
| 1000 | 1.11° / 1.35° / 1.35° | 0.75° / 0.61° / 0.59° |
| 2000 | 2.41° / 0.61° / 0.59° | 0.62° / 0.61° / 0.59° |

With the sign corrected, `K_Mz_missing = 0` already meets the old acceptance
criterion (steer < 1° at v = 10 m/s: measured 0.96°). The 2000 N·m/rad spring
was simply stiff enough to overpower a tyre force pointing the wrong way.

**4. Independent confirmation from steady-turn kinematics**
(`s4_cornering_capability.py`, 3° commanded steer at 10 m/s, against
`ψ̇ = v·tan(δ)·cos(ε)/L`):

| configuration | simulated | kinematic | error |
|---|---|---|---|
| pre-fix, K_Mz=2000, K_yaw_damp=2000 | +10.22 °/s | −31.79 °/s | **wrong sign** |
| pre-fix, K_Mz=0 | diverges (NaN) | — | — |
| corrected, K_Mz=2000, K_yaw_damp=0 | −12.918 °/s | −12.855 °/s | +0.5 % |
| **corrected, K_Mz=0, K_yaw_damp=0** (shipped) | **−2.689 °/s** | **−2.754 °/s** | **−2.4 %** |

The pre-fix model yawed opposite to its own front wheel.

## Decision

1. `steer_kin_sign` defaults to **−1.0**. `+1.0` reproduces pre-fix behaviour.
2. `K_Mz_missing` defaults to **0.0**. The constructor argument is retained.
3. `centripetal_factor` and `C_roll` were promoted from hard-coded literals to
   constructor arguments (defaults unchanged: 2.5 and 200.0) so the probes can
   sweep them.
4. `K_yaw_damp` defaults to **0.0** (was 2000.0). See below.

## Addendum — `K_yaw_damp` was a third symptom of the same bug

`K_yaw_damp` applied `-K·ω_z` to the main frame at 2000 N·m·s/rad against a
frame yaw inertia of Izz = 50 kg·m², a 40:1 override of the vehicle's own yaw
dynamics. Its justification was the same disproved premise — "PyChrono's
constraint chain does not route steer reaction to yaw on main_frame" — and its
value was tuned against the G6 gate that turned out to be inverted.

`validation/probes/s7_yaw_damping.py` ran 30 s of straight-line simulation at
5/10/20 m/s across `K_yaw_damp` ∈ {0, 50, 200, 500, 2000}, fitting the yaw-rate
envelope to detect the "~14 rad/s eigenvalue" the old comment described.

**Stability.** With `steer_kin_sign = -1.0` the model is yaw-stable at
`K_yaw_damp = 0`: peak yaw rate over 30 s is 0.0011 °/s at 5 and 10 m/s and
0.0060 °/s at 20 m/s. No instability eigenvalue appears. A small envelope growth
at 20 m/s (+0.14 /s) is present at *every* damping value including 2000
(+0.16 /s), so the damper does not address it — it is envelope-fit noise on a
signal of thousandths of a degree per second, not a physical mode.

Head-to-head over 30 s, both configurations are yaw-stable in straight running:

| configuration | peak \|ω_z\|, v=10 | peak \|ω_z\|, v=20 |
|---|---|---|
| original (sign=+1, K_Mz=2000, K_yaw=2000) | 0.0008 °/s | 0.0032 °/s |
| new (sign=−1, K_Mz=0, K_yaw=0) | 0.0011 °/s | 0.0060 °/s |

So the damper was not *needed*; removing it costs no straight-line stability.

**Fidelity.** What it did cost was steady-turn accuracy against
`ψ̇ = v·tan(δ)·cos(ε)/L` (3° steer at 10 m/s, corrected kinematics):

| `K_yaw_damp` | 0 | 50 | 200 | 500 | 2000 |
|---|---|---|---|---|---|
| turn error | **−2.4 %** | −6.6 % | −17.1 % | −32.4 % | −64.9 % |

Monotonic. Hence 0.0.

**Caveat if re-enabled.** The torque is applied about **world Z**, unlike
`K_pitch_damp` which correctly uses the body Y axis. On a leaned bike that also
bleeds off part of the lean rate — almost certainly the "couples back into roll
and worsens G4 roll" effect the original comment noted. Fix the axis before
using a non-zero value. The term is now guarded by `if self.K_yaw_damp:`.

## Consequences

**Two gates were corrected**, because they encoded the pre-fix behaviour:

*G6* (`test_g6_countersteering` → `test_g6_steer_yaw_sign_consistency`). Its
stated reasoning was wrong on two counts, and it passed only because the errors
cancelled:

- It claimed "left turn = negative yaw rate when X is forward and Y is left".
  In that right-handed frame a positive rotation about +Z carries X toward Y,
  toward the left — a left turn is a **positive** yaw rate.
- It claimed "+torque steers wheel right". Measured, rider +2 N·m points the
  wheel **left** (+0.15°); `motorcycle.py` negates rider torque into the motor.

The pre-fix model satisfied it by pointing the wheel 23.9° left while yawing
15.7 °/s right — kinematically impossible for a rolling wheel. The gate now
asserts that steer direction and yaw direction agree.

*G8* (`test_g8_capsize_low_speed`). Previously required ≥0.8° of roll growth at
v = 0.5 m/s, a margin tuned to the pre-fix model. The capsize mode survives the
fix but is weaker. `s6_capsize_sweep.py`, Δ|roll| after 1.5 s from a 5°
perturbation:

| v [m/s] | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.5 |
|---|---|---|---|---|---|---|---|---|---|
| pre-fix | +2.79 | +3.38 | +1.69 | +0.65 | −4.90 | **+1.61** | −1.71 | −3.62 | −4.96 |
| shipped | +2.43 | +0.65 | +0.68 | +0.56 | −1.49 | −2.17 | −2.46 | −3.14 | −4.82 |

Both transition at 0.7 m/s, but the shipped model is monotone where the pre-fix
model flip-flopped back to CAPSIZE at 0.8 m/s (bold). The gate now asserts the
physical *ordering* — roll grows at 0.3 m/s, decays at 1.5 m/s — which is the
actual claim of Sharp's capsize mode and is robust to tuning.

**Unaffected:** all four published-result validation anchors, re-run after the
fix with identical results (Whipple/Meijaard critical speeds to 0.02 %,
longitudinal closed-form benchmarks, Foale sag-frequency correlation, road
suite). They exercise longitudinal and vertical dynamics, which the steer sign
does not touch. Test suite: 103 passed, 1 failed — the failure being the
pre-existing, unrelated `TestMzDrift` tyre defect (0.09316 N·m/s, byte-identical
before and after).

## Known limitations left open

- ~~`K_yaw_damp = 2000`~~ — RESOLVED, now 0.0. See the addendum above.
- **The supplement has no steer→roll coupling.** `tau_roll` contains no
  steer-angle term; `K_centripetal_eff` is folded onto the *diagonal* of the
  roll equation by the quasi-static elimination `δ_eq = (K_lean/K_ctr_phys)·φ`.
  It is a 1-DOF roll oscillator, not a 2-DOF Whipple weave mode, and should not
  be described as restoring Meijaard's off-diagonal K₂ term. Consequently the
  model does not reproduce the countersteer transient.
- **`TestMzDrift`** fails on a clean tree: Mz drifts 0.0932 N·m/s at a fixed 5°
  slip angle against a 0.01 N·m/s threshold (~5.6 N·m over 60 s). Independent of
  everything above, but it is a defect in the aligning-moment path.

---

## OPEN — `tau_roll` steer coupling (analysis done, redesign NOT started)

Paused 2026-08-08 at the user's request. No code for this was written; the three
fixes above are complete and verified. Evidence:
`validation/probes/s8_roll_coupling.py` → `data/s8_roll_coupling.json`.

**The diagonal fudge is not needed for balance.** With `centripetal_factor = 0`
the bike stays upright after a 5° perturbation at every speed tested (settles to
0.39° at v=5, 0.44° at v=10, 0.55° at v=20). So `K_centripetal_eff` can be
deleted without losing straight-line stability.

**But it is load-bearing for steer response, and removing it exposes the real
problem.** Holding a commanded steer with `centripetal_factor = 0`:

| v [m/s] | true steer | roll | yaw rate |
|---|---|---|---|
| 5 | +0.077° | −8.37° | +0.31 °/s |
| 10 | +0.031° | −11.89° | +0.40 °/s |
| 15 | −0.005° | −16.71° | +0.52 °/s |

The machine leans 12° while yawing 0.4 °/s. A real motorcycle leaning 12° at
10 m/s corners at roughly 17 m radius, about 33 °/s. This one leans over
*without turning*, held up by tyre camber thrust, and the steer DOF barely moves
(0.03°) against a 3° command.

**Diagnosis.** The artificial steer-centering spring pins the steer DOF, so lean
cannot become steer and steer cannot become turn. `tau_roll`'s missing steer term
is a symptom, not the root cause. Adding an off-diagonal term to `tau_roll` while
the steer DOF stays pinned would be a fourth fudge — different numbers, same
missing physics. This is the same architectural conclusion reached independently
on 2026-06-29 ("model can't sustain a turn"): it is a straight-line stability
model, not a cornering model.

**Agreed plan (full redesign, when resumed).** A motorcycle balances *by
steering*, not by a roll spring:

1. Replace the `tau_roll` spring with a rider controller that commands steer
   torque from roll error and roll rate (steer into the fall).
2. Unpin the steer DOF — drop the artificial centering, keep only physical trail
   and gyroscopic terms.
3. Delete `K_centripetal_eff` and its fitted 2.5 factor; let PyChrono supply the
   centripetal roll path it already routes at η_B = 0.69.
4. Move `steer_target` to the geometric convention. It is currently compared
   against the *motor* angle, so a positive command points the wheel right — a
   leftover from the sign fix above.
5. Re-verify all 104 tests, the four published-result anchors, and steady-turn
   kinematics; check that countersteering emerges rather than being asserted.

Open risk, undecided: whether this lands as the default or behind a flag
(e.g. `rider_balance_mode='steering'`), since the Electron dashboard drives this
model. Decide before merging.
