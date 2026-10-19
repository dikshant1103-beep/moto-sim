# Steer-sign audit of the PyChrono motorcycle model

**Date:** 2026-08-08
**Status:** FIXED — see `docs/decisions/006-steer-sign-inversion.md`
**Scope:** verify the two claimed constraint force-routing failures.
**Verdict:** neither exists. The real defect is a sign inversion in the
front-tyre slip kinematics; the `K_Mz_missing` spring and the `K_yaw_damp`
yaw damper were both masking it. All three are now fixed.

All numbers below are reproduced by the scripts in this directory, which write
their raw output to `validation/probes/data/`. Run any of them directly:

```bash
cd moto_sim
python validation/probes/s1_routing_probe.py          # transmission ratios
python validation/probes/s1b_sign_check.py            # geometric sign audit
python validation/probes/s1c_psi_front_audit.py       # sign audit in a normal run
python validation/probes/s2_signfix_vs_compensation.py  # 2x6x3 fix-vs-spring matrix
python validation/probes/s3_countersteer.py           # steer-pulse response
python validation/probes/s4_cornering_capability.py   # steady-turn vs kinematic law
python validation/probes/s5_countersteer_truth.py     # G6 gate sign audit
python validation/probes/s6_capsize_sweep.py          # capsize sweep, pre vs post fix
```

**To reproduce the pre-fix behaviour** (nothing was deleted):
`Motorcycle.steer_kin_sign = +1.0` and `WhippleStabilityModel(K_Mz_missing=2000.0)`.

---

## 1. The method: transmission ratio

The claims were supported only by aggregate simulation behaviour. To
test them directly, inject a known generalised force at the front contact patch,
measure the response of the target degree of freedom, and divide by the response
to the *same* generalised force applied directly to that degree of freedom:

```
eta = (q_injected - q_null) / (q_reference - q_null)
```

`eta ~ 1` means the constraint chain routes the path. `eta ~ 0` is what was
claimed. All three cases share an identical settled state, timestepper and
supplement configuration, so the ratio contains no tuning parameter.

Script: `s1_routing_probe.py` → `data/s1_routing_probe.json`

## 2. Failure A does not exist

> *Claim:* "The front axle revolute joint ... does not propagate the Mz
> component along the steering axis. Consequently, the pneumatic-trail
> self-alignment torque ... is absent from the steer axis."

Measured at v = 10 m/s with a 50 N·m aligning moment:

| quantity | value |
|---|---|
| steer angular acceleration, null | +0.0006 rad/s² |
| steer angular acceleration, Mz injected at contact patch | −194.11 rad/s² |
| steer angular acceleration, equal moment applied at steer motor | +201.19 rad/s² |
| **transmission ratio η_A** | **−0.965** |

The magnitude is 0.96, not 0. **The aligning moment does reach the steering
axis**, at 96 % efficiency; the 4 % deficit is fork and steer-DOF compliance.

This is what constraint theory predicts. A revolute joint constrains five
degrees of freedom and transmits every moment component except the one about
its own axis. The front-axle revolute's free axis is the wheel *spin* axis, so
a moment about the vertical is transmitted to the fork, and onward through the
prismatic joint (which transmits all moments) to the steer axis. There is no
mechanism by which the claimed suppression could occur.

## 3. Failure B does not exist either

> *Claim:* "The prismatic fork topology absorbs this reaction at the
> fork-to-frame connection, and the net moment on the main frame body is zero."

With a 500 N lateral force injected at the front contact patch:

| quantity | value |
|---|---|
| roll angular acceleration, null | −0.0005 rad/s² |
| roll angular acceleration, Fy injected | +17.50 rad/s² |
| roll angular acceleration, equal moment (Fy·h) applied to frame | +25.53 rad/s² |
| **transmission ratio η_B** | **+0.686** |

Not zero. 69 % of the rigid-body estimate arrives at the frame. The 31 %
deficit is genuine physics, not a missing path: the contact patch is offset
from the steer axis by the mechanical trail, so part of the lateral impulse
accelerates the free steer degree of freedom instead of rolling the chassis.

## 4. The real defect: an inverted steer angle in the tyre kinematics

`motorcycle.py` builds the front-wheel heading handed to the Pacejka model as

```python
_steer_ang_raw = self._steer_motor.GetMotorAngle()
_psi_front     = yaw + _steer_ang_raw * math.cos(p.rake)
```

`GetMotorAngle()` returns the **negative** of the true geometric steer angle.

Measured directly from body orientations (`s1b_sign_check.py`), with a
+10 N·m steer-motor torque held for 0.2 s:

| quantity | value |
|---|---|
| `GetMotorAngle()` | **+30.954°** |
| true geometric steer (wheel spin-axis orientation) | **−28.317°** |
| `GetMotorAngle() · cos(rake)` | +28.277° |

Exact magnitude match, exact sign inversion. Confirmed again in an ordinary
unmodified run (`s1c_psi_front_audit.py`, v = 6 m/s, 5° roll
perturbation, 3 s): Pearson correlation between the coded heading offset and
the true heading offset is **−0.87**, median ratio **−0.81**.

Supporting checks, all passing, which localise the fault to this one line:

- Steer axis in world coordinates = (−0.411, 0, +0.912); its component on the
  vertical is 0.912 ≈ cos(rake). Correct.
- A positive moment about world +Z turns the wheel geometrically **left**
  (+24.2°). Physically correct.
- The Pacejka model is properly self-aligning: Mz opposes slip angle at
  α = 1°, 2°, 4°, 6°. Correct.

So the tyre model and the multibody model are each internally right; only the
sign of the steer angle passed between them is wrong. The front tyre is
evaluated at a slip angle derived from a steer angle of the wrong sign, so its
lateral force **amplifies** a steer perturbation instead of opposing it.

## 5. `K_Mz_missing = 2000` is masking that bug

`s2_signfix_vs_compensation.py`, 2 × 6 × 3 matrix, 5° roll
perturbation, 3 s, peak **geometric** steer excursion:

| K_Mz_missing | legacy sign, v=5 / 6 / 10 | corrected sign, v=5 / 6 / 10 |
|---|---|---|
| 0 | 73.7° / 66.3° / 54.9° | **1.43° / 1.36° / 0.96°** |
| 500 | 15.1° / 15.6° / 13.3° | 1.35° / 0.76° / 0.65° |
| 1000 | 1.11° / 1.35° / 1.35° | 0.75° / 0.61° / 0.59° |
| 2000 | 2.41° / 0.61° / 0.59° | 0.62° / 0.61° / 0.59° |
| 3000 | 0.62° / 0.61° / 0.59° | 0.62° / 0.61° / 0.59° |

With the sign corrected, **K_Mz_missing = 0 already satisfies the original
acceptance criterion** ("steer angles below 1° for 5° roll perturbations at
v = 10 m/s": measured 0.96°). The compensation parameter is not compensating a
constraint-topology failure; it is a stiff artificial spring large enough to
overpower a destabilising tyre force of the wrong sign.

## 6. Independent confirmation: steady-turn kinematics

`s4_cornering_capability.py`, v = 10 m/s, 3° commanded steer, steady
state, compared against the single-track kinematic yaw rate
`psi_dot = v·tan(δ)·cos(ε)/L`:

| configuration | simulated yaw rate | kinematic prediction | error |
|---|---|---|---|
| legacy, K_Mz=2000, K_yaw_damp=2000 | **+10.22 °/s** | −31.79 °/s | **wrong sign** |
| legacy, K_Mz=0 | diverges (NaN) | — | — |
| corrected, K_Mz=2000, K_yaw_damp=2000 | −4.76 °/s | −13.64 °/s | −65 % |
| corrected, K_Mz=2000, K_yaw_damp=0 | −12.918 °/s | −12.855 °/s | +0.5 % |
| **corrected, K_Mz=0, K_yaw_damp=0** (shipped) | **−2.689 °/s** | **−2.754 °/s** | **−2.4 %** |

The legacy model yaws in the **opposite direction to its own front wheel**.
The corrected model reproduces the kinematic steady-turn relation to within a
few percent once the artificial yaw damper is removed — −2.4 % in the shipped
configuration, +0.5 % at the larger steer angle the K_Mz=2000 case produces.
The artificial yaw damper alone costs 65 %; see
`docs/decisions/006-steer-sign-inversion.md` for the full K_yaw_damp sweep.

## 7. Two further problems, still open

**(a) The supplement contains no steer→roll coupling.** `K_centripetal_eff` was
presented as restoring "the K₂ off-diagonal term in the Meijaard benchmark".
But the implemented roll torque is

```python
tau_roll = -K_roll_total * sin(roll - lean_delta) - C_roll * roll_rate
```

with `K_roll_total = K_rider + K_centripetal_eff`. There is no steer-angle term.
The coupling has been folded onto the **diagonal** of the roll equation via a
quasi-static elimination `δ_eq = (K_lean/K_ctr_phys)·φ`. An off-diagonal term
cannot be restored by a diagonal one: the result is a one-degree-of-freedom
roll oscillator, not a two-degree-of-freedom weave mode.

**(b) Any weave "validation" against `ω = sqrt(K_net/I_eff)` is circular.** That
1-DOF reference is built from the *same* `K_rider + 2.5·K_centripetal·K_lean/
K_ctr_phys` spring the supplement imposes, including the empirical 2.5 factor,
so it compares the simulator against its own supplement rather than against
Whipple/Sharp theory. The free 2.5 factor was fitted to exactly that.

## 8. Unrelated pre-existing defect

`tests/tire/test_m0_validation.py::TestMzDrift::test_mz_no_linear_drift` fails
on a clean tree (103 of 104 tests pass). Mz drifts at 0.0932 N·m/s at a fixed
5° slip angle against a 0.01 N·m/s threshold — about 5.6 N·m over 60 s. It
touches only `TireModel` and is independent of everything above, but it is a
defect in the aligning-moment path — the same subsystem audited above.

---

## Changes applied

- `core/chassis/motorcycle.py`: `steer_kin_sign` **defaults to −1.0**, applied in
  `_psi_front`. Set `+1.0` to reproduce pre-fix behaviour.
- `core/chassis/whipple_stability.py`: `K_Mz_missing` **defaults to 0.0**;
  `centripetal_factor` (2.5) and `C_roll` (200.0) promoted from hard-coded
  literals to constructor arguments with unchanged defaults.
- Gates G6 and G8 corrected — they encoded the pre-fix behaviour. Full
  reasoning in `docs/decisions/006-steer-sign-inversion.md`.

**Verification after the fix:** test suite 103 passed / 1 failed (the failure is
the pre-existing, unrelated `TestMzDrift`, byte-identical before and after). All
four published-result anchors re-run unchanged — Whipple/Meijaard critical
speeds to 0.02 %, longitudinal closed-form benchmarks, Foale sag-frequency
correlation, road-condition suite.
