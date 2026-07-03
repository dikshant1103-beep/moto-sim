"""tune_gains.py — SPSA (stochastic-gradient) auto-tuner for the bike's steering gains.

PyChrono isn't differentiable, so we can't backprop through it. The honest form of
"SGD" for a black-box simulator is SPSA (Simultaneous Perturbation Stochastic
Approximation): estimate a STOCHASTIC GRADIENT from random ± perturbations of the
gain vector, then take a real gradient-descent step. Repeat.

What it tunes: the heading-controller gains the roam_controller uses —
  Kp (lean_gain), Kd (yaw_damp), steer_gain.
Objective (minimise): a driving-quality loss over headless in-process rollouts —
  heading-tracking error + control oscillation + a divergence (NaN/fall) penalty.

Runs in CONDA python (has pychrono), fully headless, no ROS/Gazebo/WS — fast.

    python tools/tune_gains.py --iters 40
Output: best gains → plug into roam_controller params (lean_gain/yaw_damp/steer_gain).
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np

sys.path.insert(0, ".")
from core.chassis.motorcycle import Motorcycle, BikeParams      # noqa: E402
from core.rider.rider_input import RiderInput, RiderInputSource  # noqa: E402

DT = 0.001
CTRL_HZ = 20
CTRL_EVERY = int(round(1.0 / (DT * CTRL_HZ)))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class ProgrammaticInput(RiderInputSource):
    """Rider input driven by the tuner's control law each control tick."""
    def __init__(self):
        self.lean = 0.0
        self.steer = 0.0
        self.throttle = 0.0

    def get(self, t):
        return RiderInput(throttle=self.throttle, lean_delta=self.lean,
                          steer_target=self.steer).clamp()


def make_bike(rider):
    # All 12 BikeParams._REQUIRED keys (CBR600RR-like).
    bike = {
        "wheelbase": 1.37, "rake": math.radians(24.0), "trail": 0.097,
        "front_wheel_radius": 0.299, "rear_wheel_radius": 0.318,
        "total_mass": 189.0, "cg_height": 0.58, "cg_x": 0.63,
        "front_spring_k": 18500.0, "front_damper_c": 1400.0,
        "rear_spring_k": 24000.0, "rear_damper_c": 2200.0,
    }
    # Motorcycle needs a tire + road; mirror moto_sim_main exactly.
    from core.tire.tire_interface import TireModel
    from core.road.flat_plane import FlatPlane
    def tire():
        return TireModel.from_yaml("core/tire/pacejka_coeffs/medium.yaml")
    return Motorcycle(BikeParams(bike), tire(), tire(), FlatPlane(mu_dry=1.0), rider, dt=DT)


# heading schedule: straight, hard left, hold, hard right, hold (tests turn + settle)
def target_heading(t):
    if t < 2.0:   return 0.0
    if t < 5.0:   return math.radians(60)
    if t < 8.0:   return math.radians(60)
    if t < 11.0:  return math.radians(-30)
    return math.radians(-30)


def rollout(gains, secs=12.0, v0=5.0):
    """Run one headless rollout with the given gains; return scalar loss."""
    Kp, Kd, steer_gain = gains
    lean_sign = 1.0
    lean_max, steer_max, ramp = 0.25, 0.18, 0.6
    rider = ProgrammaticInput()
    bike = make_bike(rider)
    bike.build()
    bike.reset(v0=v0)

    n = int(secs / DT)
    herr2 = 0.0       # integral heading error²
    osc = 0.0         # control oscillation (Δyaw_rate²)
    prev_yr = 0.0
    diverged = False
    samples = 0
    lean_cmd = steer_cmd = 0.0

    for k in range(n):
        t = k * DT
        telem = bike.step(t)
        c = telem["chassis"]
        roll, yaw, yr, vx = c["roll"], c["yaw"], c["yaw_rate"], c["vx"]
        if not (math.isfinite(roll) and math.isfinite(yaw) and math.isfinite(vx)):
            diverged = True
            break
        # control tick
        if k % CTRL_EVERY == 0:
            err = wrap(target_heading(t) - yaw)
            ctrl = Kp * err - Kd * yr
            lean_t = max(-lean_max, min(lean_max, lean_sign * ctrl))
            steer_t = max(-steer_max, min(steer_max, -lean_sign * steer_gain * ctrl))
            dmax = ramp / CTRL_HZ
            lean_cmd += max(-dmax, min(dmax, lean_t - lean_cmd))
            steer_cmd += max(-dmax, min(dmax, steer_t - steer_cmd))
            rider.lean = lean_cmd
            rider.steer = steer_cmd
            rider.throttle = max(0.0, min(0.4, 0.12 + 0.1 * (v0 - vx)))  # hold speed
            # metrics (sampled at control rate after settle window)
            if t > 0.5:
                herr2 += err * err
                osc += (yr - prev_yr) ** 2
                samples += 1
            prev_yr = yr

    if diverged or samples == 0:
        return 1e6 + (n - k)   # heavy penalty; earlier divergence = worse
    return (herr2 / samples) + 8.0 * (osc / samples)


def spsa(loss_fn, x0, iters=40, a=0.04, c=0.06, seed=0):
    """Minimise loss_fn via SPSA — stochastic-gradient descent for black-box sims."""
    rng = np.random.default_rng(seed)
    x = np.array(x0, dtype=float)
    lo = np.array([0.05, 0.0, 0.05])
    hi = np.array([1.2, 1.0, 1.0])
    best_x, best_l = x.copy(), loss_fn(x)
    print(f"init gains {x.round(3)}  loss {best_l:.4f}", flush=True)
    for k in range(iters):
        ak = a / (k + 1 + 10) ** 0.602
        ck = c / (k + 1) ** 0.101
        delta = rng.choice([-1.0, 1.0], size=len(x))
        lp = loss_fn(np.clip(x + ck * delta, lo, hi))
        lm = loss_fn(np.clip(x - ck * delta, lo, hi))
        ghat = (lp - lm) / (2.0 * ck) * delta      # stochastic gradient estimate
        x = np.clip(x - ak * ghat, lo, hi)          # SGD step
        l = loss_fn(x)
        if l < best_l:
            best_l, best_x = l, x.copy()
        print(f"iter {k+1:2d}/{iters}  gains {x.round(3)}  loss {l:.4f}  best {best_l:.4f}", flush=True)
    return best_x, best_l


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--secs", type=float, default=12.0)
    a = ap.parse_args()
    print(f"SPSA gain tuning — {a.iters} iters, {a.secs}s rollouts (headless)", flush=True)
    x0 = [0.35, 0.25, 0.35]   # current roam_controller defaults [Kp, Kd, steer_gain]
    best, loss = spsa(lambda g: rollout(g, secs=a.secs), x0, iters=a.iters)
    print("\n══════════════════════════════════════════════")
    print(f" BEST GAINS  Kp(lean_gain)={best[0]:.3f}  Kd(yaw_damp)={best[1]:.3f}  steer_gain={best[2]:.3f}")
    print(f" loss={loss:.4f}")
    print(" → set these in roam_controller params (lean_gain / yaw_damp / steer_gain)")
    print("══════════════════════════════════════════════")


if __name__ == "__main__":
    main()
