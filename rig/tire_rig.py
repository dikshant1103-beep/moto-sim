"""
tire_rig.py — Virtual Tire Test Rig  (kinematic rig + Pacejka PINN)

Coordinate frame (PyChrono, right-handed, Z-up)
    X  →  longitudinal (forward)
    Y  →  lateral      (left)
    Z  ↑  vertical     (up)

Rig model
─────────
  The wheel is KINEMATICALLY DRIVEN — position and orientation are prescribed
  each step; no free-body dynamics.  This matches a real tire test rig where
  the spindle carriage controls forward speed (vx) and the slip angle is set
  by the carriage heading angle (α = atan2(vy, vx)).

  The normal load Fz is prescribed (not computed from a contact spring).

  The PINN receives (Fz, α, κ, γ) and returns (Fx, Fy, Mz).
  Results are streamed via WebSocket to the Electron dashboard.

Dashboard sliders → ws_server.get_params():
  vx     [m/s]   forward speed of rig carriage   (0.5 – 15)
  vy     [m/s]   lateral speed offset            (−2 – +2)
  torque [Nm]    wheel drive torque              (−200 – +200)
"""

import math
import os
import sys as _sys
import time as _time_mod

import numpy as np
import matplotlib.pyplot as plt

import pychrono as chrono

USE_IRRLICHT = os.environ.get('TIRE_RIG_HEADLESS', '0') != '1'

# ── Test mode: set TIRE_RIG_TEST=<name>
#
#  PHASE 1 — Static
#   static       : fixed α=10°, κ=0.05, γ=0, Fz=1470 N  → output constant
#   alpha_sweep  : α ramps 0→20° over T_END, κ=0         → Fy/Mz Pacejka curve
#   kappa_sweep  : κ oscillates ±0.8, α=0                 → Fx tanh curve
#
#  PHASE 2 — Coupled
#   friction_ellipse : α=10° fixed, κ ramps 0→0.8         → Fy reduction
#   trail_braking    : γ=30°, α=10°, κ ramps 0→−0.8       → Fy collapse + Mz offset
#   camber_sweep     : γ oscillates ±30°, α=5° fixed       → Mz/Fy shift
#
#  PHASE 3 — Transient
#   sweep        : α ramps 0→0.5 rad over T_END, κ=0     → (legacy alias)
#   step         : α jumps at T_END/2, no-relaxation ref
#   relax_step   : α step; α' lags → verify τ = σ/Vx
#   low_speed    : Vx ramps 5→0.1 m/s, α=10° fixed       → blend stability
TEST_MODE = os.environ.get('TIRE_RIG_TEST', '').lower()
if TEST_MODE:
    print(f"[tire_rig] TEST MODE: {TEST_MODE!r}")
if USE_IRRLICHT:
    import pychrono.irrlicht as chronoirr

from pinn_model import DummyTirePINN
from tire_plots  import SimLog, plot_dashboard
import ws_server

# ═════════════════════════════════════════════════════════════════════════════
# Simulation parameters
# ═════════════════════════════════════════════════════════════════════════════
DT          = 0.001   # timestep          [s]
T_END       = 10.0    # episode length    [s]  — resets and repeats
LOG_EVERY   = 0.10    # console interval  [s]

WS_SEND_HZ  = 50
_ws_every   = max(1, round(1.0 / (DT * WS_SEND_HZ)))

WHEEL_RADIUS = 0.30   # m
WHEEL_WIDTH  = 0.20   # m
WHEEL_MASS   = 15.0   # kg

R_CROWN = 0.08        # m  — toroidal crown radius; y_cp = R_CROWN·sin(γ)

# Nominal vertical load — typical motorcycle rear tyre (~150 kg on rear)
FZ_NOMINAL = 1470.0   # N

# Default rig carriage speeds (overridden live from dashboard)
VX_DEFAULT =  5.0     # m/s
VY_DEFAULT =  0.0     # m/s  — vy slider acts as offset on top of auto-sweep

# Auto-sweep amplitude for vy — keeps slip-angle chart dynamic without slider input
VY_SWEEP_AMP = 2.0    # m/s  (sweeps α across full ±20° range at vx=5)

# ── Relaxation / wheel-dynamics constants ─────────────────────────────────────
# Defined after WHEEL_MASS / WHEEL_RADIUS so I_WHEEL can reference them.
SIGMA          = 0.30                                    # m  — relaxation length
I_WHEEL        = 0.5 * WHEEL_MASS * WHEEL_RADIUS ** 2   # kg·m²  (solid cylinder)
KAPPA_EPS      = 1e-3   # m/s — kappa denominator guard
VX_BLEND_LO    = 0.5    # m/s — below this: bypass relaxation ODE (alpha' → alpha)
VX_BLEND_HI    = 1.5    # m/s — above this: full ODE active
TORQUE_DEFAULT = 0.0    # Nm  — drive torque (dashboard-controllable)


def _smoothstep(x: float) -> float:
    """C¹ smooth step: 0 at x≤0, 1 at x≥1, cubic between."""
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


def _tire_deriv(
    alpha_prime: float, omega: float,
    alpha_kin: float, vx: float, gamma: float, Fz: float, torque: float,
) -> tuple:
    """
    Derivative function for the coupled [alpha_prime, omega_wheel] state.
    Called 4× per RK4 step (k1..k4).
    Returns: (d_alpha_prime, d_omega, Fx, Fy, Mz, kappa)
    """
    kappa = (WHEEL_RADIUS * omega - vx) / (abs(vx) + KAPPA_EPS)
    kappa = max(-1.0, min(1.0, kappa))

    Fx, Fy, Mz = pinn.predict_combined(
        Fz=Fz, slip_angle=alpha_prime, kappa=kappa, camber=gamma,
    )

    vx_abs  = abs(vx)
    blend   = _smoothstep((vx_abs - VX_BLEND_LO) / (VX_BLEND_HI - VX_BLEND_LO))
    d_alpha = blend * (vx_abs / SIGMA) * (alpha_kin - alpha_prime)
    d_omega = (torque - Fx * WHEEL_RADIUS) / I_WHEEL

    return d_alpha, d_omega, Fx, Fy, Mz, kappa

# ═════════════════════════════════════════════════════════════════════════════
# 1.  PyChrono system  (used for Irrlicht visualisation only)
# ═════════════════════════════════════════════════════════════════════════════
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))

# ── Ground plane (visual reference) ──────────────────────────────────────────
mat_ground = chrono.ChContactMaterialSMC()
mat_ground.SetYoungModulus(2.0e8)
mat_ground.SetFriction(0.8)
mat_ground.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(
    60.0, 60.0, 0.05,
    2000.0,
    False,   # no collision needed
    True,    # visual shape
    mat_ground,
)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, -0.025))
system.Add(ground)

# ── Wheel body — KINEMATIC (SetFixed=True, driven by SetPos/SetRot each step) ─
_vol = math.pi * WHEEL_RADIUS**2 * WHEEL_WIDTH
_rho = WHEEL_MASS / _vol

mat_wheel = chrono.ChContactMaterialSMC()
wheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    WHEEL_RADIUS,
    WHEEL_WIDTH,
    _rho,
    True,    # create_visualization ON
    False,   # create_collision OFF
    mat_wheel,
)
wheel.SetPos(chrono.ChVector3d(0.0, 0.0, WHEEL_RADIUS))
wheel.SetFixed(True)    # kinematic control — dynamics solver skips this body
system.Add(wheel)

# ═════════════════════════════════════════════════════════════════════════════
# 2.  Irrlicht visualisation  (set TIRE_RIG_HEADLESS=1 to disable)
# ═════════════════════════════════════════════════════════════════════════════
if USE_IRRLICHT:
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 600)
    vis.SetWindowTitle("Virtual Tire Test Rig — Kinematic + PINN")
    vis.Initialize()
    vis.AddCamera(
        chrono.ChVector3d(3.0, -4.0, 2.0),
        chrono.ChVector3d(0.0,  0.0, 0.3),
    )
    vis.AddTypicalLights()
    print("[tire_rig] Irrlicht window started.")
else:
    print("[tire_rig] Headless mode — streaming to Electron dashboard.")

# ═════════════════════════════════════════════════════════════════════════════
# 3.  PyTorch PINN
# ═════════════════════════════════════════════════════════════════════════════
pinn = DummyTirePINN(
    mu=0.9, k_alpha=12.0, k_gamma=0.9,
    C_pac=1.8, E_pac=0.0,
    tp0=0.03, Bt=10.0, Ct=1.5, K_gamma_Mz=0.02,
    Kx=12.0,
)
pinn.eval()

# ═════════════════════════════════════════════════════════════════════════════
# 4.  Data log
# ═════════════════════════════════════════════════════════════════════════════
sim_log = SimLog()

# ── Start WebSocket telemetry server ──────────────────────────────────────────
ws_server.start(host="localhost", port=8765)

# ═════════════════════════════════════════════════════════════════════════════
# 5.  Kinematic simulation loop
# ═════════════════════════════════════════════════════════════════════════════
print("\nTire rig running continuously. Use the dashboard to control parameters.")
print("Press Ctrl+C to stop.\n")

_run_index = 0

while True:                          # outer — restarts on Reset command
    _run_index += 1

    # Read initial params from dashboard
    params       = ws_server.get_params()
    vx           = params.get("vx",     VX_DEFAULT)
    vy           = params.get("vy",     VY_DEFAULT)
    torque_drive = params.get("torque", TORQUE_DEFAULT)

    # Reset kinematic state
    pos_x       = 0.0
    pos_y       = 0.0
    t           = 0.0
    next_log_t  = 0.0
    _ws_counter = 0
    _t_wall_0   = _time_mod.perf_counter()

    # Reset dynamics state
    alpha_prime = 0.0                        # relaxed slip angle [rad]
    omega_wheel = vx / WHEEL_RADIUS          # rolling without slip [rad/s]
    spin_acc    = 0.0                        # accumulated spin for 3-D viewport

    # ── Per-episode timing accumulators ────────────────────────────────────────
    _step_count   = 0
    _step_sum_ms  = 0.0
    _step_max_ms  = 0.0
    _spike_count  = 0          # steps > 2 ms
    _SPIKE_THRESH = 2.0        # ms

    # ── Payload sample counter (log every 100 WS pushes) ─────────────────────
    _ws_push_count = 0

    print(
        f"\n{'t[s]':>7}  {'vx[m/s]':>9}  {'α[°]':>8}  {'α\'[°]':>8}  "
        f"{'κ':>7}  {'ω[r/s]':>8}  {'Fx[N]':>8}  {'Fy[N]':>8}  {'Mz[Nm]':>8}"
    )
    print("─" * 100)

    while t <= T_END:

        _step_t0 = _time_mod.perf_counter()   # timing — start of step

        # ── Irrlicht render / real-time pacing ───────────────────────────────
        if USE_IRRLICHT:
            if not vis.Run():
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        else:
            # Soft real-time: pace sim to wall clock so 3-D viewport looks smooth
            _ahead = t - (_time_mod.perf_counter() - _t_wall_0)
            if _ahead > 0.002:
                _time_mod.sleep(_ahead - 0.001)

        # ── Live params (updated every step from dashboard sliders) ──────────
        params       = ws_server.get_params()
        vx           = params.get("vx",     VX_DEFAULT)
        torque_drive = params.get("torque", TORQUE_DEFAULT)

        # ── Test-mode input overrides ────────────────────────────────────────
        # gamma defaults to 0; test modes may override it before the
        # prescribed-state block that uses it.
        gamma = 0.0

        if TEST_MODE == 'static':
            vy           = VX_DEFAULT * math.tan(math.radians(10.0))
            torque_drive = 0.0
            omega_wheel  = vx * (1.0 + 0.05) / WHEEL_RADIUS   # pin κ=0.05

        elif TEST_MODE in ('sweep', 'alpha_sweep'):
            # α ramps 0 → 20° over T_END
            vy           = VX_DEFAULT * math.tan(math.radians(20.0) * t / T_END)
            torque_drive = 0.0
            omega_wheel  = vx / WHEEL_RADIUS   # κ = 0

        elif TEST_MODE == 'kappa_sweep':
            # κ oscillates ±0.8; α=0
            vy           = 0.0
            torque_drive = 0.0
            kappa_target = 0.8 * math.sin(2.0 * math.pi * t / T_END)
            omega_wheel  = vx * (1.0 + kappa_target) / WHEEL_RADIUS

        elif TEST_MODE == 'friction_ellipse':
            # α=10° fixed, κ ramps 0 → 0.8
            vy           = VX_DEFAULT * math.tan(math.radians(10.0))
            torque_drive = 0.0
            kappa_target = 0.8 * (t / T_END)
            omega_wheel  = vx * (1.0 + kappa_target) / WHEEL_RADIUS

        elif TEST_MODE == 'trail_braking':
            # γ=30°, α=10°, braking torque ramps 0 → −200 Nm
            gamma        = math.radians(30.0)
            vy           = VX_DEFAULT * math.tan(math.radians(10.0))
            torque_drive = -200.0 * (t / T_END)

        elif TEST_MODE == 'camber_sweep':
            # γ oscillates ±30°, α=5° fixed
            gamma        = math.radians(30.0) * math.sin(2.0 * math.pi * t / T_END)
            vy           = VX_DEFAULT * math.tan(math.radians(5.0))
            torque_drive = 0.0
            omega_wheel  = vx / WHEEL_RADIUS

        elif TEST_MODE == 'low_speed':
            # Vx ramps 5 → 0.1 m/s; α=10° fixed
            vx           = max(VX_DEFAULT * (1.0 - t / T_END) + 0.1, 0.1)
            vy           = vx * math.tan(math.radians(10.0))
            torque_drive = 0.0

        elif TEST_MODE in ('step', 'relax_step'):
            vy           = 0.0 if t < T_END / 2 else VX_DEFAULT * math.tan(math.radians(15.0))
            torque_drive = 0.0
            if TEST_MODE == 'step':
                omega_wheel = vx / WHEEL_RADIUS   # no relaxation reference

        else:
            # Normal interactive mode: auto-sweep vy + dashboard offsets
            _phase = 2.0 * math.pi * t / T_END
            vy     = VY_SWEEP_AMP * math.sin(_phase) + params.get("vy", VY_DEFAULT)

        # ── Prescribed tire state ────────────────────────────────────────────
        Fz       = FZ_NOMINAL
        # gamma already set above (default 0.0, overridden by some test modes)
        alpha_kin = math.atan2(vy, max(abs(vx), 0.01))   # kinematic slip [rad]
        y_cp     = R_CROWN * math.sin(gamma)

        # ── RK4 over [alpha_prime, omega_wheel] ──────────────────────────────
        # k1
        da1, dw1, _, _, _, _ = _tire_deriv(
            alpha_prime, omega_wheel, alpha_kin, vx, gamma, Fz, torque_drive)
        # k2
        da2, dw2, _, _, _, _ = _tire_deriv(
            alpha_prime + 0.5*DT*da1, omega_wheel + 0.5*DT*dw1,
            alpha_kin, vx, gamma, Fz, torque_drive)
        # k3
        da3, dw3, _, _, _, _ = _tire_deriv(
            alpha_prime + 0.5*DT*da2, omega_wheel + 0.5*DT*dw2,
            alpha_kin, vx, gamma, Fz, torque_drive)
        # k4
        da4, dw4, _, _, _, _ = _tire_deriv(
            alpha_prime + DT*da3, omega_wheel + DT*dw3,
            alpha_kin, vx, gamma, Fz, torque_drive)

        alpha_prime += DT / 6.0 * (da1 + 2.0*da2 + 2.0*da3 + da4)
        omega_wheel += DT / 6.0 * (dw1 + 2.0*dw2 + 2.0*dw3 + dw4)

        # Low-speed bypass: smoothly pull alpha_prime → alpha_kin near standstill
        _blend = _smoothstep((abs(vx) - VX_BLEND_LO) / (VX_BLEND_HI - VX_BLEND_LO))
        alpha_prime = _blend * alpha_prime + (1.0 - _blend) * alpha_kin

        # ── Final forces from integrated state ───────────────────────────────
        kappa = (WHEEL_RADIUS * omega_wheel - vx) / (abs(vx) + KAPPA_EPS)
        kappa = max(-1.0, min(1.0, kappa))
        Fx, Fy, Mz = pinn.predict_combined(
            Fz=Fz, slip_angle=alpha_prime, kappa=kappa, camber=gamma,
        )

        # ── Kinematic integration ─────────────────────────────────────────────
        pos_x   += vx * DT
        pos_y   += vy * DT
        spin_acc += omega_wheel * DT   # physical wheel spin for 3-D viewport

        # ── Update PyChrono wheel pose ────────────────────────────────────────
        wheel.SetPos(chrono.ChVector3d(pos_x, pos_y, WHEEL_RADIUS))
        q_spin = chrono.ChQuaterniond()
        q_spin.SetFromAngleAxis(spin_acc, chrono.ChVector3d(0.0, 1.0, 0.0))
        wheel.SetRot(q_spin)

        system.DoStepDynamics(DT)
        t += DT

        # ── Pause / reset from dashboard ─────────────────────────────────────
        while ws_server.is_paused():
            _time_mod.sleep(0.05)
        if ws_server.is_reset_requested():
            break

        # ── Derived quantities ────────────────────────────────────────────────
        slip_deg        = math.degrees(alpha_kin)
        alpha_prime_deg = math.degrees(alpha_prime)
        camber_deg      = math.degrees(gamma)

        # ── WebSocket telemetry (downsampled to WS_SEND_HZ) ───────────────────
        _ws_counter += 1
        if _ws_counter >= _ws_every:
            _ws_counter = 0
            _ws_push_count += 1
            _frame = {
                "time":        round(t, 4),
                "Fx":          round(Fx, 3),
                "Fy":          round(Fy, 3),
                "Fz":          round(Fz, 3),
                "alpha":       round(slip_deg, 4),         # kinematic α [°]
                "alpha_prime": round(alpha_prime_deg, 4),  # relaxed α'  [°]
                "kappa":       round(kappa, 5),            # physical κ
                "omega":       round(omega_wheel, 4),      # wheel ω [rad/s]
                "gamma":       round(camber_deg, 4),
                "Mz":          round(Mz, 4),
                "vx":          round(vx, 4),
                "vy":          round(vy, 4),
                "y_cp":        round(y_cp, 5),
                # Pose for 3-D viewport
                "px":  round(pos_x, 4),
                "py":  round(pos_y, 4),
                "pz":  round(WHEEL_RADIUS, 4),
                "spin": round(spin_acc % (2.0 * math.pi), 4),
            }
            ws_server.push_frame(_frame)

            if _ws_push_count % 100 == 0:
                print(
                    f"[PAYLOAD #{_ws_push_count:6d}] "
                    f"t={_frame['time']:.3f}s  "
                    f"α={_frame['alpha']:+7.3f}°  α'={_frame['alpha_prime']:+7.3f}°  "
                    f"κ={_frame['kappa']:+.4f}  ω={_frame['omega']:+7.2f}r/s  "
                    f"Fx={_frame['Fx']:+8.2f}N  Fy={_frame['Fy']:+8.2f}N  "
                    f"clients={ws_server.stats()['clients']}",
                    flush=True,
                )

        # ── SimLog (Fy_steady = PINN at kinematic α, Fy = relaxed output) ─────
        Fy_steady_ref, _ = pinn.predict(Fz=Fz, slip_angle=alpha_kin, camber=gamma)
        sim_log.log(
            t         = t,
            alpha_deg = slip_deg,
            gamma_deg = camber_deg,
            Fz        = Fz,
            Fy        = Fy,
            Fy_steady = float(Fy_steady_ref),
        )

        # ── Timing ────────────────────────────────────────────────────────────
        _dt_ms = (_time_mod.perf_counter() - _step_t0) * 1e3
        _step_count  += 1
        _step_sum_ms += _dt_ms
        if _dt_ms > _step_max_ms:
            _step_max_ms = _dt_ms
        if _dt_ms > _SPIKE_THRESH:
            _spike_count += 1

        # ── Console print ─────────────────────────────────────────────────────
        if t >= next_log_t - 1e-9:
            print(
                f"{t:7.3f}  {vx:9.4f}  {slip_deg:8.3f}  {alpha_prime_deg:8.3f}  "
                f"{kappa:7.4f}  {omega_wheel:8.2f}  "
                f"{Fx:8.3f}  {Fy:8.3f}  {Mz:8.4f}"
            )
            next_log_t += LOG_EVERY

    # ── Episode timing summary ─────────────────────────────────────────────────
    _mean_ms = _step_sum_ms / max(_step_count, 1)
    _budget_ok = "OK" if _mean_ms < 1.0 else "OVER BUDGET"
    print(
        f"─ run {_run_index:4d} complete  t={t:.3f}s  "
        f"step: mean={_mean_ms:.3f}ms  max={_step_max_ms:.3f}ms  "
        f"spikes(>{_SPIKE_THRESH:.0f}ms)={_spike_count}  [{_budget_ok}]"
        f"  ws_pushes={_ws_push_count}\n"
    )

    # ── relax_step validation plot (once per run) ──────────────────────────────
    if TEST_MODE == 'relax_step' and len(sim_log) > 0:
        import numpy as np
        d = sim_log.to_arrays()
        mask_post = np.asarray(d["t"]) >= T_END / 2
        t_post = np.asarray(d["t"])[mask_post] - T_END / 2
        Fy_post    = np.asarray(d["Fy"])[mask_post]
        Fy_ss_post = np.asarray(d["Fy_steady"])[mask_post]

        tau = SIGMA / VX_DEFAULT
        alpha_step_rad = math.atan2(
            VX_DEFAULT * math.tan(math.radians(15.0)), VX_DEFAULT)
        Fy_final_ref, _ = pinn.predict(Fz=FZ_NOMINAL, slip_angle=alpha_step_rad)
        Fy_theory = Fy_final_ref * (1.0 - np.exp(-t_post / tau))

        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        fig.suptitle(
            f"Relaxation step validation  (σ={SIGMA} m, τ=σ/vx={tau:.3f} s)",
            fontsize=11, fontweight="bold")

        ax = axes[0]
        ax.plot(d["t"], d["Fy"],          color="tomato",    lw=2.0, label="Fy (relaxed)")
        ax.plot(d["t"], d["Fy_steady"],   color="steelblue", lw=1.2, ls="--",
                label="Fy_steady (kinematic α)")
        ax.axvline(T_END / 2, color="gray", lw=0.9, ls=":", label="step input")
        ax.set_xlabel("t [s]"); ax.set_ylabel("Fy [N]")
        ax.set_title("Fy vs time"); ax.legend(fontsize=8); ax.grid(alpha=0.35)

        ax2 = axes[1]
        ax2.plot(t_post, Fy_post,    color="tomato",   lw=2.0, label="Fy (sim)")
        ax2.plot(t_post, Fy_theory,  color="limegreen", lw=1.5, ls="--",
                 label=f"Fy_theory (1−e^(−t/τ)), τ={tau:.3f}s")
        ax2.set_xlabel("t − t_step [s]"); ax2.set_ylabel("Fy [N]")
        ax2.set_title("Post-step: sim vs theory")
        ax2.legend(fontsize=8); ax2.grid(alpha=0.35)

        fig.tight_layout()
        _plot_path = f"/tmp/relax_step_run{_run_index}.png"
        fig.savefig(_plot_path, dpi=130, bbox_inches="tight")
        print(f"[relax_step] validation plot → {_plot_path}")
        plt.close(fig)
        break   # single-run validation
