"""
tire_plots.py — Visualization and diagnostics module for tire simulation output.

Zero physics code.  All functions accept logged data (SimLog or plain dicts)
and return matplotlib objects.

Public API
──────────
    SimLog              — append-only data logger; use inside simulation loops
    RealTimeMonitor     — optional live Fy display via plt.ion()
    plot_fy_vs_time()   — Panel B: transient Fy and Fy_steady vs time
    plot_fy_vs_slip()   — Panel A: Fy vs slip angle with saturation envelope
    plot_contributions()— Panel C: slip vs camber force contributions
    plot_phase_portrait()— Panel D: Fy vs α colored by time
    plot_crossover()    — Panel E: required α vs γ (crossover annotation)
    plot_dashboard()    — Combined A+B+C+D diagnostic figure

Quick start
───────────
    log = SimLog()
    # inside loop:
    log.log(t=t, alpha_deg=α°, gamma_deg=γ°, Fz=Fz, Fy=Fy, Fy_steady=Fy_ss)
    # after loop:
    fig = plot_dashboard(log, title="run-1", save_path="dashboard.png")
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize


# ═════════════════════════════════════════════════════════════════════════════
# 1. Data container
# ═════════════════════════════════════════════════════════════════════════════

@dataclass
class SimLog:
    """
    Append-only time-series logger.

    Channels
    ────────
    t          : simulation time          [s]
    alpha_deg  : slip angle               [°]
    gamma_deg  : camber angle             [°]
    Fz         : normal load              [N]
    Fy         : applied lateral force    [N]  (after relaxation filter)
    Fy_steady  : PINN steady-state output [N]  (before relaxation filter)

    When no relaxation is used, pass Fy_steady = Fy (they will be identical
    and the dashboard will annotate "no relaxation applied").
    """

    t:          list = field(default_factory=list)
    alpha_deg:  list = field(default_factory=list)
    gamma_deg:  list = field(default_factory=list)
    Fz:         list = field(default_factory=list)
    Fy:         list = field(default_factory=list)
    Fy_steady:  list = field(default_factory=list)

    def log(
        self, *,
        t:         float,
        alpha_deg: float,
        gamma_deg: float,
        Fz:        float,
        Fy:        float,
        Fy_steady: float,
    ) -> None:
        """Append one sample.  Call once per simulation timestep."""
        self.t.append(t)
        self.alpha_deg.append(alpha_deg)
        self.gamma_deg.append(gamma_deg)
        self.Fz.append(Fz)
        self.Fy.append(Fy)
        self.Fy_steady.append(Fy_steady)

    def to_arrays(self) -> dict:
        """Return {channel_name: np.ndarray} for all channels."""
        return {k: np.asarray(v, dtype=np.float64)
                for k, v in self.__dict__.items()}

    def __len__(self) -> int:
        return len(self.t)


# ═════════════════════════════════════════════════════════════════════════════
# 2. Real-time monitor
# ═════════════════════════════════════════════════════════════════════════════

class RealTimeMonitor:
    """
    Lightweight live Fy display using matplotlib interactive mode.

    Updates every `update_every` simulation steps — lightweight blit-free
    approach using draw_idle() + flush_events().

    COMPATIBILITY NOTE
    ──────────────────
    Works in standalone Python simulation loops (tire_validation.py).
    NOT compatible with PyChrono/Irrlicht (tire_rig.py): Irrlicht owns its
    own event loop; plt.ion() will conflict.  Disable the monitor in
    Irrlicht-based simulations.

    Usage
    ─────
        monitor = RealTimeMonitor(update_every=50)
        for step in range(n_steps):
            log.log(...)
            monitor.update(log)
        monitor.close()
    """

    def __init__(
        self,
        update_every: int   = 50,
        window_s:     float = 5.0,
        title:        str   = "Real-time Tire Monitor",
    ) -> None:
        self.update_every = max(1, update_every)
        self.window_s     = window_s
        self._step        = 0

        plt.ion()
        self.fig, axes = plt.subplots(1, 2, figsize=(12, 3))
        self.fig.suptitle(title, fontsize=10)

        # Left: Fy vs time
        ax_t = axes[0]
        self.line_fy,  = ax_t.plot([], [], color="tomato",    lw=1.5,
                                    label="Fy (applied)")
        self.line_fys, = ax_t.plot([], [], color="steelblue", lw=1.0, ls="--",
                                    label="Fy_steady (PINN)")
        ax_t.set_xlabel("t [s]")
        ax_t.set_ylabel("Fy [N]")
        ax_t.legend(fontsize=7, loc="upper right")
        ax_t.grid(True, alpha=0.35)
        ax_t.set_title("Lateral force vs time", fontsize=9)

        # Right: Fy vs alpha scatter (rolling)
        ax_s = axes[1]
        self.scat = ax_s.scatter([], [], s=2, c=[], cmap="viridis",
                                  vmin=0.0, vmax=1.0, alpha=0.6)
        ax_s.set_xlabel("α [°]")
        ax_s.set_ylabel("Fy [N]")
        ax_s.grid(True, alpha=0.35)
        ax_s.set_title("Fy vs slip angle", fontsize=9)

        self._ax_t = ax_t
        self._ax_s = ax_s

        self.fig.tight_layout()
        self.fig.canvas.draw()
        plt.pause(0.001)

    def update(self, log: SimLog) -> None:
        """Call once per simulation step; plots refresh every update_every calls."""
        self._step += 1
        if self._step % self.update_every != 0 or len(log) < 2:
            return

        d = log.to_arrays()
        t, fy, fys, alpha = d["t"], d["Fy"], d["Fy_steady"], d["alpha_deg"]

        # Rolling window
        if self.window_s > 0.0 and (t[-1] - t[0]) > self.window_s:
            mask = t >= t[-1] - self.window_s
            t, fy, fys, alpha = t[mask], fy[mask], fys[mask], alpha[mask]

        # Time axis
        self.line_fy.set_data(t, fy)
        self.line_fys.set_data(t, fys)
        self._ax_t.relim()
        self._ax_t.autoscale_view()

        # Slip scatter (color = normalised time position in window)
        t_norm = (t - t[0]) / max(t[-1] - t[0], 1e-9)
        offsets = np.column_stack([alpha, fy])
        self.scat.set_offsets(offsets)
        self.scat.set_array(t_norm)
        self._ax_s.relim()
        self._ax_s.autoscale_view()

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def close(self) -> None:
        plt.ioff()
        plt.close(self.fig)


# ═════════════════════════════════════════════════════════════════════════════
# 3. Individual plot functions
# ═════════════════════════════════════════════════════════════════════════════

def plot_fy_vs_time(
    data:    dict,
    ax:      plt.Axes | None = None,
    *,
    title:   str  = "B — Fy vs Time",
) -> plt.Axes:
    """
    Panel B: Fy (applied) and Fy_steady (PINN output) vs time.

    The gap between the two lines visualises relaxation lag.
    If both are identical (no relaxation applied), a note is added.

    Parameters
    ----------
    data : dict returned by SimLog.to_arrays()
    ax   : optional Axes to plot into (creates a new figure if None)
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 3))

    t   = data["t"]
    fy  = data["Fy"]
    fys = data["Fy_steady"]

    has_relaxation = np.max(np.abs(fy - fys)) > 0.5   # >0.5 N gap → relaxation active

    ax.plot(t, fys, color="steelblue", lw=1.4, ls="--",
            label="Fy_steady  (PINN, no filter)", zorder=3)
    ax.plot(t, fy,  color="tomato",    lw=2.0,
            label="Fy  (applied, relaxed)", zorder=4)

    if has_relaxation:
        ax.fill_between(t, fy, fys, where=(fys > fy),
                        color="steelblue", alpha=0.10,
                        label="Relaxation lag  (Fy_ss > Fy)")
        ax.fill_between(t, fy, fys, where=(fys < fy),
                        color="tomato",    alpha=0.10)
    else:
        ax.text(0.98, 0.05, "Fy = Fy_steady\n(no relaxation)",
                transform=ax.transAxes, fontsize=7, color="dimgray",
                ha="right", va="bottom")

    ax.set_xlabel("t [s]")
    ax.set_ylabel("Fy [N]")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="upper left")
    ax.grid(True, alpha=0.35)
    ax.set_xlim(t[0], t[-1])
    return ax


def plot_fy_vs_slip(
    data:  dict,
    ax:    plt.Axes | None = None,
    *,
    mu:    float = 0.9,
    title: str   = "A — Fy vs Slip Angle",
) -> plt.Axes:
    """
    Panel A: Fy and Fy_steady vs slip angle α.

    Overlays the theoretical peak (μ·Fz_mean) as a saturation reference.
    The two series show how the relaxation filter shifts the operating point
    at each slip angle.

    Parameters
    ----------
    data  : dict from SimLog.to_arrays()
    mu    : peak friction coefficient (for saturation reference line)
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(6, 5))

    alpha = data["alpha_deg"]
    fy    = data["Fy"]
    fys   = data["Fy_steady"]
    Fz    = data["Fz"]
    t     = data["t"]

    Fy_peak = mu * np.mean(Fz[Fz > 1.0]) if np.any(Fz > 1.0) else mu * 1000.0

    ax.scatter(alpha, fys, s=4,  c="steelblue", alpha=0.40, label="Fy_steady",  zorder=2)
    ax.scatter(alpha, fy,  s=4,  c="tomato",    alpha=0.55, label="Fy (applied)", zorder=3)
    ax.axhline( Fy_peak, color="gray", lw=0.9, ls=":", label=f"+μFz ≈ {Fy_peak:.0f} N")
    ax.axhline(-Fy_peak, color="gray", lw=0.9, ls=":")
    ax.axhline(0.0,      color="black", lw=0.5, alpha=0.4)
    ax.axvline(0.0,      color="black", lw=0.5, alpha=0.4)

    ax.set_xlabel("Slip angle α [°]")
    ax.set_ylabel("Fy [N]")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(True, alpha=0.35)
    return ax


def plot_contributions(
    data:    dict,
    ax:      plt.Axes | None = None,
    *,
    k_alpha: float = 10.0,
    k_gamma: float = 0.9,
    title:   str   = "C — Slip vs Camber Contributions",
) -> plt.Axes:
    """
    Panel C: linear slip and camber force components vs time.

    Computes:
        Fy_slip_lin   = k_alpha · Fz · α_rad   (pre-saturation slip component)
        Fy_camber_lin = k_gamma · Fz · γ_rad   (pre-saturation camber component)

    When γ = 0 throughout, the camber curve is flat at zero — correct and
    expected for car-tire or no-lean tests.

    Parameters
    ----------
    k_alpha, k_gamma : normalised stiffness coefficients (must match model)
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 3))

    t     = data["t"]
    Fz    = data["Fz"]
    alpha = np.radians(data["alpha_deg"])
    gamma = np.radians(data["gamma_deg"])
    fy    = data["Fy"]

    Fy_slip   = k_alpha * Fz * alpha
    Fy_camber = k_gamma * Fz * gamma
    Fy_raw    = Fy_slip + Fy_camber   # pre-saturation sum

    ax.plot(t, Fy_slip,   color="steelblue",   lw=1.5, label=f"Slip  (k_α·Fz·α)")
    ax.plot(t, Fy_camber, color="saddlebrown", lw=1.5, label=f"Camber (k_γ·Fz·γ)")
    ax.plot(t, Fy_raw,    color="seagreen",    lw=1.2, ls="--",
            label="Fy_raw (slip + camber, linear)")
    ax.plot(t, fy,        color="tomato",      lw=2.0,
            label="Fy (saturated, applied)")

    ax.axhline(0.0, color="black", lw=0.5, alpha=0.4)

    max_camber_frac = (np.max(np.abs(Fy_camber)) / max(np.max(np.abs(Fy_raw)), 1.0)) * 100
    ax.text(0.02, 0.97,
            f"Max camber fraction: {max_camber_frac:.1f}%",
            transform=ax.transAxes, fontsize=8, va="top", color="saddlebrown")

    ax.set_xlabel("t [s]")
    ax.set_ylabel("Force [N]")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="upper right")
    ax.grid(True, alpha=0.35)
    ax.set_xlim(t[0], t[-1])
    return ax


def plot_phase_portrait(
    data:  dict,
    ax:    plt.Axes | None = None,
    *,
    title: str = "D — Phase Portrait  (Fy vs α, colored by time)",
) -> plt.Axes:
    """
    Panel D: Fy vs slip angle as a trajectory colored by simulation time.

    Early time = dark (viridis low), late time = bright (viridis high).
    The trajectory shape reveals hysteresis due to relaxation:
      - Without relaxation: collapses to a single curve
      - With relaxation and a step input: shows a loop (rise then settle)
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(6, 5))

    alpha = data["alpha_deg"]
    fy    = data["Fy"]
    t     = data["t"]

    # Build LineCollection for smooth time-colored trajectory
    pts   = np.column_stack([alpha, fy]).reshape(-1, 1, 2)
    segs  = np.concatenate([pts[:-1], pts[1:]], axis=1)
    norm  = Normalize(vmin=t[0], vmax=t[-1])
    lc    = LineCollection(segs, cmap="viridis", norm=norm, linewidth=1.5, alpha=0.85)
    lc.set_array(np.asarray(t[:-1]))
    ax.add_collection(lc)

    # Colorbar
    sm = ScalarMappable(cmap="viridis", norm=norm)
    sm.set_array([])
    cb = plt.colorbar(sm, ax=ax, pad=0.02)
    cb.set_label("t [s]", fontsize=8)
    cb.ax.tick_params(labelsize=7)

    # Mark start/end
    ax.scatter([alpha[0]],  [fy[0]],  s=40, c="navy",    zorder=5, label="start")
    ax.scatter([alpha[-1]], [fy[-1]], s=40, c="crimson",  zorder=5, label="end")

    ax.autoscale()
    ax.set_xlabel("Slip angle α [°]")
    ax.set_ylabel("Fy [N]")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(True, alpha=0.35)
    ax.axhline(0.0, color="black", lw=0.5, alpha=0.4)
    ax.axvline(0.0, color="black", lw=0.5, alpha=0.4)
    return ax


def plot_crossover(
    gammas_deg:  np.ndarray,
    alphas_deg:  np.ndarray,
    *,
    gc_deg:      float | None = None,
    Fy_req:      float | None = None,
    F_max:       float | None = None,
    ax:          plt.Axes | None = None,
    title:       str           = "E — Crossover Angle  (α* vs γ)",
) -> plt.Axes:
    """
    Panel E: required equilibrium slip angle α* vs camber angle γ.

    Annotates the crossover point (α=0) and shades the slip-dominated vs
    camber-dominated regions.

    Parameters
    ----------
    gammas_deg : camber angle sweep [°]
    alphas_deg : corresponding equilibrium slip angles [°]
    gc_deg     : crossover angle [°]; computed from zero-crossing if None
    Fy_req     : required force [N] for annotation
    F_max      : friction limit [N] for annotation
    """
    if ax is None:
        _, ax = plt.subplots(figsize=(7, 5))

    # Auto-detect crossover from sign change if not provided
    if gc_deg is None:
        sign_changes = np.where(np.diff(np.sign(alphas_deg)))[0]
        if len(sign_changes) > 0:
            i = sign_changes[0]
            a0, a1 = alphas_deg[i], alphas_deg[i + 1]
            g0, g1 = gammas_deg[i], gammas_deg[i + 1]
            gc_deg = g0 + (-a0) * (g1 - g0) / (a1 - a0)

    ax.axhline(0.0, color="gray", lw=0.9, ls="--", alpha=0.70)
    ax.plot(gammas_deg, alphas_deg, color="steelblue", lw=2.2,
            label="α*(γ)  equilibrium slip")

    if gc_deg is not None:
        ax.axvline(gc_deg, color="tomato", lw=1.3, ls="--",
                   label=f"γ_c = {gc_deg:.1f}°")
        ax.fill_between(gammas_deg, alphas_deg,
                        where=(gammas_deg <= gc_deg),
                        color="steelblue", alpha=0.07,
                        label="Slip-dominated  (α > 0)")
        ax.fill_between(gammas_deg, alphas_deg,
                        where=(gammas_deg >= gc_deg),
                        color="tomato", alpha=0.07,
                        label="Camber-dominated  (α < 0)")

    label_str = ""
    if Fy_req is not None and F_max is not None:
        label_str = f"Fy_req = {Fy_req:.0f} N  ({Fy_req/F_max*100:.0f}% μFz)"
    elif Fy_req is not None:
        label_str = f"Fy_req = {Fy_req:.0f} N"
    if label_str:
        ax.text(0.02, 0.97, label_str, transform=ax.transAxes,
                fontsize=8, va="top", color="dimgray")

    ax.set_xlabel("Camber angle γ [°]")
    ax.set_ylabel("Required slip angle α* [°]")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.35)
    ax.set_xlim(gammas_deg[0], gammas_deg[-1])
    return ax


def plot_mz_vs_slip(
    alphas_deg: np.ndarray,
    Fy:         np.ndarray,
    Mz:         np.ndarray,
    *,
    ax_fy:   "plt.Axes | None" = None,
    ax_mz:   "plt.Axes | None" = None,
    Fz:      float = 1000.0,
    mu:      float = 0.9,
    title:   str   = "Fy & Mz vs Slip Angle",
) -> tuple:
    """
    Overlay Fy and Mz on stacked axes sharing the x-axis.

    Annotates:
      - Fy peak with vertical reference line
      - Mz peak (most negative = max steering feedback)
      - Mz zero-crossing (torque reversal / "steering goes light")

    Parameters
    ----------
    alphas_deg : slip angle sweep [°]
    Fy         : lateral force [N]
    Mz         : aligning torque [N·m]
    ax_fy, ax_mz : optional existing Axes (must share x); created if None
    Fz, mu     : for saturation reference line on Fy panel

    Returns
    -------
    (ax_fy, ax_mz)
    """
    if ax_fy is None or ax_mz is None:
        fig, (ax_fy, ax_mz) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        fig.suptitle(title, fontsize=11, fontweight="bold")

    Fy_peak_lim  = mu * Fz
    idx_fy_pk    = int(np.argmax(Fy))
    alpha_fy_pk  = alphas_deg[idx_fy_pk]
    idx_mz_pk    = int(np.argmin(Mz))
    alpha_mz_pk  = alphas_deg[idx_mz_pk]

    # Torque reversal: Mz transitions from negative → positive (skip trivial α=0 zero)
    rev_idx  = np.where((Mz[:-1] < 0.0) & (Mz[1:] >= 0.0))[0]
    alpha_zc = None
    if len(rev_idx) > 0:
        i = rev_idx[0]
        alpha_zc = alphas_deg[i] + (-Mz[i]) * (alphas_deg[i+1] - alphas_deg[i]) / (Mz[i+1] - Mz[i])

    # ── Fy panel ──────────────────────────────────────────────────────────────
    ax_fy.plot(alphas_deg, Fy, color="steelblue", lw=2.2, label="Fy")
    ax_fy.axhline(Fy_peak_lim, color="tomato", lw=0.9, ls=":", alpha=0.7,
                  label=f"μ·Fz = {Fy_peak_lim:.0f} N")
    ax_fy.axvline(alpha_fy_pk, color="steelblue", lw=0.9, ls="--", alpha=0.55)
    if alpha_zc is not None:
        ax_fy.axvline(alpha_zc, color="tomato", lw=1.0, ls="--", alpha=0.6,
                      label=f"Mz reversal @ {alpha_zc:.1f}°")
    ax_fy.set_ylabel("Fy [N]")
    ax_fy.legend(fontsize=8, loc="lower right")
    ax_fy.grid(True, alpha=0.35)
    ax_fy.set_ylim(0.0, Fy_peak_lim * 1.18)
    ax_fy.annotate(
        f"Fy peak\n{Fy[idx_fy_pk]:.0f} N @ {alpha_fy_pk:.1f}°",
        xy=(alpha_fy_pk, Fy[idx_fy_pk]),
        xytext=(alpha_fy_pk + alphas_deg[-1] * 0.06, Fy[idx_fy_pk] * 0.84),
        fontsize=8, color="steelblue",
        arrowprops=dict(arrowstyle="->", color="steelblue", lw=0.8),
    )

    # ── Mz panel ──────────────────────────────────────────────────────────────
    ax_mz.axhline(0.0, color="black", lw=0.7, alpha=0.5)
    if alpha_zc is not None:
        ax_mz.axvspan(alpha_zc, alphas_deg[-1], alpha=0.08, color="tomato",
                      label="Destabilising region")
    ax_mz.plot(alphas_deg, Mz, color="seagreen", lw=2.2, label="Mz")
    ax_mz.axvline(alpha_fy_pk, color="steelblue", lw=0.9, ls="--", alpha=0.55,
                  label=f"Fy peak @ {alpha_fy_pk:.1f}°")
    ax_mz.annotate(
        f"Mz peak\n{Mz[idx_mz_pk]:.1f} N·m\n@ {alpha_mz_pk:.1f}°",
        xy=(alpha_mz_pk, Mz[idx_mz_pk]),
        xytext=(alpha_mz_pk + alphas_deg[-1] * 0.06, Mz[idx_mz_pk] * 1.3),
        fontsize=8, color="seagreen",
        arrowprops=dict(arrowstyle="->", color="seagreen", lw=0.8),
    )
    if alpha_zc is not None:
        ax_mz.axvline(alpha_zc, color="tomato", lw=1.0, ls="--", alpha=0.7)
        ax_mz.text(
            alpha_zc + alphas_deg[-1] * 0.02, 0.3,
            f"steering\ngoes light\n@ {alpha_zc:.1f}°",
            fontsize=7.5, color="tomato",
        )
    ax_mz.set_xlabel("Slip angle α [°]")
    ax_mz.set_ylabel("Mz [N·m]")
    ax_mz.legend(fontsize=8, loc="upper right")
    ax_mz.grid(True, alpha=0.35)
    ax_mz.set_xlim(0.0, alphas_deg[-1])

    return ax_fy, ax_mz


# ═════════════════════════════════════════════════════════════════════════════
# 4. Combined dashboard
# ═════════════════════════════════════════════════════════════════════════════

def plot_dashboard(
    log:          SimLog,
    *,
    title:        str         = "Tire Diagnostics Dashboard",
    save_path:    str | None  = None,
    model_params: dict | None = None,
    mu:           float       = 0.9,
) -> plt.Figure:
    """
    2×2 diagnostic figure combining Panels A, B, C, D.

    Parameters
    ----------
    log          : SimLog filled during the simulation run
    title        : figure super-title
    save_path    : if given, save the figure to this path (e.g. "dash.png")
    model_params : dict with keys 'k_alpha', 'k_gamma' (for Panel C).
                   Defaults to {k_alpha: 10.0, k_gamma: 0.9} if not provided.
    mu           : peak friction coefficient (for Panel A saturation line)

    Returns
    -------
    plt.Figure

    Panel layout
    ────────────
    ┌───────────────────┬──────────────────────┐
    │ A: Fy vs α        │ B: Fy vs time        │
    │   (scatter)       │   (Fy + Fy_steady)   │
    ├───────────────────┼──────────────────────┤
    │ C: Contributions  │ D: Phase portrait    │
    │   (slip + camber) │   (α→Fy, colored t)  │
    └───────────────────┴──────────────────────┘
    """
    if len(log) < 2:
        raise ValueError("SimLog must have at least 2 samples before plotting.")

    params = {"k_alpha": 10.0, "k_gamma": 0.9}
    if model_params:
        params.update(model_params)

    data = log.to_arrays()

    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(
        title,
        fontsize=13, fontweight="bold",
    )
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.45, wspace=0.32)

    ax_A = fig.add_subplot(gs[0, 0])
    ax_B = fig.add_subplot(gs[0, 1])
    ax_C = fig.add_subplot(gs[1, 0])
    ax_D = fig.add_subplot(gs[1, 1])

    plot_fy_vs_slip(      data, ax=ax_A, mu=mu)
    plot_fy_vs_time(      data, ax=ax_B)
    plot_contributions(   data, ax=ax_C,
                          k_alpha=params["k_alpha"],
                          k_gamma=params["k_gamma"])
    plot_phase_portrait(  data, ax=ax_D)

    # Summary annotation on Panel B
    t_arr = data["t"]
    fy    = data["Fy"]
    ax_B.text(
        0.98, 0.97,
        f"n={len(log)} samples\n"
        f"t = [{t_arr[0]:.2f}, {t_arr[-1]:.2f}] s\n"
        f"Fy ∈ [{fy.min():.1f}, {fy.max():.1f}] N",
        transform=ax_B.transAxes, fontsize=7,
        va="top", ha="right", color="dimgray",
    )

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Dashboard saved → {save_path}")

    return fig
