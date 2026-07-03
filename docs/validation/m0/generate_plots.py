"""
docs/validation/m0/generate_plots.py — M0 validation plots.

Run from moto_sim/ root:
    python docs/validation/m0/generate_plots.py

Outputs:
    docs/validation/m0/fy_vs_alpha.png
    docs/validation/m0/fx_vs_kappa.png
    docs/validation/m0/friction_locus.png
    docs/validation/m0/data/fy_vs_alpha.csv
    docs/validation/m0/data/fx_vs_kappa.csv
    docs/validation/m0/data/friction_locus.csv
"""

import sys, math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

ROOT = Path(__file__).parent.parent.parent.parent  # moto_sim/
sys.path.insert(0, str(ROOT))

from core.tire.pinn_model import DummyTirePINN, TireCoeffs

OUT  = Path(__file__).parent
DATA = OUT / "data"
MEDIUM_YAML = ROOT / "core/tire/pacejka_coeffs/medium.yaml"

coeffs = TireCoeffs.from_yaml(MEDIUM_YAML)
pinn   = DummyTirePINN.from_coeffs(coeffs)

STYLE = {
    "axes.spines.top":    False,
    "axes.spines.right":  False,
    "axes.grid":          True,
    "grid.alpha":         0.30,
    "grid.linestyle":     "--",
    "font.family":        "monospace",
    "axes.titlesize":     11,
    "axes.labelsize":     10,
    "legend.fontsize":    9,
    "figure.dpi":         150,
}
matplotlib.rcParams.update(STYLE)

COLORS = ["#e63946", "#457b9d", "#2a9d8f"]   # red, blue, teal

# ─────────────────────────────────────────────────────────────────────────────
# Plot 1 — Fy vs α  (three load cases)
# ─────────────────────────────────────────────────────────────────────────────
alpha_deg = np.linspace(-25.0, 25.0, 501)
alpha_rad = np.radians(alpha_deg)
Fz_cases  = [735.0, 1470.0, 2940.0]   # ½, 1, 2 × nominal
labels    = ["Fz = 735 N (½ nominal)", "Fz = 1470 N (nominal)", "Fz = 2940 N (2× nominal)"]

fig1, ax1 = plt.subplots(figsize=(8, 5))
csv1_rows = [["alpha_deg"] + [f"Fy_Fz{int(f)}" for f in Fz_cases]]

for Fz, lbl, col in zip(Fz_cases, labels, COLORS):
    Fy_arr = np.array([pinn.predict(Fz=Fz, slip_angle=a)[0] for a in alpha_rad])
    ax1.plot(alpha_deg, Fy_arr, color=col, lw=2.0, label=lbl)
    # Mark peak
    pk_idx = int(np.argmax(np.abs(Fy_arr)))
    ax1.scatter([alpha_deg[pk_idx]], [Fy_arr[pk_idx]],
                color=col, s=60, zorder=5)

for row_i, a in enumerate(alpha_deg):
    row = [f"{a:.3f}"]
    for Fz in Fz_cases:
        row.append(f"{pinn.predict(Fz=Fz, slip_angle=np.radians(a))[0]:.4f}")
    csv1_rows.append(row)

ax1.axhline(0, color="black", lw=0.7)
ax1.axvline(0, color="black", lw=0.7)
ax1.set_xlabel("Slip angle α [°]")
ax1.set_ylabel("Lateral force Fy [N]")
ax1.set_title("Fy vs α — medium compound  (κ=0, γ=0)")
ax1.legend(loc="upper left")
fig1.tight_layout()
fig1.savefig(OUT / "fy_vs_alpha.png", bbox_inches="tight")
plt.close(fig1)
print("[M0 plots] fy_vs_alpha.png saved")

(DATA / "fy_vs_alpha.csv").write_text(
    "\n".join(",".join(r) for r in csv1_rows)
)

# ─────────────────────────────────────────────────────────────────────────────
# Plot 2 — Fx vs κ  (α=0, Fz=1470 N, γ=0)
# ─────────────────────────────────────────────────────────────────────────────
kappas = np.linspace(-1.0, 1.0, 1001)
Fz_nom = 1470.0

Fx_arr = np.array([
    pinn.predict_combined(Fz=Fz_nom, slip_angle=0.0, kappa=k)[0]
    for k in kappas
])

# Saturation reference
F_max = coeffs.mu * Fz_nom
sat_line = F_max * np.ones_like(kappas)

fig2, ax2 = plt.subplots(figsize=(8, 4))
ax2.plot(kappas, Fx_arr,   color=COLORS[0], lw=2.0, label="Fx (combined slip)")
ax2.axhline( F_max, color="gray", lw=1.0, ls="--", label=f"μ·Fz = ±{F_max:.0f} N")
ax2.axhline(-F_max, color="gray", lw=1.0, ls="--")
ax2.axhline(0, color="black", lw=0.7)
ax2.axvline(0, color="black", lw=0.7)
ax2.set_xlabel("Longitudinal slip κ [–]")
ax2.set_ylabel("Longitudinal force Fx [N]")
ax2.set_title("Fx vs κ — medium compound  (α=0, γ=0, Fz=1470 N)")
ax2.legend()
fig2.tight_layout()
fig2.savefig(OUT / "fx_vs_kappa.png", bbox_inches="tight")
plt.close(fig2)
print("[M0 plots] fx_vs_kappa.png saved")

csv2 = ["kappa,Fx"]
for k, fx in zip(kappas, Fx_arr):
    csv2.append(f"{k:.5f},{fx:.4f}")
(DATA / "fx_vs_kappa.csv").write_text("\n".join(csv2))

# ─────────────────────────────────────────────────────────────────────────────
# Plot 3 — Friction locus  (joint α-κ sweep, visual proof B4 fix)
# ─────────────────────────────────────────────────────────────────────────────
alpha_grid = np.linspace(-20.0, 20.0, 41)    # 41 α levels
kappa_grid = np.linspace(-0.80, 0.80, 81)    # 81 κ levels

Fx_locus = []
Fy_locus = []
alpha_locus = []
kappa_locus = []

for a_deg in alpha_grid:
    a_rad = math.radians(a_deg)
    for k in kappa_grid:
        fx, fy, _ = pinn.predict_combined(
            Fz=Fz_nom, slip_angle=a_rad, kappa=k, camber=0.0
        )
        Fx_locus.append(fx)
        Fy_locus.append(fy)
        alpha_locus.append(a_deg)
        kappa_locus.append(k)

Fx_locus = np.array(Fx_locus)
Fy_locus = np.array(Fy_locus)

# Build friction circle reference
theta = np.linspace(0, 2*math.pi, 500)
Fx_circle = F_max * np.cos(theta)
Fy_circle = F_max * np.sin(theta)

fig3, ax3 = plt.subplots(figsize=(7, 7))

sc = ax3.scatter(
    Fx_locus, Fy_locus,
    c=np.abs(np.array(alpha_locus)),   # colour by |α|
    cmap="plasma",
    s=8, alpha=0.6, linewidths=0,
    label="Combined-slip locus",
)
cbar = fig3.colorbar(sc, ax=ax3, fraction=0.046, pad=0.04)
cbar.set_label("|α| [°]")

ax3.plot(Fx_circle, Fy_circle, color="gray", lw=1.5, ls="--",
         label=f"Friction limit μ·Fz = {F_max:.0f} N")
ax3.axhline(0, color="black", lw=0.5)
ax3.axvline(0, color="black", lw=0.5)
ax3.set_aspect("equal")
ax3.set_xlabel("Longitudinal force Fx [N]")
ax3.set_ylabel("Lateral force Fy [N]")
ax3.set_title(
    "Friction locus — Gxα/Gyκ combined slip (B4 fix)\n"
    "medium compound, Fz=1470 N, α∈[−20°,+20°], κ∈[−0.8,+0.8]"
)
ax3.legend(loc="upper right")
fig3.tight_layout()
fig3.savefig(OUT / "friction_locus.png", bbox_inches="tight")
plt.close(fig3)
print("[M0 plots] friction_locus.png saved")

csv3 = ["alpha_deg,kappa,Fx,Fy"]
for a, k, fx, fy in zip(alpha_locus, kappa_locus, Fx_locus, Fy_locus):
    csv3.append(f"{a:.2f},{k:.4f},{fx:.4f},{fy:.4f}")
(DATA / "friction_locus.csv").write_text("\n".join(csv3))

print("[M0 plots] All done — docs/validation/m0/")
