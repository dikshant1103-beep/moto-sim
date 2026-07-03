#pragma once
// ContactPatch — Hertzian normal pressure distribution (Phase 2.3)
//
// Replaces the single-point Fz contact assumption with a distributed
// pressure field P_n(x,y) over an elliptical contact area.
//
// Physics:
//   A motorcycle tire is approximately toroidal — two principal radii:
//     R_x : outer tire radius   (~0.30 m, determines longitudinal extent)
//     R_y : cross-section radius (~0.06 m, determines lateral extent)
//
//   Hertz theory (spherical approximation per axis):
//     a  = (3/4 · Fz · R_x / E*)^(1/3)   [m]  longitudinal semi-axis
//     b  = (3/4 · Fz · R_y / E*)^(1/3)   [m]  lateral semi-axis
//     P_0 = 3·Fz / (2·π·a·b)             [Pa] peak pressure
//     P(x,y) = P_0 · sqrt(1 − (x/a)² − (y/b)²)  (inside ellipse)
//
//   With camber γ:
//     Contact ellipse shifts laterally: δy = R_y · sin(γ)   [m]
//     The ellipse tilts — outer shoulder carries more load.
//     Effective a, b change slightly with γ (Rao & Pacejka 2002).
//
//   Beam-spring correction (sidewall bending):
//     The tire carcass is not infinitely rigid — sidewall compliance
//     redistributes pressure toward the edges.  Modelled as a correction
//     factor on the Hertzian ellipse:
//       P(x,y) → P_H(x,y) · (1 + β_sw · (x/a)²)
//     where β_sw ≈ 0.05-0.15 (sidewall bending index, compound-dependent).
//     This "flattens" the Hertzian peak and adds a secondary edge-loading component.
//
// Discretisation for LuGreDiscrete (N elements):
//   The longitudinal bristle positions x_i ∈ [−a, a] receive local pressure
//   P_i = ∫_{−b}^{+b} P(x_i, y) dy / (total Fz)  (laterally integrated, normalised).
//
// References:
//   Hertz (1882); Johnson "Contact Mechanics" (1985) Ch. 4
//   Rao & Pacejka (2002) "Tyre contact patch normal pressure distribution"
//      Proc. 17th IAVSD Symposium
//   Gim & Nikravesh (1990) "Analytical model for radial tires" Tire Sci. Technol.

#include <array>
#include <cmath>
#include <algorithm>

namespace moto {

// ── Tire geometry for contact patch computation ───────────────────────────────
struct TireGeometry {
    double R_outer  = 0.299;   // [m]  outer rolling radius
    double R_cross  = 0.060;   // [m]  tyre cross-section radius (half-width profile)
    double E_star   = 6.0e6;   // [Pa] effective contact modulus (rubber composite)
    double nu       = 0.40;    // Poisson ratio of rubber
    double beta_sw  = 0.08;    // sidewall bending index (0=rigid, ~0.15=compliant)
};

// ── Contact patch geometry result ─────────────────────────────────────────────
struct ContactEllipse {
    double a;          // [m]  longitudinal semi-axis
    double b;          // [m]  lateral semi-axis
    double P_0;        // [Pa] peak Hertzian pressure
    double y_shift;    // [m]  lateral shift due to camber
    double area;       // [m²] π·a·b
};

// ── Static geometry functions ─────────────────────────────────────────────────

inline ContactEllipse hertz_ellipse(double Fz, double gamma,
                                     const TireGeometry& g = {})
{
    ContactEllipse e;
    // Effective modulus: E* = E / (2*(1-ν²))
    const double E_eff = g.E_star / (2.0 * (1.0 - g.nu * g.nu));

    // Hertz contact semi-axes from two perpendicular cylinder contacts
    // a (long): governed by R_outer (larger → longer contact patch)
    // b (lat):  governed by R_cross (smaller → narrow contact band)
    e.a = std::cbrt(0.75 * Fz * g.R_outer / E_eff);
    e.b = std::cbrt(0.75 * Fz * g.R_cross / E_eff);

    // Camber shifts contact ellipse laterally [Rao & Pacejka 2002]
    e.y_shift = g.R_cross * std::sin(gamma);

    // Peak Hertzian pressure
    e.area = M_PI * e.a * e.b;
    e.P_0  = 1.5 * Fz / e.area;   // = 3*Fz/(2*π*a*b)

    return e;
}

// ── Pressure at (x, y) in contact ellipse frame ───────────────────────────────

inline double patch_pressure(double x, double y,
                              const ContactEllipse& e,
                              double beta_sw = 0.08)
{
    const double xn = x / (e.a + 1e-9);
    const double yn = (y - e.y_shift) / (e.b + 1e-9);
    const double r2 = xn*xn + yn*yn;
    if (r2 >= 1.0) return 0.0;

    // Hertzian base + sidewall bending correction
    const double P_H = e.P_0 * std::sqrt(1.0 - r2);
    return P_H * (1.0 + beta_sw * xn * xn);   // edge loading from carcass bending
}

// ── Discrete pressure weights for LuGreDiscrete(N elements) ──────────────────
// Integrates pressure laterally at each longitudinal element position,
// normalised so ΣP_i = 1.  Feed directly into LuGreDiscrete::compute().

template <int N>
std::array<double, N> discrete_pressure_weights(const ContactEllipse& e,
                                                   double beta_sw = 0.08,
                                                   int n_y = 20)
{
    std::array<double, N> w{};
    double total = 0.0;

    for (int i = 0; i < N; ++i) {
        // Normalised longitudinal position ξ ∈ [−1, +1]
        const double xi = -1.0 + 2.0 * i / (N - 1);
        const double x  = xi * e.a;

        // Integrate pressure laterally at this x
        const double b_loc = e.b * std::sqrt(std::max(0.0, 1.0 - xi * xi));
        double sum = 0.0;
        for (int j = 0; j < n_y; ++j) {
            const double y = -b_loc + (2.0 * j + 1.0) * b_loc / n_y;
            sum += patch_pressure(x, y, e, beta_sw);
        }
        w[i] = sum * (2.0 * b_loc / n_y);
        total += w[i];
    }
    // Normalise
    if (total > 1e-12)
        for (auto& wi : w) wi /= total;
    return w;
}

// ── Phase 2 combined diagnostics ──────────────────────────────────────────────
struct PatchDiagnostics {
    ContactEllipse ellipse;
    double sliding_fraction;    // fraction of bristles sliding [0,1]
    double peak_pressure_MPa;   // for tyre-wear estimation
    double contact_area_cm2;
};

} // namespace moto
