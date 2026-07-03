#pragma once
// LuGreModel — Contact-patch stick-slip friction (Phase 2.2)
//
// The LuGre model (de Wit, Olsson, Åström, Lischinsky 1995) represents the
// contact patch as a uniform field of microscopic bristles.  The average
// bristle deflection z evolves according to:
//
//     dz/dt  = V_s  −  σ₀ · |V_s| / g(V_s)  · z
//     F      = σ₀ · z  +  σ₁ · ż  +  σ₂ · V_s
//
// where g(V_s) is the Stribeck friction curve:
//     g(V_s) = Fz · [µ_c + (µ_s − µ_c) · exp(−(|V_s|/V_s*)²)]
//
// Key properties:
//   • Pre-sliding regime  (|V_s| ≈ 0): z builds like a spring → F = σ₀ · z
//     (stiction — matches experimental "break-away" force)
//   • Sliding regime      (|V_s| large): z saturates → F ≈ µ_c · Fz
//     (Coulomb sliding friction)
//   • Stribeck transition: smooth µ_c → µ_s → µ_c velocity-dependent curve
//
// 2D extension:
//   State vector z ∈ ℝ² (longitudinal x, lateral y)
//   V_s  = [Vx·κ,  Vx·sin(α')] is the slip velocity vector
//   Output: [Fx, Fy] — replaces Pacejka forces in the combined-slip phase
//
// Discrete bristle variant (Phase 2.2 "two-peak"):
//   N longitudinal elements each with their own z_i, loaded by the local
//   Hertzian pressure P_i from ContactPatch.  Captures leading-edge stick /
//   trailing-edge slip transition — the two-peak response under large slip.
//
// References:
//   de Wit et al. (1995) IEEE Trans. Autom. Control 40(3) 419-425
//   Velenis et al. (2005) Veh. Syst. Dyn. 43 3-29 (2D extension)
//   Deur et al. (2004) "An Extension of the LuGre Tyre Friction Model"
//      J. Dyn. Syst. Meas. Control 126(4) 763-771

#include <array>
#include <cmath>
#include <algorithm>

namespace moto {

// ── LuGre parameters ─────────────────────────────────────────────────────────
struct LuGreCoeffs {
    double sigma0  = 1.50e5;  // [N/m/N]  bristle stiffness per unit Fz
    double sigma1  = 3.00e2;  // [N·s/m/N] bristle damping per unit Fz
    double sigma2  = 0.04;   // [−]       viscous friction coefficient
    double mu_c    = 0.75;   // Coulomb friction coefficient
    double mu_s    = 0.95;   // static (stiction) friction coefficient
    double V_s_star = 0.5;   // [m/s] Stribeck reference velocity
    int    N_elem  = 10;     // number of longitudinal bristle elements
};

// ── Lumped (single-state) 2D LuGre model ─────────────────────────────────────
// One z state per force direction — ~1 µs per call.

class LuGreLumped {
public:
    explicit LuGreLumped(const LuGreCoeffs& c = {}) : c_(c) {}

    void reset() { z_[0] = z_[1] = 0.0; }

    // Compute [Fx, Fy] and advance z state.
    // V_sx = vx * kappa  (longitudinal slip velocity)
    // V_sy = vx * sin(alpha')  (lateral slip velocity)
    std::array<double, 2> compute(double Fz,
                                   double V_sx, double V_sy,
                                   double dt)
    {
        const double V_s  = std::sqrt(V_sx*V_sx + V_sy*V_sy);
        const double g    = g_stribeck(V_s, Fz);

        // Update z (Euler integration of LuGre ODE)
        // dz/dt = Vs - sigma0 * |Vs| / g * z
        const double denom = g + 1e-10;
        for (int i = 0; i < 2; ++i) {
            const double Vs_i = (i == 0) ? V_sx : V_sy;
            const double dz_dt = Vs_i - c_.sigma0 * V_s / denom * z_[i];
            z_[i] += dz_dt * dt;
        }

        // Forces: F = Fz * (sigma0*z + sigma1*dz/dt + sigma2*Vs)
        std::array<double, 2> F;
        for (int i = 0; i < 2; ++i) {
            const double Vs_i  = (i == 0) ? V_sx : V_sy;
            const double dz_dt = Vs_i - c_.sigma0 * V_s / denom * z_[i];
            F[i] = Fz * (c_.sigma0 * z_[i] + c_.sigma1 * dz_dt + c_.sigma2 * Vs_i);
        }
        return F;
    }

    const std::array<double, 2>& z() const { return z_; }

private:
    LuGreCoeffs          c_;
    std::array<double, 2> z_{};

    double g_stribeck(double V_s, double Fz) const {
        const double r = V_s / c_.V_s_star;
        const double mu = c_.mu_c + (c_.mu_s - c_.mu_c) * std::exp(-r * r);
        return mu * Fz;
    }
};

// ── Discrete bristle model (N longitudinal elements) ─────────────────────────
// Captures the "two-peak" partial-slip response: leading contact edge sticks
// while trailing edge slides.  Local state z_i weighted by local pressure P_i.
// Uses SIMD-friendly scalar loop (compiler auto-vectorises at -O3 -march=native).

class LuGreDiscrete {
public:
    static constexpr int N = 10;   // bristle elements (even number, symmetric)

    explicit LuGreDiscrete(const LuGreCoeffs& c = {}) : c_(c) {
        z_x_.fill(0.0); z_y_.fill(0.0);
        // Element positions: uniform from −1 to +1 along contact patch
        for (int i = 0; i < N; ++i)
            xi_[i] = -1.0 + 2.0 * i / (N - 1);
    }

    void reset() { z_x_.fill(0.0); z_y_.fill(0.0); }

    // pressure_dist[N]: normalised local pressure at each element (sums to 1)
    // V_sx, V_sy: global slip velocities [m/s]
    std::array<double, 2> compute(double Fz,
                                   const std::array<double, N>& p_norm,
                                   double V_sx, double V_sy,
                                   double dt)
    {
        const double V_s  = std::sqrt(V_sx*V_sx + V_sy*V_sy);
        double Fx = 0.0, Fy = 0.0;

        for (int i = 0; i < N; ++i) {
            const double Fz_i = Fz * p_norm[i];   // local normal load
            const double g_i  = g_stribeck(V_s, Fz_i);
            const double denom = g_i + 1e-12;

            // x bristle
            const double dz_x = V_sx - c_.sigma0 * V_s / denom * z_x_[i];
            z_x_[i] += dz_x * dt;
            Fx += Fz_i * (c_.sigma0 * z_x_[i] + c_.sigma1 * dz_x + c_.sigma2 * V_sx);

            // y bristle
            const double dz_y = V_sy - c_.sigma0 * V_s / denom * z_y_[i];
            z_y_[i] += dz_y * dt;
            Fy += Fz_i * (c_.sigma0 * z_y_[i] + c_.sigma1 * dz_y + c_.sigma2 * V_sy);
        }
        return { Fx, Fy };
    }

    // Expose bristle deflections for visualisation / telemetry
    const std::array<double, N>& z_x() const { return z_x_; }
    const std::array<double, N>& z_y() const { return z_y_; }
    const std::array<double, N>& xi()  const { return xi_;  }

    // Fraction of elements in sliding state (|z_i| ≈ saturation)
    double sliding_fraction(double Fz) const {
        int n_slide = 0;
        for (int i = 0; i < N; ++i) {
            const double mag = std::sqrt(z_x_[i]*z_x_[i] + z_y_[i]*z_y_[i]);
            const double sat = c_.mu_s * Fz / (c_.sigma0 * N + 1.0);
            if (mag > 0.9 * sat) ++n_slide;
        }
        return static_cast<double>(n_slide) / N;
    }

private:
    LuGreCoeffs              c_;
    std::array<double, N>    z_x_{}, z_y_{};
    std::array<double, N>    xi_{};  // normalised element positions

    double g_stribeck(double V_s, double Fz_i) const {
        const double r  = V_s / c_.V_s_star;
        const double mu = c_.mu_c + (c_.mu_s - c_.mu_c) * std::exp(-r * r);
        return mu * Fz_i;
    }
};

} // namespace moto
