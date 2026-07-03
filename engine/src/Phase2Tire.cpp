#include "moto/Phase2Tire.h"
#include <algorithm>
#include <cmath>

namespace moto {

TireForces Phase2Tire::compute(double alpha_kin, double kappa,
                                  double Fz, double gamma,
                                  double vx, double dt)
{
    const double vx_s = std::max(std::abs(vx), 1e-3);

    // ── ZOH relaxation ────────────────────────────────────────────────────────
    const double decay   = std::exp(-vx_s * dt / cfg_.mf.sigma);
    alpha_prime_ = alpha_kin + (alpha_prime_ - alpha_kin) * decay;

    // ── 2.3 Hertzian contact patch ────────────────────────────────────────────
    const auto ellipse = hertz_ellipse(Fz, gamma, cfg_.geom);
    const auto p_norm  = discrete_pressure_weights<LuGreDiscrete::N>(
        ellipse, cfg_.geom.beta_sw);

    // ── 2.1 Full Motorcycle MF forces ─────────────────────────────────────────
    const TireForces F_mf = mf_.compute_raw(alpha_prime_, kappa, Fz, gamma, vx);

    // ── 2.2 LuGre discrete bristle model ─────────────────────────────────────
    const double V_sx = vx_s * kappa;
    const double V_sy = vx_s * std::sin(alpha_prime_);
    const auto   F_lg = lugre_.compute(Fz, p_norm, V_sx, V_sy, dt);

    // ── Blend Full MF and LuGre ───────────────────────────────────────────────
    // β = 0 → 100% LuGre  (low speed, stick-slip regime)
    // β = 1 → 100% MF     (high speed, validated against Pacejka data)
    const double beta = std::clamp(
        (vx_s - cfg_.blend_v_lo) / (cfg_.blend_v_hi - cfg_.blend_v_lo),
        0.0, 1.0);

    const double Fx = beta * F_mf.Fx + (1.0 - beta) * F_lg[0];
    const double Fy = beta * F_mf.Fy + (1.0 - beta) * F_lg[1];
    // Mz: Full MF gives the physics-based trail + gyro (no LuGre equivalent)
    const double Mz = F_mf.Mz;

    // ── Diagnostics ────────────────────────────────────────────────────────────
    last_diag_.ellipse           = ellipse;
    last_diag_.sliding_fraction  = lugre_.sliding_fraction(Fz);
    last_diag_.peak_pressure_MPa = ellipse.P_0 * 1e-6;
    last_diag_.contact_area_cm2  = ellipse.area * 1e4;

    return { Fx, Fy, Mz };
}

} // namespace moto
