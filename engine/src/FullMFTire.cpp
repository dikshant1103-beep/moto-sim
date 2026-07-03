#include "moto/FullMFTire.h"
#include <algorithm>
#include <cmath>

namespace moto {

// ── Pure lateral force Fy0(Fz, α, γ) ─────────────────────────────────────────
// Pacejka 2006 §11.3, Eq. 11.8–11.15
//
// Key motorcycle extension: large-camber correction to D_y via γ².
// µ_y(γ) = µ_y0 * (1 − p_Dy3 * γ²)   [Pacejka §11.3, Eq. 11.9]
// At γ=55° (0.96 rad): µ_y drops by ~9% vs zero-lean — critical for cornering limit.

double FullMFTire::Fy_pure(double Fz, double alpha, double gamma) const {
    // Large-camber correction to peak friction [Pacejka §11.3]
    const double mu_y = c_.mu_y0 * (1.0 - c_.p_Dy3 * gamma * gamma);
    const double D    = mu_y * Fz;

    // Lateral stiffness [Pacejka §11.10]
    // K_Fy = p_Ky1 * Fz0 * sin(2 * atan(Fz / (p_Ky2 * Fz0))) * cos(γ)
    const double K_Fy = c_.p_Ky1 * c_.Fz0
                        * std::sin(2.0 * std::atan(Fz / (c_.p_Ky2 * c_.Fz0)))
                        * std::cos(gamma);
    const double B    = K_Fy / (c_.C_Fy * D + 1e-8);
    const double Ba   = B * alpha;
    const double phi  = Ba - c_.E_Fy * (Ba - std::atan(Ba));
    return D * std::sin(c_.C_Fy * std::atan(phi)) + c_.S_Vy0;
}

// ── Nonlinear camber thrust Fy_γ(Fz, γ) ──────────────────────────────────────
// Replaces the linear k_gamma * Fz * γ approximation.
// Uses a full MF curve in γ — captures saturation at large lean angles.
// Pacejka 2006 §11.4, Cossalter (2006) §2.5

double FullMFTire::Fy_camber(double Fz, double gamma) const {
    const double dfz  = c_.dfz(Fz);
    // Camber stiffness amplitude (Fz-dependent)
    const double D_gamma = (c_.p_Ky6 + c_.p_Ky7 * dfz) * Fz;
    const double Bg = c_.B_gamma;
    const double Cg = c_.C_gamma;
    const double Eg = c_.E_gamma;

    const double Bg_g  = Bg * gamma;
    const double phi_g = Bg_g - Eg * (Bg_g - std::atan(Bg_g));
    return D_gamma * std::sin(Cg * std::atan(phi_g));
}

// ── Pure longitudinal force Fx0(Fz, κ, γ) ────────────────────────────────────
// Large-camber correction to µ_x: reduces traction at high lean (same mechanism as Fy).

double FullMFTire::Fx_pure(double Fz, double kappa, double gamma) const {
    const double mu_x = c_.mu_x0 * (1.0 - c_.p_Dx3 * gamma * gamma);
    return mu_x * Fz * std::tanh(c_.K_x * kappa);
}

// ── Pneumatic trail + aligning torque Mz_trail ───────────────────────────────

double FullMFTire::Mz_pneum(double Fz, double alpha, double Fy_comb) const {
    const double tp = c_.tp0 * std::cos(c_.Ct * std::atan(c_.Bt * alpha));
    return -Fy_comb * tp;
}

// ── Gyroscopic aligning moment Mz_gyro ───────────────────────────────────────
// When the wheel is spinning (Ω) and the bike is leaned (γ), the angular
// momentum H = I_spin * Ω creates a gyroscopic moment when the steer angle α
// changes. This manifests as an additional self-aligning torque.
// Pacejka 2006 §11.6 gyroscopic spin effect, simplified to first order.
// M_z_gyro ≈ C_gyro * I_spin * Ω * Vx * cos(γ) * α  [N·m]

double FullMFTire::Mz_gyro(double Fz, double alpha, double gamma, double vx) const {
    return c_.C_gyro * c_.I_spin * omega_spin_ * vx
           * std::cos(gamma) * alpha;
}

// ── Turn-slip lateral force Fy_spin ──────────────────────────────────────────
// When the bike corners (ψ̇ ≠ 0), the contact patch rotates about the road
// normal — this creates a spin slip κ_s = −R * ψ̇ / Vx.
// Equivalent to a "self-steer" camber-like force from the tire footprint spinning.
// Cossalter (2006) §2.5, Pacejka (2006) §11.5

double FullMFTire::Fy_turnspl(double Fz, double vx) const {
    if (std::abs(vx) < 0.5 || std::abs(psi_dot_) < 1e-4) return 0.0;
    const double kappa_s = -c_.R_eff * psi_dot_ / vx;  // spin slip [−]
    const double D_s     = c_.D_spin * Fz;
    const double Bs_k    = c_.B_spin * kappa_s;
    const double phi_s   = Bs_k - 0.0 * (Bs_k - std::atan(Bs_k));  // E_s = 0
    return D_s * std::sin(c_.C_spin * std::atan(phi_s));
}

// ── Full combined-slip force evaluation (no state advance) ────────────────────

TireForces FullMFTire::compute_raw(double alpha_p, double kappa,
                                     double Fz, double gamma, double vx) const {
    // ── Pure-slip forces ──────────────────────────────────────────────────────
    const double Fy0_alpha  = Fy_pure  (Fz, alpha_p, gamma);
    const double Fy0_gamma  = Fy_camber(Fz, gamma);
    const double Fy0        = Fy0_alpha + Fy0_gamma;  // total pure-slip lateral
    const double Fx0        = Fx_pure  (Fz, kappa, gamma);

    // ── Turn-slip addition ────────────────────────────────────────────────────
    const double Fy_ts = Fy_turnspl(Fz, vx);

    // ── Combined slip (Pacejka §4.3.2, C∞ smooth) ────────────────────────────
    const double Gxa = std::cos(c_.Cxa * std::atan(c_.Bxa * alpha_p));
    const double Gyk = std::cos(c_.Cyk * std::atan(c_.Byk * kappa));

    const double Fx = Gxa * Fx0;
    const double Fy = Gyk * Fy0 + Fy_ts;

    // ── Aligning torque (pneumatic trail + camber + gyroscopic) ───────────────
    const double Mz_t  = Mz_pneum(Fz, alpha_p, Fy);
    const double Mz_g  = Mz_gyro (Fz, alpha_p, gamma, vx);
    const double Mz_c  = c_.K_gamma_Mz * gamma * Fz;   // camber Mz offset
    const double Mz    = Mz_t + Mz_g + Mz_c;

    return { Fx, Fy, Mz };
}

// ── Public API — ZOH relaxation + full MF ─────────────────────────────────────

TireForces FullMFTire::compute(double alpha_kin, double kappa,
                                 double Fz, double gamma,
                                 double vx, double dt)
{
    // ZOH relaxation (dt=0 freezes state for RK4 sub-steps — same contract as PacejkaTire)
    const double vx_s  = std::max(std::abs(vx), 1e-3);
    const double decay = std::exp(-vx_s * dt / c_.sigma);
    alpha_prime_ = alpha_kin + (alpha_prime_ - alpha_kin) * decay;

    return compute_raw(alpha_prime_, kappa, Fz, gamma, vx);
}

} // namespace moto
