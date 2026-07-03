#include "moto/TireModel.h"
#include <cmath>
#include <algorithm>

namespace moto {

// ── Pacejka MF — C++ port of pinn_model.py DummyTirePINN ────────────────────
// Source: Pacejka (2006) "Tyre and Vehicle Dynamics" Ch. 11 (motorcycle)

TireForces PacejkaTire::pure_slip(double Fz, double alpha_p, double gamma) const {
    const double D   = c_.mu * Fz;
    const double Bcd = c_.k_alpha * Fz;
    const double B   = Bcd / (c_.C_pac * D + 1e-8);
    const double Ba  = B * alpha_p;
    const double phi_ = Ba - c_.E_pac * (Ba - std::atan(Ba));
    const double Fy_pac = D * std::sin(c_.C_pac * std::atan(phi_));
    const double Fy_cam = c_.k_gamma * Fz * gamma;
    const double Fy = std::clamp(Fy_pac + Fy_cam, -D, D);
    const double tp  = c_.tp0 * std::cos(c_.Ct * std::atan(c_.Bt * alpha_p));
    const double Mz  = -Fy_pac * tp + c_.K_gamma_Mz * gamma * Fz;
    return { 0.0, Fy, Mz };
}

TireForces PacejkaTire::compute_raw(double ap, double kappa,
                                      double Fz, double gamma) const
{
    const auto ps  = pure_slip(Fz, ap, gamma);
    const double Fx0 = c_.mu * Fz * std::tanh(c_.Kx * kappa);
    const double Gxa = std::cos(c_.Cxa * std::atan(c_.Bxa * ap));
    const double Gyk = std::cos(c_.Cyk * std::atan(c_.Byk * kappa));
    const double Fx  = Gxa * Fx0;
    const double Fy  = Gyk * ps.Fy;
    const double tp  = c_.tp0 * std::cos(c_.Ct * std::atan(c_.Bt * ap));
    const double Mz  = -Fy * tp + c_.K_gamma_Mz * gamma * Fz;
    return { Fx, Fy, Mz };
}

TireForces PacejkaTire::compute(double alpha_kin, double kappa,
                                  double Fz, double gamma,
                                  double vx, double dt)
{
    const double vx_s = std::max(std::abs(vx), 1e-3);
    const double decay = std::exp(-vx_s * dt / c_.sigma);
    alpha_prime_ = alpha_kin + (alpha_prime_ - alpha_kin) * decay;
    return compute_raw(alpha_prime_, kappa, Fz, gamma);
}

} // namespace moto
