#pragma once
// FullMFTire — Motorcycle-specific Pacejka Magic Formula (Pacejka 2006, Ch. 11.6)
//
// Key differences from the simplified Pacejka in PacejkaTire (Phase 1):
//
//   1. Large-camber correction to D_y:  µ_y(γ) = µ_y0 * (1 − p_Dy3 * γ²)
//      At γ = 55° lean: ~9% grip reduction.  Standard car/bicycle MF ignores this.
//
//   2. Nonlinear camber thrust (full MF curve, not linear in γ):
//      Fy_γ = D_γ * sin(C_γ * atan(B_γ * γ − E_γ*(B_γ*γ − atan(B_γ*γ))))
//
//   3. Gyroscopic aligning moment from wheel spin angular momentum:
//      Mz_gyro = C_gyro * I_spin * Ω * Vx * cos(γ) * α
//      Creates a self-steering tendency at speed under lean — VI-grade feature.
//
//   4. Turn-slip (spin about contact-patch normal during cornering):
//      φ_t = ψ̇ / Vx  (1/m, curvature of path)
//      κ_s = −R * φ_t  (dimensionless spin slip)
//      Adds lateral force component proportional to φ_t.
//
// References:
//   Pacejka (2006) §11.3–11.6 (motorcycle steady-state)
//   Cossalter (2006) Ch. 2.3–2.5 (camber and turn-slip)
//   de Vries & Pacejka (1998) "Motorcycle tyre measurements" Vehicle System Dynamics

#include "TireModel.h"
#include <cmath>
#include <array>

namespace moto {

// ── Full Motorcycle MF parameter set ────────────────────────────────────────

struct FullMFCoeffs {
    // Reference conditions
    double Fz0  = 1470.0;   // [N]  reference normal load
    double V0   = 10.0;     // [m/s] reference speed (normalisation)

    // ── Lateral (Fy) ─────────────────────────────────────────────────────────
    double mu_y0  = 0.90;   // peak lateral friction at Fz0, γ=0
    double p_Dy3  = 0.10;   // γ² reduction of µ_y (large-camber correction)
    double p_Ky1  = 13.5;   // lateral stiffness amplitude
    double p_Ky2  = 2.00;   // stiffness Fz normalisation exponent
    double C_Fy   = 1.35;   // shape factor (motorcycle: slightly > car 1.3)
    double E_Fy   = -0.80;  // curvature factor (negative → soft peak)
    double S_Vy0  = 0.0;    // lateral force offset at α=0 (bias)

    // Nonlinear camber thrust coefficients (replaces k_gamma * Fz * γ)
    double p_Ky6  = 1.20;   // camber stiffness amplitude
    double p_Ky7  = 0.20;   // camber stiffness Fz sensitivity
    double B_gamma = 4.00;  // camber MF — stiffness factor
    double C_gamma = 1.10;  // camber MF — shape factor
    double E_gamma = -0.50; // camber MF — curvature

    // ── Longitudinal (Fx) ────────────────────────────────────────────────────
    double mu_x0  = 0.95;   // peak longitudinal friction
    double p_Dx3  = 0.08;   // γ² reduction of µ_x
    double K_x    = 11.0;   // longitudinal slip stiffness (tanh-based)

    // ── Aligning torque (Mz) ─────────────────────────────────────────────────
    double tp0    = 0.030;  // [m] max pneumatic trail
    double Bt     = 10.0;
    double Ct     = 1.50;
    double K_gamma_Mz = 0.020;
    // Gyroscopic Mz
    double C_gyro  = 0.003; // [(N·m·s)/rad] gyroscopic Mz coupling
    double I_spin  = 0.38;  // [kg·m²] wheel spin inertia

    // ── Turn-slip (spin about contact normal) ────────────────────────────────
    double B_spin  = 3.0;   // turn-slip MF stiffness
    double C_spin  = 1.5;   // turn-slip MF shape
    double D_spin  = 0.12;  // turn-slip amplitude (fraction of Fz0)
    double R_eff   = 0.299; // [m] effective rolling radius

    // ── Combined slip weighting (Pacejka §4.3.2) ─────────────────────────────
    double Bxa = 10.0; double Cxa = 1.00;
    double Byk =  8.0; double Cyk = 1.00;

    // ── Relaxation ────────────────────────────────────────────────────────────
    double sigma = 0.30;    // [m]

    // Derived
    double dfz(double Fz) const { return (Fz - Fz0) / Fz0; }
};

// ── Full Motorcycle MF tire model ────────────────────────────────────────────

class FullMFTire final : public TireModelBase {
public:
    explicit FullMFTire(const FullMFCoeffs& c = {}) : c_(c) {}

    // compute() — same API as TireModelBase (ZOH relaxation + full MF)
    // Extra inputs via set_spin_rate() / set_yaw_rate()
    TireForces compute(double alpha_kin, double kappa,
                       double Fz, double gamma,
                       double vx, double dt) override;

    // TireModelBase hook — called by EOM each step before compute().
    void set_kinematics(double omega_wheel, double psi_dot) override {
        omega_spin_ = omega_wheel;
        psi_dot_    = psi_dot;
    }

    // Direct setters kept for external use
    void set_wheel_omega(double omega) { omega_spin_ = omega; }
    void set_yaw_rate(double psi_dot)  { psi_dot_    = psi_dot; }

    // Pure-slip query (no state advance) — for benchmarking / dataset generation
    TireForces compute_raw(double alpha_prime, double kappa,
                            double Fz, double gamma,
                            double vx) const;

    const FullMFCoeffs& coeffs() const { return c_; }

protected:
    double relaxation_length() const override { return c_.sigma; }

private:
    FullMFCoeffs c_;
    double omega_spin_ = 0.0;  // [rad/s] wheel angular velocity
    double psi_dot_    = 0.0;  // [rad/s] chassis yaw rate

    // Sub-functions
    double Fy_pure   (double Fz, double alpha, double gamma) const;
    double Fy_camber (double Fz, double gamma) const;
    double Fx_pure   (double Fz, double kappa, double gamma) const;
    double Mz_pneum  (double Fz, double alpha, double Fy_comb) const;
    double Mz_gyro   (double Fz, double alpha, double gamma, double vx) const;
    double Fy_turnspl(double Fz, double vx) const;
};

} // namespace moto
