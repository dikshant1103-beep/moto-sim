#pragma once
#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace moto {

// ── Tire force output ─────────────────────────────────────────────────────────
struct TireForces {
    double Fx{};   // [N]   longitudinal (traction positive)
    double Fy{};   // [N]   lateral
    double Mz{};   // [N·m] self-aligning torque
};

// ── Pacejka coefficients (C++ equivalent of TireCoeffs in Python) ────────────
struct PacejkaCoeffs {
    double mu         = 0.90;
    double k_alpha    = 12.0;
    double k_gamma    = 0.90;
    double C_pac      = 1.80;
    double E_pac      = 0.00;
    double Kx         = 12.0;
    double tp0        = 0.030;
    double Bt         = 10.0;
    double Ct         = 1.50;
    double K_gamma_Mz = 0.020;
    double Bxa        = 10.0;
    double Cxa        = 1.00;
    double Byk        = 8.00;
    double Cyk        = 1.00;
    double sigma      = 0.30;   // relaxation length [m]
};

// ── Abstract tire model interface ─────────────────────────────────────────────
class TireModelBase {
public:
    virtual ~TireModelBase() = default;

    // Advance relaxation and return combined-slip forces.
    // Signature matches Python TireModel.compute_forces().
    virtual TireForces compute(double alpha_kin, double kappa,
                                double Fz, double gamma,
                                double vx, double dt) = 0;

    virtual void reset() { alpha_prime_ = 0.0; }
    double alpha_prime() const { return alpha_prime_; }

    // Called by EOM once per step before compute() to pass kinematic state that
    // some models need (FullMFTire: wheel spin omega for gyroscopic Mz, psi_dot
    // for turn-slip).  Default is a no-op so PacejkaTire / OnnxTire are unaffected.
    virtual void set_kinematics(double /*omega_wheel*/, double /*psi_dot*/) {}

    // Advance ZOH relaxation without computing forces.
    // Called once per physics step by rk4_step AFTER integration.
    void advance_relaxation(double alpha_kin, double vx, double dt) {
        const double vx_s = std::max(std::abs(vx), 1e-3);
        const double sigma = relaxation_length();
        const double decay = std::exp(-vx_s * dt / sigma);
        alpha_prime_ = alpha_kin + (alpha_prime_ - alpha_kin) * decay;
    }

protected:
    virtual double relaxation_length() const { return 0.30; } // [m] override per model

protected:
    double alpha_prime_ = 0.0;
};

// ── Analytical Pacejka MF (C++ port of pinn_model.py) ────────────────────────
class PacejkaTire final : public TireModelBase {
public:
    explicit PacejkaTire(const PacejkaCoeffs& c = {}) : c_(c) {}

    TireForces compute(double alpha_kin, double kappa,
                       double Fz, double gamma,
                       double vx, double dt) override;

    // Raw combined-slip evaluation at a given relaxed slip angle.
    // Does NOT advance ZOH state — used by OnnxTire fallback guard.
    TireForces compute_raw(double alpha_prime, double kappa,
                            double Fz, double gamma) const;

protected:
    double relaxation_length() const override { return c_.sigma; }

private:
    PacejkaCoeffs c_;
    TireForces pure_slip(double Fz, double alpha_p, double gamma) const;
};

// ── ONNX Runtime tire model ───────────────────────────────────────────────────
// Loads a .onnx file exported from PINN-2 (train_pinn.py).
// Falls back to Pacejka if the friction ellipse is violated.
class OnnxTire final : public TireModelBase {
public:
    OnnxTire(const std::string& onnx_path, const PacejkaCoeffs& fallback = {});
    ~OnnxTire() override;

    TireForces compute(double alpha_kin, double kappa,
                       double Fz, double gamma,
                       double vx, double dt) override;

    void reset() override;

    // Inference latency statistics
    double mean_latency_us() const;
    int    n_fallbacks()     const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    PacejkaTire fallback_;
};

} // namespace moto
