#pragma once
// Phase2Tire — Full Phase 2 tyre model combining 2.1 + 2.2 + 2.3
//
// Force computation pipeline:
//
//   Fz, γ  ──────────────► ContactPatch (2.3)
//                              │ Hertzian ellipse + sidewall correction
//                              │ → local pressure weights P_i[N]
//                              ▼
//   α', κ, Vx ──────────► LuGreDiscrete (2.2)
//                              │ 10-element bristle model
//                              │ local stick/slip at each element
//                              │ → [Fx_bristle, Fy_bristle]
//                              ▼
//   FullMF blend ────────► Blended output:
//                         Fx = (1−β)·Fx_MF  + β·Fx_bristle
//                         Fy = (1−β)·Fy_MF  + β·Fy_bristle
//                         Mz = Mz_MF  (pneumatic trail + gyro)
//
//   β = blend(Vx) — at low speed (< 1 m/s) trust bristle model completely;
//                    at high speed use MF predominantly (validated at speed).
//
// Why blend? The LuGre bristle model excels at low speed (stick-slip,
// stand-still, ABS thresholds) but is not calibrated to match the full
// Pacejka curve at combined slip in the 5–20 m/s range.  The full MF
// (FullMFTire) is validated at speed but misses pre-sliding dynamics.
// The blend gives the best of both — matching VI-grade's approach.

#include "FullMFTire.h"
#include "LuGreModel.h"
#include "ContactPatch.h"

namespace moto {

struct Phase2Config {
    FullMFCoeffs mf;
    LuGreCoeffs  lugre;
    TireGeometry geom;
    double blend_v_lo = 0.5;   // [m/s] below this → 100% LuGre
    double blend_v_hi = 3.0;   // [m/s] above this → 100% Full MF
};

class Phase2Tire final : public TireModelBase {
public:
    explicit Phase2Tire(const Phase2Config& cfg = {}) : cfg_(cfg),
        mf_(cfg.mf), lugre_(cfg.lugre) {}

    TireForces compute(double alpha_kin, double kappa,
                       double Fz, double gamma,
                       double vx, double dt) override;

    void reset() override {
        TireModelBase::reset();
        mf_.reset();
        lugre_.reset();
    }

    void set_kinematics(double omega, double psi_d) override {
        mf_.set_kinematics(omega, psi_d);
    }
    void set_wheel_omega(double omega) { mf_.set_wheel_omega(omega); }
    void set_yaw_rate   (double psi_d) { mf_.set_yaw_rate(psi_d);   }

    // Diagnostics for dashboard
    PatchDiagnostics last_diagnostics() const { return last_diag_; }
    double sliding_fraction() const { return last_diag_.sliding_fraction; }

protected:
    double relaxation_length() const override { return cfg_.mf.sigma; }

private:
    Phase2Config    cfg_;
    FullMFTire      mf_;
    LuGreDiscrete   lugre_;
    PatchDiagnostics last_diag_{};
};

} // namespace moto
