#pragma once
// PINNCoeffFitter — Phase 3.1 real-time tyre coefficient fitting
//
// Loads a trained ONNX model (coeff_fitter.onnx) and updates FullMFCoeffs
// from a ring buffer of recent telemetry frames.
//
// Architecture:  MLP  34 → 128 → 64 → 32 → 6
//   Input  (34) : [mean,std,min,max] × 8 features + [Fz_mean, T_mean]
//                 Features: [Fz, α, κ, γ, T_tread, Fx, Fy, Mz]
//   Output (6)  : scale factors [mu_y, p_Ky1, E_Fy, mu_x, K_x, tp0]
//                 bounded in [0.35, 1.55]; 1.0 = nominal
//
// Update rate: every UPDATE_STEPS = 100 physics steps (10 Hz at 1 kHz)
// Ring buffer: last WINDOW = 64 telemetry frames  (≈ 1.3 s of riding)
//
// Usage (in main loop):
//   PINNCoeffFitter fitter("coeff_fitter.onnx", nominal_coeffs);
//   // per step:
//   fitter.push({Fz_f, alpha_f, kappa_f, gamma_f, T_tread_f,
//                Fx_f, Fy_f, Mz_f});
//   auto& c = fitter.current_coeffs();   // updated every 100 steps
//
// References:
//   Phase 3.1 roadmap; Pacejka (2006) §11.3; training: rig/train_coeff_fitter.py

#include "FullMFTire.h"
#include <array>
#include <deque>
#include <string>
#include <chrono>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace moto {

// ── One telemetry observation ─────────────────────────────────────────────────
struct CoeffObservation {
    float Fz;      // [N]   normal load
    float alpha;   // [rad] slip angle
    float kappa;   // [–]   slip ratio
    float gamma;   // [rad] camber
    float T_tread; // [°C]  tread temperature
    float Fx;      // [N]
    float Fy;      // [N]
    float Mz;      // [N·m]
};

// ── Diagnostic output ─────────────────────────────────────────────────────────
struct CoeffFitterDiag {
    float scales[6]{1,1,1,1,1,1}; // [mu_y, p_Ky1, E_Fy, mu_x, K_x, tp0]
    int   n_updates = 0;
    double last_latency_us = 0.0;
    bool   ort_available = false;
};

// ── Names ─────────────────────────────────────────────────────────────────────
static constexpr const char* SCALE_NAMES[6] = {
    "mu_y", "p_Ky1", "E_Fy", "mu_x", "K_x", "tp0"
};

// ── Normalisation (must match generate_coeff_dataset.py) ─────────────────────
static constexpr float FEAT_CENTER[8] = { 1470.f, 0.f, 0.f, 0.f, 70.f, 0.f, 0.f, 0.f };
static constexpr float FEAT_SCALE [8] = {  750.f, 0.2f, 0.25f, 0.30f, 50.f, 1500.f, 1500.f, 30.f };

// ── Scale bounds ──────────────────────────────────────────────────────────────
static constexpr float SCALE_MIN[6] = { 0.45f, 0.60f, 0.70f, 0.45f, 0.70f, 0.60f };
static constexpr float SCALE_MAX[6] = { 1.15f, 1.40f, 1.30f, 1.15f, 1.30f, 1.50f };

// ── PINNCoeffFitter ───────────────────────────────────────────────────────────

class PINNCoeffFitter {
public:
    static constexpr int WINDOW        = 64;   // observations in ring buffer
    static constexpr int UPDATE_STEPS  = 100;  // physics steps between updates
    static constexpr int N_FEAT        = 8;
    static constexpr int N_INPUT       = 34;   // 4×8 + 2
    static constexpr int N_OUTPUT      = 6;

    // Construct without ORT (returns nominal coefficients always)
    explicit PINNCoeffFitter(const FullMFCoeffs& nominal = {})
        : nominal_(nominal), current_(nominal) {}

    // Construct with ONNX model
    PINNCoeffFitter(const std::string& onnx_path, const FullMFCoeffs& nominal = {});

    // Push one observation into the ring buffer
    void push(const CoeffObservation& obs);

    // Call every physics step — runs inference every UPDATE_STEPS pushes
    void step();

    // Returns the currently active FullMFCoeffs (updated at 10 Hz)
    const FullMFCoeffs& current_coeffs() const { return current_; }

    // Diagnostic snapshot
    const CoeffFitterDiag& diagnostics() const { return diag_; }

    bool ort_loaded() const { return ort_loaded_; }

private:
    FullMFCoeffs nominal_;
    FullMFCoeffs current_;
    CoeffFitterDiag diag_;

    std::deque<CoeffObservation> ring_;
    int step_counter_ = 0;
    bool ort_loaded_  = false;

    void run_inference();
    void compute_stats(float out[N_INPUT]) const;
    void apply_scales(const float scales[N_OUTPUT]);

#ifdef HAVE_ONNXRUNTIME
    Ort::Env            env_{ ORT_LOGGING_LEVEL_WARNING, "coeff_fitter" };
    Ort::Session        session_{ nullptr };
    Ort::AllocatorWithDefaultOptions alloc_;
    std::string         input_name_;
    std::string         output_name_;
#endif
};

} // namespace moto
