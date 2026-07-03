#include "moto/PINNCoeffFitter.h"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>

namespace moto {

// ── ONNX constructor ──────────────────────────────────────────────────────────
PINNCoeffFitter::PINNCoeffFitter(const std::string& onnx_path,
                                  const FullMFCoeffs& nominal)
    : nominal_(nominal), current_(nominal)
{
#ifdef HAVE_ONNXRUNTIME
    try {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        session_     = Ort::Session(env_, onnx_path.c_str(), opts);
        input_name_  = session_.GetInputNameAllocated(0, alloc_).get();
        output_name_ = session_.GetOutputNameAllocated(0, alloc_).get();
        ort_loaded_  = true;
        diag_.ort_available = true;
    } catch (const std::exception& e) {
        // Graceful fallback: no ONNX, always return nominal
        ort_loaded_ = false;
    }
#endif
}

// ── Ring buffer push ──────────────────────────────────────────────────────────
void PINNCoeffFitter::push(const CoeffObservation& obs)
{
    ring_.push_back(obs);
    if (static_cast<int>(ring_.size()) > WINDOW)
        ring_.pop_front();
}

// ── Per-step call ─────────────────────────────────────────────────────────────
void PINNCoeffFitter::step()
{
    ++step_counter_;
    if (step_counter_ % UPDATE_STEPS == 0 &&
        static_cast<int>(ring_.size()) >= WINDOW / 2)
    {
        run_inference();
    }
}

// ── Build statistics feature vector ──────────────────────────────────────────
void PINNCoeffFitter::compute_stats(float out[N_INPUT]) const
{
    const int n = static_cast<int>(ring_.size());
    if (n == 0) {
        for (int i = 0; i < N_INPUT; ++i) out[i] = 0.0f;
        return;
    }

    // Collect raw normalised observations into 8 per-feature arrays
    float buf[N_FEAT][128];   // max WINDOW=64, extra headroom
    float Fz_sum = 0.f, T_sum = 0.f;

    int idx = 0;
    for (const auto& o : ring_) {
        float raw[N_FEAT] = { o.Fz, o.alpha, o.kappa, o.gamma,
                               o.T_tread, o.Fx, o.Fy, o.Mz };
        for (int f = 0; f < N_FEAT; ++f)
            buf[f][idx] = (raw[f] - FEAT_CENTER[f]) / (FEAT_SCALE[f] + 1e-9f);
        Fz_sum += o.Fz;
        T_sum  += o.T_tread;
        ++idx;
    }

    // For each feature: mean, std, min, max
    int ptr = 0;
    for (int f = 0; f < N_FEAT; ++f) {
        float mn = buf[f][0], mx = buf[f][0], s = 0.f, s2 = 0.f;
        for (int i = 0; i < idx; ++i) {
            float v = buf[f][i];
            s  += v;
            s2 += v * v;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        float mean = s / idx;
        float var  = s2 / idx - mean * mean;
        float std_ = std::sqrt(var > 0.f ? var : 0.f);
        out[ptr++] = mean;   // ptr 0..7
    }
    for (int f = 0; f < N_FEAT; ++f) {
        float s = 0.f, s2 = 0.f;
        for (int i = 0; i < idx; ++i) { s += buf[f][i]; s2 += buf[f][i]*buf[f][i]; }
        float mean = s/idx, var = s2/idx - mean*mean;
        out[ptr++] = std::sqrt(var > 0.f ? var : 0.f);  // ptr 8..15
    }
    for (int f = 0; f < N_FEAT; ++f) {
        float mn = buf[f][0];
        for (int i = 1; i < idx; ++i) if (buf[f][i] < mn) mn = buf[f][i];
        out[ptr++] = mn;    // ptr 16..23
    }
    for (int f = 0; f < N_FEAT; ++f) {
        float mx = buf[f][0];
        for (int i = 1; i < idx; ++i) if (buf[f][i] > mx) mx = buf[f][i];
        out[ptr++] = mx;    // ptr 24..31
    }
    // Two extra: normalised mean Fz and mean T
    out[ptr++] = (Fz_sum/idx - 1470.f) / 750.f;   // ptr 32
    out[ptr++] = (T_sum /idx -   70.f) /  50.f;   // ptr 33
    // ptr == 34
}

// ── Apply scale factors to nominal coefficients ───────────────────────────────
void PINNCoeffFitter::apply_scales(const float scales[N_OUTPUT])
{
    // Clamp to physical bounds
    float s[N_OUTPUT];
    for (int i = 0; i < N_OUTPUT; ++i)
        s[i] = std::clamp(scales[i], SCALE_MIN[i], SCALE_MAX[i]);

    current_         = nominal_;
    current_.mu_y0  *= s[0];   // mu_y
    current_.p_Ky1  *= s[1];   // lateral stiffness
    current_.E_Fy   *= s[2];   // shape curvature
    current_.mu_x0  *= s[3];   // longitudinal grip
    current_.K_x    *= s[4];   // longitudinal stiffness
    current_.tp0    *= s[5];   // pneumatic trail

    // Update diagnostics
    for (int i = 0; i < N_OUTPUT; ++i)
        diag_.scales[i] = s[i];
    ++diag_.n_updates;
}

// ── ONNX inference ────────────────────────────────────────────────────────────
void PINNCoeffFitter::run_inference()
{
#ifdef HAVE_ONNXRUNTIME
    if (!ort_loaded_) return;

    float input[N_INPUT];
    compute_stats(input);

    try {
        auto t0 = std::chrono::high_resolution_clock::now();

        const int64_t shape[] = { 1, N_INPUT };
        auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
            mem_info, input, N_INPUT, shape, 2);

        const char* in_names[]  = { input_name_.c_str() };
        const char* out_names[] = { output_name_.c_str() };
        auto outputs = session_.Run(Ort::RunOptions{nullptr},
                                     in_names,  &in_tensor, 1,
                                     out_names, 1);

        const float* raw = outputs[0].GetTensorData<float>();
        apply_scales(raw);

        auto t1 = std::chrono::high_resolution_clock::now();
        diag_.last_latency_us =
            std::chrono::duration<double, std::micro>(t1 - t0).count();
    } catch (...) {
        // Keep previous coefficients on inference error
    }
#else
    // Without ORT, return nominal + slight random walk to verify wiring
    (void)0;
#endif
}

} // namespace moto
