#include "moto/TireModel.h"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <algorithm>

#ifdef HAVE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>

namespace moto {

// Normalisation constants — must match train_pinn.py
// IN_CENTER = [1750, 0, 0, 0],  IN_SCALE = [750, 0.175, 0.25, 0.20]
// OUT_SCALE = [3000, 3000, 65]
// NOTE: model.forward() includes normalise_in and denormalise_out,
//       so the ONNX model takes PHYSICAL inputs and returns PHYSICAL outputs.
//       No manual normalization needed here (learned from Python PINN-3 bug).

// Guard threshold: Pacejka combined-slip with camber can reach ellipse ratio ~2.0
// (camber thrust adds ~50% Fy on top of slip-angle Fy, both contributing to ellipse).
// Set GUARD_SLACK=2.5 to catch only truly pathological PINN outputs (far off-distribution)
// while allowing normal cornering + camber conditions to pass without triggering fallback.
static constexpr float GUARD_SLACK = 2.5f;

struct OnnxTire::Impl {
    Ort::Env          env{ORT_LOGGING_LEVEL_WARNING, "moto_engine"};
    Ort::Session      session{nullptr};
    Ort::AllocatorWithDefaultOptions alloc;
    std::string       input_name;

    // Statistics
    double total_ns    = 0.0;
    int    n_calls     = 0;
    int    n_fallbacks = 0;
    float  bias_Fx = 0.0f, bias_Fy = 0.0f, bias_Mz = 0.0f;

    void load(const std::string& path) {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetInterOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session = Ort::Session(env, path.c_str(), opts);
        auto name_ptr = session.GetInputNameAllocated(0, alloc);
        input_name    = name_ptr.get();
    }

    std::array<float, 3> infer(float Fz, float alpha, float kappa, float gamma) {
        // Physical units in → physical units out (normalisation inside ONNX graph)
        float input_data[4] = { Fz, alpha, kappa, gamma };
        std::array<int64_t, 2> shape = {1, 4};

        auto mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, input_data, 4, shape.data(), shape.size());

        const char* input_names[]  = { input_name.c_str() };
        const char* output_names[] = { "tire_output" };

        auto t0 = std::chrono::high_resolution_clock::now();
        auto outputs = session.Run(Ort::RunOptions{}, input_names, &input_tensor, 1,
                                   output_names, 1);
        auto t1 = std::chrono::high_resolution_clock::now();
        total_ns += std::chrono::duration<double, std::nano>(t1 - t0).count();
        ++n_calls;

        const float* out_ptr = outputs[0].GetTensorData<float>();
        return { out_ptr[0] - bias_Fx,
                 out_ptr[1] - bias_Fy,
                 out_ptr[2] - bias_Mz };
    }
};

OnnxTire::OnnxTire(const std::string& onnx_path, const PacejkaCoeffs& fb)
    : impl_(std::make_unique<Impl>()), fallback_(fb)
{
    impl_->load(onnx_path);
    // Compute zero-slip bias and store for calibration.
    // Any real ML model has a small residual at zero slip; subtract it so
    // the engine sees exactly (0,0,0) at (alpha=0, kappa=0, gamma=0).
    // Use mean Fz = 1250 N as the calibration point (centre of training range).
    auto b = impl_->infer(1250.0f, 0.0f, 0.0f, 0.0f);
    impl_->bias_Fx = b[0];
    impl_->bias_Fy = b[1];
    impl_->bias_Mz = b[2];
}

OnnxTire::~OnnxTire() = default;

void OnnxTire::reset() {
    TireModelBase::reset();
    fallback_.reset();
}

TireForces OnnxTire::compute(double alpha_kin, double kappa,
                               double Fz, double gamma,
                               double vx, double dt)
{
    // ZOH relaxation
    const double vx_s = std::max(std::abs(vx), 1e-3);
    const double decay = std::exp(-vx_s * dt / 0.30);  // sigma from coeffs
    alpha_prime_ = alpha_kin + (alpha_prime_ - alpha_kin) * decay;

    // ONNX inference (physical units in/out)
    auto [Fx_f, Fy_f, Mz_f] = impl_->infer(
        static_cast<float>(Fz),
        static_cast<float>(alpha_prime_),
        static_cast<float>(kappa),
        static_cast<float>(gamma));

    // Physics guard: friction ellipse check
    const double mu = 0.90;   // TODO: read from loaded coefficients
    const double mu_fz2 = (mu * Fz) * (mu * Fz) + 1e-6;
    const double ellipse = (Fx_f * Fx_f + Fy_f * Fy_f) / mu_fz2;

    if (ellipse > GUARD_SLACK) {
        ++impl_->n_fallbacks;
        // Use the already-relaxed alpha_prime_ (not alpha_kin) so the fallback
        // and OnnxTire always share the same slip angle state — prevents
        // force discontinuities when the guard toggles between paths.
        return fallback_.compute_raw(
            static_cast<double>(alpha_prime_),
            kappa, Fz, gamma);
    }

    return { static_cast<double>(Fx_f),
             static_cast<double>(Fy_f),
             static_cast<double>(Mz_f) };
}

double OnnxTire::mean_latency_us() const {
    return impl_->n_calls > 0
           ? impl_->total_ns / impl_->n_calls * 1e-3
           : 0.0;
}

int OnnxTire::n_fallbacks() const { return impl_->n_fallbacks; }

} // namespace moto

#else  // ── Stub when ORT not available ────────────────────────────────────────

namespace moto {

struct OnnxTire::Impl {};

OnnxTire::OnnxTire(const std::string&, const PacejkaCoeffs& fb)
    : impl_(std::make_unique<Impl>()), fallback_(fb)
{
    // ORT not compiled in — will always fall back to Pacejka
}

OnnxTire::~OnnxTire() = default;
void OnnxTire::reset() { TireModelBase::reset(); fallback_.reset(); }

TireForces OnnxTire::compute(double a, double k, double Fz, double g,
                               double vx, double dt) {
    return fallback_.compute(a, k, Fz, g, vx, dt);
}

double OnnxTire::mean_latency_us() const { return 0.0; }
int    OnnxTire::n_fallbacks()     const { return 0;   }

} // namespace moto

#endif // HAVE_ONNXRUNTIME
