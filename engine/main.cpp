// engine/main.cpp — Phase 1+2 smoke test + real-time streaming mode + validation support
//
// Usage:
//   ./moto_engine                                  (5s smoke test, Pacejka)
//   ./moto_engine --pinn /path/tire_pinn_medium.onnx
//   ./moto_engine --full-mf                        (Phase 2.1 — full motorcycle Pacejka MF)
//   ./moto_engine --phase2                         (Phase 2.1+2.2+2.3 — FullMF + LuGre + contact patch)
//   ./moto_engine --stream                         (real-time WS bridge mode)
//   ./moto_engine --stream --full-mf
//   ./moto_engine --v0 10.0 --t 5.0
//   ./moto_engine --headless --v0 10.0 --phi0 0.5 --duration 2.0
//   ./moto_engine --bike /path/cbr600rr.yaml
//   ./moto_engine --road /path/wet_asphalt.yaml

#include "moto/BikeParams.h"
#include "moto/RoadParams.h"
#include "moto/State.h"
#include "moto/EOM.h"
#include "moto/TireModel.h"
#include "moto/Kinematics.h"
#include "moto/FullMFTire.h"
#include "moto/Phase2Tire.h"
#include "moto/PINNCoeffFitter.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

using namespace moto;
using namespace std::chrono;

// ── JSON helpers ──────────────────────────────────────────────────────────────

static std::string jf(double v, int dp = 4) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(dp) << v;
    return os.str();
}

enum class TireMode { Pacejka, PINN, FullMF, Phase2 };

// ── Phase 3.1 coefficient fitters — accessible from all run functions ─────
// Set by main() after construction; nullptr = no fitter active.
namespace { PINNCoeffFitter* g_cf_front = nullptr; PINNCoeffFitter* g_cf_rear = nullptr; }

static std::string tire_mode_label(TireMode m) {
    switch (m) {
        case TireMode::PINN:    return "C++ 13-DOF (PINN)";
        case TireMode::FullMF:  return "C++ 13-DOF (FullMF)";
        case TireMode::Phase2:  return "C++ 13-DOF (Phase2)";
        default:                return "C++ 13-DOF (Pacejka)";
    }
}

static std::string telemetry_to_json(const Telemetry& tel,
                                      const RiderInput& inp,
                                      const StateVec& s,
                                      const KinematicState& ks,
                                      double front_ap, double rear_ap,
                                      TireMode tire_mode,
                                      const RoadParams& road,
                                      double T_f_tread = 85.0, double T_r_tread = 85.0,
                                      double T_f_carcass = 80.0, double T_r_carcass = 80.0,
                                      double f_slide = 0.0, double r_slide = 0.0,
                                      double f_area_cm2 = 0.0, double r_area_cm2 = 0.0,
                                      const float* cf_scales = nullptr, int cf_updates = 0,
                                      double cf_latency_us = 0.0)
{
    // Must exactly match the flat dict produced by moto_sim_main._flatten()
    // so all dashboard panels work without changes.
    std::string j = "{";
    auto kv = [&](const char* k, double v, int dp=4, bool last=false) {
        j += "\""; j += k; j += "\":"; j += jf(v, dp);
        if (!last) j += ",";
    };

    kv("t",           tel.t,               3);
    kv("roll",        tel.roll);
    kv("pitch",       tel.pitch);
    kv("yaw",         tel.yaw);
    kv("roll_rate",   tel.roll_rate > 0 ? tel.roll_rate : s[PHI_DOT]);
    kv("yaw_rate",    tel.yaw_rate);
    kv("vx",          tel.vx);
    kv("vy",          tel.vy);
    kv("ax",          tel.ax);
    kv("ay",          tel.ay);
    // Front tire
    kv("f_alpha",      tel.alpha_f);
    kv("f_kappa",      tel.kappa_f);
    kv("f_Fz",         tel.Fz_f,      2);
    kv("f_Fx",         tel.Fx_f,      2);
    kv("f_Fy",         tel.Fy_f,      2);
    kv("f_Mz",         tel.Mz_f,      3);
    kv("f_alpha_prime", front_ap);
    // Rear tire
    kv("r_alpha",      tel.alpha_r);
    kv("r_kappa",      tel.kappa_r);
    kv("r_Fz",         tel.Fz_r,      2);
    kv("r_Fx",         tel.Fx_r,      2);
    kv("r_Fy",         tel.Fy_r,      2);
    kv("r_Mz",         tel.Mz_r,      3);
    kv("r_alpha_prime", rear_ap);
    // Inputs
    kv("throttle",     inp.throttle);
    kv("brake_front",  inp.brake_front);
    kv("brake_rear",   inp.brake_rear);
    kv("steer_torque", inp.steer_torque);
    kv("steer_angle",  tel.steer_angle);
    kv("lean_delta",   inp.lean_delta);
    // Suspension
    kv("fork_travel",  tel.fork_travel,   4);
    kv("rear_travel",  tel.rear_travel,   4);
    // Road (actual road params)
    kv("road_mu_f",    road.mu,   2);
    kv("road_mu_r",    road.mu,   2);
    kv("road_bank_f",  road.banking, 3);
    kv("road_bank_r",  road.banking, 3);
    // M5 thermal (placeholder — not modelled in C++ engine Phase 1)
    kv("f_T_tread", T_f_tread, 1); kv("f_T_carcass", T_f_carcass, 1);
    kv("f_wear",       0.0, 4);  kv("f_pressure",  240.0, 1);
    kv("r_T_tread", T_r_tread, 1); kv("r_T_carcass", T_r_carcass, 1);
    kv("r_wear",       0.0, 4);  kv("r_pressure",  240.0, 1);
    // Phase 1 diagnostics
    kv("effective_trail",     tel.effective_trail,     4);
    kv("effective_wheelbase", tel.effective_wheelbase, 4);
    // Road condition telemetry
    kv("road_mu",       road.mu,       2);
    kv("road_banking",  road.banking,  4);
    kv("road_gradient", road.gradient, 4);
    // Phase 2 diagnostics (non-zero when --full-mf or --phase2)
    kv("f_sliding_frac",   f_slide,     3);
    kv("r_sliding_frac",   r_slide,     3);
    kv("f_contact_cm2",    f_area_cm2,  2);
    kv("r_contact_cm2",    r_area_cm2,  2);
    // Phase 3.1 coefficient fitter (non-zero when --coeff-fit)
    if (cf_scales) {
        kv("cf_mu_y",    cf_scales[0], 4);
        kv("cf_p_Ky1",   cf_scales[1], 4);
        kv("cf_E_Fy",    cf_scales[2], 4);
        kv("cf_mu_x",    cf_scales[3], 4);
        kv("cf_K_x",     cf_scales[4], 4);
        kv("cf_tp0",     cf_scales[5], 4);
        kv("cf_updates", static_cast<double>(cf_updates), 0);
        kv("cf_lat_us",  cf_latency_us, 1);
    } else {
        kv("cf_mu_y",  1.0, 4); kv("cf_p_Ky1", 1.0, 4);
        kv("cf_E_Fy",  1.0, 4); kv("cf_mu_x",  1.0, 4);
        kv("cf_K_x",   1.0, 4); kv("cf_tp0",   1.0, 4);
        kv("cf_updates", 0.0, 0); kv("cf_lat_us", 0.0, 1);
    }
    j += "\"engine\":\"";
    j += tire_mode_label(tire_mode);
    j += "\"}";
    return j;
}

// Parse simple JSON key:value for rider commands from stdin
static void parse_command(const std::string& line, RiderInput& inp) {
    auto get = [&](const char* key) -> double {
        auto pos = line.find(key);
        if (pos == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
        auto colon = line.find(':', pos);
        if (colon == std::string::npos) return std::numeric_limits<double>::quiet_NaN();
        try { return std::stod(line.substr(colon + 1)); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
    };
    auto set = [](double v, double& target) { if (!std::isnan(v)) target = v; };
    set(get("throttle"),    inp.throttle);
    set(get("brake_front"), inp.brake_front);
    set(get("brake_rear"),  inp.brake_rear);
    set(get("steer_torque"),inp.steer_torque);
    set(get("lean_delta"),  inp.lean_delta);
}

// ── Headless duration mode ────────────────────────────────────────────────────
// Runs sim for exactly `duration` seconds, printing JSON at 50 Hz, then exits.

static int run_headless(BikeParams& p,
                         std::unique_ptr<TireModelBase>& front_tire,
                         std::unique_ptr<TireModelBase>& rear_tire,
                         const RoadParams& road,
                         TireMode tire_mode, double v0, double phi0_deg, double duration)
{
    StateVec s = make_initial_state(v0);
    // Apply initial roll perturbation
    if (phi0_deg != 0.0)
        s[PHI] = phi0_deg * (M_PI / 180.0);

    RiderInput inp;
    constexpr double DT      = 0.001;   // 1 kHz physics
    constexpr int    WS_EVERY = 20;     // output at 50 Hz

    const int n_steps = static_cast<int>(duration / DT);
    int step = 0;
    double t = 0.0;

    // Disable stdout buffering for low-latency JSON lines
    std::cout.setf(std::ios::unitbuf);

    for (step = 0; step < n_steps; ++step) {
        t = step * DT;

        // Output telemetry at 50 Hz
        if (step % WS_EVERY == 0) {
            Telemetry tel;
            eom(t, s, p, inp, *front_tire, *rear_tire, road, &tel);
            tel.t = t; tel.roll_rate = s[PHI_DOT];
            auto ks = compute_kinematics(p, s, 0.0, 0.0);
            // Phase 3.1: push observation + step fitter
            if (g_cf_front) {
                g_cf_front->push({ (float)tel.Fz_f, (float)tel.alpha_f, (float)tel.kappa_f,
                                   (float)ks.gamma_front, 85.0f,
                                   (float)tel.Fx_f, (float)tel.Fy_f, (float)tel.Mz_f });
                g_cf_front->step();
            }
            if (g_cf_rear) {
                g_cf_rear->push({ (float)tel.Fz_r, (float)tel.alpha_r, (float)tel.kappa_r,
                                  (float)ks.gamma_rear, 85.0f,
                                  (float)tel.Fx_r, (float)tel.Fy_r, (float)tel.Mz_r });
                g_cf_rear->step();
            }
            auto* p2f = dynamic_cast<Phase2Tire*>(front_tire.get());
            auto* p2r = dynamic_cast<Phase2Tire*>(rear_tire.get());
            const auto* cfd = g_cf_front ? &g_cf_front->diagnostics() : nullptr;
            std::cout << telemetry_to_json(tel, inp, s, ks,
                                            front_tire->alpha_prime(), rear_tire->alpha_prime(),
                                            tire_mode, road, 85.0, 85.0, 80.0, 80.0,
                                            p2f ? p2f->sliding_fraction() : 0.0,
                                            p2r ? p2r->sliding_fraction() : 0.0,
                                            p2f ? p2f->last_diagnostics().contact_area_cm2 : 0.0,
                                            p2r ? p2r->last_diagnostics().contact_area_cm2 : 0.0,
                                            cfd ? cfd->scales : nullptr,
                                            cfd ? cfd->n_updates : 0,
                                            cfd ? cfd->last_latency_us : 0.0) << '\n';
        }

        // Physics step
        s = rk4_step(t, s, DT, p, inp, *front_tire, *rear_tire, road);
    }
    return 0;
}

// ── Streaming mode ────────────────────────────────────────────────────────────

static void run_streaming(BikeParams& p,
                           std::unique_ptr<TireModelBase>& front_tire,
                           std::unique_ptr<TireModelBase>& rear_tire,
                           const RoadParams& road,
                           TireMode tire_mode, double v0)
{
    StateVec s = make_initial_state(v0);
    RiderInput inp;
    std::mutex inp_mtx;
    std::atomic<bool> running{true};
    std::atomic<bool> paused{false};
    std::atomic<bool> do_reset{false};

    // stdin reader thread — reads JSON commands from Python bridge
    std::thread stdin_thread([&]() {
        std::string line;
        while (running && std::getline(std::cin, line)) {
            if (line.empty()) continue;
            if (line.find("\"cmd\":\"pause\"")  != std::string::npos) { paused = true;  continue; }
            if (line.find("\"cmd\":\"resume\"") != std::string::npos) { paused = false; continue; }
            if (line.find("\"cmd\":\"reset\"")  != std::string::npos) { do_reset = true; continue; }
            std::lock_guard<std::mutex> lk(inp_mtx);
            parse_command(line, inp);
        }
        running = false;
    });

    constexpr double DT      = 0.001;   // 1 kHz physics
    constexpr int    WS_EVERY = 20;     // output at 50 Hz
    int step = 0;
    double t = 0.0;

    // Simple first-order tire thermal model
    constexpr double T_AMB       = 25.0;
    constexpr double BETA_COOL   = 0.30;
    constexpr double ALPHA_HEAT  = 0.0015;
    double T_f_tread = T_AMB, T_r_tread = T_AMB;
    double T_f_carcass = T_AMB, T_r_carcass = T_AMB;

    // Disable stdout buffering for low-latency JSON lines
    std::cout.setf(std::ios::unitbuf);

    auto wall_start = high_resolution_clock::now();

    while (running) {
        if (do_reset) {
            s = make_initial_state(v0);
            front_tire->reset();
            rear_tire->reset();
            step = 0; t = 0.0;
            T_f_tread = T_r_tread = T_f_carcass = T_r_carcass = T_AMB;
            do_reset = false;
            wall_start = high_resolution_clock::now();
        }

        if (paused) {
            std::this_thread::sleep_for(milliseconds(5));
            continue;
        }

        // Capture current inputs
        RiderInput cur_inp;
        { std::lock_guard<std::mutex> lk(inp_mtx); cur_inp = inp; }

        // Output telemetry at 50 Hz
        if (step % WS_EVERY == 0) {
            Telemetry tel;
            eom(t, s, p, cur_inp, *front_tire, *rear_tire, road, &tel);
            tel.t = t; tel.roll_rate = s[PHI_DOT];
            auto ks = compute_kinematics(p, s, 0.0, 0.0);
            // Phase 3.1 fitter
            if (g_cf_front) {
                g_cf_front->push({ (float)tel.Fz_f, (float)tel.alpha_f, (float)tel.kappa_f,
                                   (float)ks.gamma_front, (float)T_f_tread,
                                   (float)tel.Fx_f, (float)tel.Fy_f, (float)tel.Mz_f });
                g_cf_front->step();
            }
            if (g_cf_rear) {
                g_cf_rear->push({ (float)tel.Fz_r, (float)tel.alpha_r, (float)tel.kappa_r,
                                  (float)ks.gamma_rear, (float)T_r_tread,
                                  (float)tel.Fx_r, (float)tel.Fy_r, (float)tel.Mz_r });
                g_cf_rear->step();
            }
            auto* p2f = dynamic_cast<Phase2Tire*>(front_tire.get());
            auto* p2r = dynamic_cast<Phase2Tire*>(rear_tire.get());
            const auto* cfd = g_cf_front ? &g_cf_front->diagnostics() : nullptr;
            std::cout << telemetry_to_json(tel, cur_inp, s, ks,
                                            front_tire->alpha_prime(), rear_tire->alpha_prime(),
                                            tire_mode, road,
                                            T_f_tread, T_r_tread, T_f_carcass, T_r_carcass,
                                            p2f ? p2f->sliding_fraction() : 0.0,
                                            p2r ? p2r->sliding_fraction() : 0.0,
                                            p2f ? p2f->last_diagnostics().contact_area_cm2 : 0.0,
                                            p2r ? p2r->last_diagnostics().contact_area_cm2 : 0.0,
                                            cfd ? cfd->scales : nullptr,
                                            cfd ? cfd->n_updates : 0,
                                            cfd ? cfd->last_latency_us : 0.0) << '\n';
        }

        // Physics step
        s = rk4_step(t, s, DT, p, cur_inp, *front_tire, *rear_tire, road);
        t += DT;
        step++;

        // Thermal update (1 kHz)
        {
            auto ks2 = compute_kinematics(p, s, 0.0, 0.0);
            const double vx_b = std::abs(s[VX_CG]);
            const double Pf = std::abs(ks2.Fz_front * ks2.alpha_f_kin * vx_b)
                            + std::abs(ks2.kappa_f  * ks2.Fz_front * vx_b);
            const double Pr = std::abs(ks2.Fz_rear  * ks2.alpha_r_kin * vx_b)
                            + std::abs(ks2.kappa_r  * ks2.Fz_rear  * vx_b);
            T_f_tread    += DT * (-BETA_COOL * (T_f_tread    - T_AMB) + ALPHA_HEAT * Pf);
            T_r_tread    += DT * (-BETA_COOL * (T_r_tread    - T_AMB) + ALPHA_HEAT * Pr);
            T_f_carcass  += DT * (-BETA_COOL * (T_f_carcass  - T_AMB) + ALPHA_HEAT * 0.4 * Pf);
            T_r_carcass  += DT * (-BETA_COOL * (T_r_carcass  - T_AMB) + ALPHA_HEAT * 0.4 * Pr);
        }

        // Real-time pacing (1 kHz)
        auto target = wall_start + microseconds(static_cast<long long>(t * 1e6));
        auto now    = high_resolution_clock::now();
        if (target > now)
            std::this_thread::sleep_until(target);
    }

    running = false;
    stdin_thread.join();
}

// ── Smoke test mode ───────────────────────────────────────────────────────────

static int run_smoke_test(BikeParams& p,
                           std::unique_ptr<TireModelBase>& front_tire,
                           std::unique_ptr<TireModelBase>& rear_tire,
                           const RoadParams& road,
                           TireMode tire_mode, double v0, double t_sim)
{
    StateVec s = make_initial_state(v0);
    RiderInput inp;
    constexpr double DT = 0.001;
    int n_steps  = static_cast<int>(t_sim / DT);
    int log_every = static_cast<int>(0.5 / DT);
    Telemetry tel;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "\n 13-DOF Motorcycle Engine — Phase 1 Smoke Test\n";
    std::cout << " v0=" << v0 << " m/s  dt=" << DT*1000 << " ms  T=" << t_sim << " s\n";
    std::cout << " Road: " << road.name << "  mu=" << road.mu
              << "  banking=" << road.banking << " rad"
              << "  gradient=" << road.gradient << " rad\n\n";
    std::cout << std::setw(7)  << "t[s]"
              << std::setw(8)  << "vx[m/s]"
              << std::setw(9)  << "roll[°]"
              << std::setw(9)  << "yaw[°/s]"
              << std::setw(9)  << "Fz_f[N]"
              << std::setw(9)  << "Fz_r[N]"
              << std::setw(8)  << "Fy_f[N]"
              << std::setw(9)  << "fork[mm]"
              << std::setw(9)  << "trail[m]"
              << "\n" << std::string(88, '-') << "\n";

    auto wall_t0 = high_resolution_clock::now();

    for (int k = 0; k < n_steps; ++k) {
        const double t = k * DT;
        if (k % log_every == 0) {
            eom(t, s, p, inp, *front_tire, *rear_tire, road, &tel);
            std::cout << std::setw(7)  << t
                      << std::setw(8)  << tel.vx
                      << std::setw(9)  << tel.roll  * (180.0/M_PI)
                      << std::setw(9)  << tel.yaw_rate * (180.0/M_PI)
                      << std::setw(9)  << tel.Fz_f
                      << std::setw(9)  << tel.Fz_r
                      << std::setw(8)  << tel.Fy_f
                      << std::setw(9)  << tel.fork_travel * 1000.0
                      << std::setw(9)  << tel.effective_trail
                      << "\n";
        }
        s = rk4_step(t, s, DT, p, inp, *front_tire, *rear_tire, road);
    }

    auto wall_ms = duration<double, std::milli>(high_resolution_clock::now() - wall_t0).count();
    std::cout << std::string(88, '-') << "\n";
    std::cout << "\n Simulation: " << n_steps << " steps in "
              << std::setprecision(1) << wall_ms << " ms"
              << "  (" << std::setprecision(0) << n_steps / (wall_ms * 1e-3) << " steps/s";
    std::cout << (wall_ms < t_sim * 1000.0 ? "  FASTER THAN REAL-TIME  [" : "  SLOWER  [")
              << std::setprecision(1) << t_sim * 1000.0 / wall_ms << "x])\n";

    // Sanity checks (always run)
    eom(t_sim, s, p, inp, *front_tire, *rear_tire, road, &tel);
    bool ok = true;
    auto chk = [&](const char* name, double v, double lo, double hi) {
        if (v < lo || v > hi) {
            std::cout << " FAIL: " << name << "=" << v
                      << " outside [" << lo << "," << hi << "]\n";
            ok = false;
        }
    };
    chk("vx",    tel.vx,             v0*0.5, v0*1.5);
    chk("roll",  tel.roll*180/M_PI,  -15.0,  15.0);
    chk("Fz_f",  tel.Fz_f,           100.0, 3000.0);
    chk("Fz_r",  tel.Fz_r,           100.0, 3000.0);
    chk("trail", tel.effective_trail,  0.01,   0.20);
    std::cout << (ok ? "\n ALL SANITY CHECKS PASS\n" : "\n SANITY CHECKS FAILED\n");

    // Tire-model specific diagnostics
    if (tire_mode == TireMode::PINN) {
        auto* ot = dynamic_cast<OnnxTire*>(front_tire.get());
        std::cout << " PINN: " << (ot ? ot->mean_latency_us() : 0.0) << " us/call  "
                  << "fallbacks=" << (ot ? ot->n_fallbacks() : 0) << "\n";
    } else if (tire_mode == TireMode::Phase2) {
        auto* p2 = dynamic_cast<Phase2Tire*>(front_tire.get());
        if (p2) {
            auto d = p2->last_diagnostics();
            std::cout << " Phase2: contact_area=" << d.contact_area_cm2 << " cm²"
                      << "  peak_pressure=" << d.peak_pressure_MPa << " MPa"
                      << "  sliding_fraction=" << d.sliding_fraction << "\n";
        }
    }
    return ok ? 0 : 1;
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string onnx_path;
    std::string bike_yaml;
    std::string road_yaml;
    double v0       = 10.0;
    double t_sim    = 5.0;
    double phi0_deg = 0.0;
    double duration = -1.0;    // -1 = not set
    bool   stream   = false;
    bool   headless = false;

    bool   use_full_mf    = false;
    bool   use_phase2     = false;
    bool   use_coeff_fit  = false;
    std::string cf_model_path;

    for (int i = 1; i < argc; ++i) {
        if      (!std::strcmp(argv[i], "--pinn")       && i+1 < argc) onnx_path     = argv[++i];
        else if (!std::strcmp(argv[i], "--v0")         && i+1 < argc) v0            = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--t")          && i+1 < argc) t_sim         = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--phi0")       && i+1 < argc) phi0_deg      = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--duration")   && i+1 < argc) duration      = std::stod(argv[++i]);
        else if (!std::strcmp(argv[i], "--bike")       && i+1 < argc) bike_yaml     = argv[++i];
        else if (!std::strcmp(argv[i], "--road")       && i+1 < argc) road_yaml     = argv[++i];
        else if (!std::strcmp(argv[i], "--stream"))                    stream        = true;
        else if (!std::strcmp(argv[i], "--headless"))                  headless      = true;
        else if (!std::strcmp(argv[i], "--full-mf"))                   use_full_mf   = true;
        else if (!std::strcmp(argv[i], "--phase2"))                    use_phase2    = true;
        else if (!std::strcmp(argv[i], "--coeff-fit"))                 use_coeff_fit = true;
        else if (!std::strcmp(argv[i], "--cf-model") && i+1 < argc)   cf_model_path = argv[++i];
    }

    // ── Load bike params ──────────────────────────────────────────────────────
    BikeParams p;
    if (!bike_yaml.empty()) {
        try {
            p = BikeParams::from_yaml(bike_yaml);
            std::cerr << "[moto_engine] Bike: " << bike_yaml << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[moto_engine] ERROR loading bike YAML: " << e.what() << "\n";
            return 1;
        }
    }

    // ── Load road params ──────────────────────────────────────────────────────
    RoadParams road;
    if (!road_yaml.empty()) {
        try {
            road = RoadParams::from_yaml(road_yaml);
            std::cerr << "[moto_engine] Road: " << road.name
                      << "  mu=" << road.mu
                      << "  banking=" << road.banking
                      << "  gradient=" << road.gradient << "\n";
        } catch (const std::exception& e) {
            std::cerr << "[moto_engine] ERROR loading road YAML: " << e.what() << "\n";
            return 1;
        }
    }

    // ── Build tire models ─────────────────────────────────────────────────────
    PacejkaCoeffs medium_coeffs;
    std::unique_ptr<TireModelBase> front_tire, rear_tire;
    TireMode tire_mode = TireMode::Pacejka;

    if (!onnx_path.empty()) {
        tire_mode  = TireMode::PINN;
        front_tire = std::make_unique<OnnxTire>(onnx_path, medium_coeffs);
        rear_tire  = std::make_unique<OnnxTire>(onnx_path, medium_coeffs);
        if (!stream && !headless) std::cerr << "[moto_engine] PINN tire: " << onnx_path << "\n";
    } else if (use_phase2) {
        tire_mode  = TireMode::Phase2;
        front_tire = std::make_unique<Phase2Tire>();
        rear_tire  = std::make_unique<Phase2Tire>();
        if (!stream && !headless) std::cerr << "[moto_engine] Phase2 tires (FullMF + LuGre + Hertz contact)\n";
    } else if (use_full_mf) {
        tire_mode  = TireMode::FullMF;
        front_tire = std::make_unique<FullMFTire>();
        rear_tire  = std::make_unique<FullMFTire>();
        if (!stream && !headless) std::cerr << "[moto_engine] FullMF tires (large camber + gyro Mz + turn-slip)\n";
    } else {
        front_tire = std::make_unique<PacejkaTire>(medium_coeffs);
        rear_tire  = std::make_unique<PacejkaTire>(medium_coeffs);
        if (!stream && !headless) std::cerr << "[moto_engine] Pacejka analytical tires\n";
    }

    // ── Phase 3.1 coefficient fitter ─────────────────────────────────────────
    // Build with ONNX model if available, otherwise returns nominal always
    if (cf_model_path.empty())
        cf_model_path = std::string(getenv("HOME") ? getenv("HOME") : ".")
                        + "/Desktop/Project vi grade/Startup/moto_sim/dataset/coeff_fitter.onnx";

    auto front_fitter = std::make_unique<PINNCoeffFitter>(
        use_coeff_fit ? cf_model_path : "", FullMFCoeffs{});
    auto rear_fitter  = std::make_unique<PINNCoeffFitter>(
        use_coeff_fit ? cf_model_path : "", FullMFCoeffs{});

    if (use_coeff_fit) {
        g_cf_front = front_fitter.get();
        g_cf_rear  = rear_fitter.get();
        if (front_fitter->ort_loaded())
            std::cerr << "[moto_engine] Coeff-fitter: " << cf_model_path << "\n";
        else
            std::cerr << "[moto_engine] Coeff-fitter: ORT not available — returning nominal\n";
    }

    // ── Mode dispatch ─────────────────────────────────────────────────────────
    if (headless) {
        double dur = (duration > 0.0) ? duration : t_sim;
        return run_headless(p, front_tire, rear_tire, road, tire_mode, v0, phi0_deg, dur);
    }

    if (stream) {
        std::cerr << "[moto_engine] Streaming at 1 kHz, telemetry at 50 Hz\n";
        run_streaming(p, front_tire, rear_tire, road, tire_mode, v0);
        return 0;
    }

    return run_smoke_test(p, front_tire, rear_tire, road, tire_mode, v0, t_sim);
}
