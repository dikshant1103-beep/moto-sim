#pragma once
// BikeParams — all fixed parameters for one motorcycle configuration.
// Units: SI throughout (m, kg, N, Nm, rad, rad/s).
// Matches cbr600rr.yaml values used in the Python simulator.

#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>

namespace moto {

struct BikeParams {
    // ── Geometry ──────────────────────────────────────────────────────────────
    double wheelbase   = 1.370;   // [m]  front-to-rear axle distance
    double rake        = 0.4189;  // [rad] steering head angle from vertical (24°)
    double trail       = 0.097;   // [m]  mechanical trail at contact patch
    double fork_offset = 0.030;   // [m]  fork offset (perpendicular to steer axis)
    double R_front     = 0.299;   // [m]  front wheel radius
    double R_rear      = 0.318;   // [m]  rear wheel radius
    double cg_height   = 0.580;   // [m]  CG above ground
    double cg_to_rear  = 0.740;   // [m]  CG to rear axle (wheelbase - cg_x_from_rear)

    // ── Masses ────────────────────────────────────────────────────────────────
    double total_mass  = 189.0;   // [kg]  total (bike + rider)
    double rider_mass  =  75.0;   // [kg]  rider body mass
    double front_unsprung = 12.0; // [kg]  front wheel + fork lower
    double rear_unsprung  = 12.0; // [kg]  rear wheel + swingarm

    // Component masses (used by YAML loader and analytical formulas)
    double front_wheel_mass = 4.5;   // [kg]
    double rear_wheel_mass  = 7.0;   // [kg]
    double main_frame_mass  = 165.0; // [kg]
    double swingarm_mass    = 5.0;   // [kg]
    double front_fork_mass  = 7.5;   // [kg]

    // ── Inertia tensors (about CG, kg·m²) ────────────────────────────────────
    double Ixx = 30.0;   // roll
    double Iyy = 80.0;   // pitch  (includes wheels rotating in sim)
    double Izz = 50.0;   // yaw
    double Ixz = -2.5;   // coupling

    // Wheel spin inertia (about spin axis)
    double I_front_spin = 0.38;   // [kg·m²]
    double I_rear_spin  = 0.58;   // [kg·m²]

    // Steer inertia (about steer axis)
    double I_steer = 0.20;        // [kg·m²]

    // Rider lean inertia
    double I_rider_roll = 8.0;    // [kg·m²]
    double rider_lean_arm = 0.55; // [m]  effective CG shift per rad of lean

    // ── Suspension ────────────────────────────────────────────────────────────
    double k_front    = 18500.0;  // [N/m]  front spring
    double c_front    =   900.0;  // [N·s/m] front damper
    double k_rear     = 42000.0;  // [N/m]
    double c_rear     =  1800.0;  // [N·s/m]
    // Spring precompression at equilibrium:
    //   rest = (Fz_static - m_unsprung * G) / k_spring
    //   Fz_f_static ≈ 852 N  → rest_f = (852-118)/18500 = 0.040 m
    //   Fz_r_static ≈ 1002 N → rest_r = (1002-118)/42000 = 0.021 m
    double susp_front_rest = 0.040; // [m]  spring precompression at static equilibrium
    double susp_rear_rest  = 0.021;

    // ── Powertrain ────────────────────────────────────────────────────────────
    double engine_torque_max = 80.0;   // [N·m]  rear wheel drive torque max
    double brake_torque_max  = 250.0;  // [N·m]  combined

    // ── Aerodynamics ──────────────────────────────────────────────────────────
    double CdA  = 0.35;  // [m²] drag area
    double CLA  = 0.10;  // [m²] lift area (positive = downforce)
    double rho_air = 1.225; // [kg/m³]

    // ── Steering damping ──────────────────────────────────────────────────────
    double C_steer = 0.50;  // [N·m·s/rad]

    // ── Gravity ───────────────────────────────────────────────────────────────
    static constexpr double G = 9.81;

    // ── Derived (computed from primary params) ────────────────────────────────
    double cg_to_front() const { return wheelbase - cg_to_rear; }
    double sprung_mass()  const { return total_mass - front_unsprung - rear_unsprung; }

    // ── YAML loader (pure C++17 std, no external dependencies) ───────────────
    static BikeParams from_yaml(const std::string& path) {
        BikeParams p;
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("BikeParams: cannot open file: " + path);

        std::string line;
        while (std::getline(f, line)) {
            // Strip inline comments
            auto hash = line.find('#');
            if (hash != std::string::npos)
                line = line.substr(0, hash);

            // Trim leading whitespace
            auto ltrim = line.find_first_not_of(" \t\r\n");
            if (ltrim == std::string::npos) continue;
            line = line.substr(ltrim);

            auto colon = line.find(':');
            if (colon == std::string::npos) continue;

            std::string key = line.substr(0, colon);
            std::string val = line.substr(colon + 1);

            // Trim both strings
            auto trim = [](std::string& s) {
                auto a = s.find_first_not_of(" \t\r\n\"'");
                auto b = s.find_last_not_of(" \t\r\n\"'");
                s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
            };
            trim(key);
            trim(val);
            if (key.empty() || val.empty()) continue;

            try {
                double dv = std::stod(val);
                if      (key == "wheelbase")           p.wheelbase        = dv;
                else if (key == "rake")                p.rake             = dv;
                else if (key == "trail")               p.trail            = dv;
                else if (key == "front_wheel_radius")  p.R_front          = dv;
                else if (key == "rear_wheel_radius")   p.R_rear           = dv;
                else if (key == "total_mass")          p.total_mass       = dv;
                else if (key == "front_wheel_mass")    p.front_wheel_mass = dv;
                else if (key == "rear_wheel_mass")     p.rear_wheel_mass  = dv;
                else if (key == "main_frame_mass")     p.main_frame_mass  = dv;
                else if (key == "swingarm_mass")       p.swingarm_mass    = dv;
                else if (key == "front_fork_mass")     p.front_fork_mass  = dv;
                // cg_x in YAML = distance from rear axle
                else if (key == "cg_x")                p.cg_to_rear       = dv;
                else if (key == "cg_height")           p.cg_height        = dv;
                else if (key == "frame_Ixx")           p.Ixx              = dv;
                else if (key == "frame_Iyy")           p.Iyy              = dv;
                else if (key == "frame_Izz")           p.Izz              = dv;
                else if (key == "frame_Ixz")           p.Ixz              = dv;
                else if (key == "front_wheel_Iyy")     p.I_front_spin     = dv;
                else if (key == "rear_wheel_Iyy")      p.I_rear_spin      = dv;
                else if (key == "rider_mass")          p.rider_mass       = dv;
                else if (key == "rider_lean_arm")      p.rider_lean_arm   = dv;
                else if (key == "front_spring_k")      p.k_front          = dv;
                else if (key == "front_damper_c")      p.c_front          = dv;
                else if (key == "rear_spring_k")       p.k_rear           = dv;
                else if (key == "rear_damper_c")       p.c_rear           = dv;
                else if (key == "engine_torque_max")   p.engine_torque_max= dv;
                else if (key == "brake_torque_max")    p.brake_torque_max = dv;
                else if (key == "steering_friction")   p.C_steer          = dv;
                else if (key == "Cd_A")                p.CdA              = dv;
                else if (key == "Cl_A")                p.CLA              = dv;
            } catch (...) {
                // non-numeric value (e.g. "name:") — skip silently
            }
        }

        // Compute suspension rest lengths from static equilibrium
        const double L     = p.wheelbase;
        const double Fz_f  = p.total_mass * G * (L - p.cg_to_rear) / L;
        const double Fz_r  = p.total_mass * G * p.cg_to_rear / L;
        p.susp_front_rest  = (Fz_f - p.front_unsprung * G) / p.k_front;
        p.susp_rear_rest   = (Fz_r - p.rear_unsprung  * G) / p.k_rear;

        return p;
    }
};

} // namespace moto
