#pragma once
#include <array>
#include <cmath>

namespace moto {

// ─────────────────────────────────────────────────────────────────────────────
// 13-DOF motorcycle state vector  (26 elements: positions + velocities)
//
// DOF layout:
//   Chassis translation  : x_cg, y_cg, z_cg          [m]
//   Chassis rotation     : phi, theta, psi             [rad]   roll, pitch, yaw
//   Steering             : delta                       [rad]
//   Front suspension     : s_f                         [m]     + = compressed
//   Rear suspension      : s_r                         [m]
//   Front wheel spin     : omega_f                     [rad/s]
//   Rear wheel spin      : omega_r                     [rad/s]
//   Rider lateral lean   : phi_rider                   [rad]   relative to chassis
//   Rider long. position : x_rider                     [m]     relative to chassis CG
//
// State array layout: [q (13), q_dot (13)]  — total 26 elements
// ─────────────────────────────────────────────────────────────────────────────

constexpr int N_DOF   = 13;
constexpr int N_STATE = 26;

// Position indices
enum Pos : int {
    X_CG=0, Y_CG, Z_CG,           //  0–2   chassis CG world position
    PHI, THETA, PSI,               //  3–5   roll, pitch, yaw
    DELTA,                         //  6     steer angle
    SF, SR,                        //  7–8   suspension travel
    OMEGA_F, OMEGA_R,              //  9–10  wheel spin rates
    PHI_RIDER, X_RIDER,            // 11–12  rider DOF
};

// Velocity indices = Pos + N_DOF
enum Vel : int {
    VX_CG   = N_DOF + X_CG,
    VY_CG   = N_DOF + Y_CG,
    VZ_CG   = N_DOF + Z_CG,
    PHI_DOT = N_DOF + PHI,
    THETA_DOT = N_DOF + THETA,
    PSI_DOT = N_DOF + PSI,
    DELTA_DOT = N_DOF + DELTA,
    SF_DOT  = N_DOF + SF,
    SR_DOT  = N_DOF + SR,
    ALPHA_F = N_DOF + OMEGA_F,   // angular acceleration (stored as d(omega)/dt)
    ALPHA_R = N_DOF + OMEGA_R,
    PHI_RIDER_DOT = N_DOF + PHI_RIDER,
    X_RIDER_DOT   = N_DOF + X_RIDER,
};

using StateVec = std::array<double, N_STATE>;

// ── Convenience accessors ─────────────────────────────────────────────────────
inline double  phi  (const StateVec& s) { return s[PHI];   }
inline double  psi  (const StateVec& s) { return s[PSI];   }
inline double  theta(const StateVec& s) { return s[THETA]; }
inline double  delta(const StateVec& s) { return s[DELTA]; }
inline double  phi_dot  (const StateVec& s) { return s[PHI_DOT];   }
inline double  psi_dot  (const StateVec& s) { return s[PSI_DOT];   }
inline double  delta_dot(const StateVec& s) { return s[DELTA_DOT]; }

// Forward speed (body frame x projection onto world)
inline double vx_world(const StateVec& s) { return s[VX_CG]; }

// Initialise a straight-line steady-state at given speed v0 [m/s]
inline StateVec make_initial_state(double v0 = 5.0) {
    StateVec s{};
    s[Z_CG]  = 0.58;     // CG height
    s[VX_CG] = v0;
    s[OMEGA_F] = v0 / 0.299;   // front wheel spin
    s[OMEGA_R] = v0 / 0.318;   // rear wheel spin
    // Static suspension compression under weight
    s[SF] = 0.0;   // equilibrium (rest value absorbs static sag)
    s[SR] = 0.0;
    return s;
}

} // namespace moto
