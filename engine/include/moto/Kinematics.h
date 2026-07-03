#pragma once
#include "Vec3.h"
#include "BikeParams.h"
#include "State.h"

namespace moto {

// ─────────────────────────────────────────────────────────────────────────────
// Exact motorcycle kinematics — Cossalter (2006) §2.2–2.4, Foale (2002) Ch. 2
// All functions take the current state and return geometric quantities.
// ─────────────────────────────────────────────────────────────────────────────

struct KinematicState {
    // Contact patch positions in world frame
    Vec3 p_front;   // front contact patch
    Vec3 p_rear;    // rear contact patch

    // Dynamic geometry
    double effective_wheelbase;  // [m]  changes with steering + lean
    double effective_trail;      // [m]  Cossalter §2.3, function of δ and φ
    double effective_rake;       // [rad] dynamic rake angle

    // Camber angles at contact patches
    double gamma_front;  // [rad]  positive = lean right
    double gamma_rear;

    // Slip angles (kinematic, before relaxation)
    double alpha_f_kin;  // [rad]
    double alpha_r_kin;

    // Slip ratios
    double kappa_f;
    double kappa_r;

    // Normal load split (quasi-static + dynamic)
    double Fz_front;  // [N]
    double Fz_rear;

    // Wheel spin speeds [rad/s] and contact patch speeds [m/s]
    double v_f;   // front contact patch forward speed
    double v_r;   // rear contact patch forward speed
};

// Compute the full kinematic state from the dynamics state and rider inputs.
KinematicState compute_kinematics(const BikeParams& p,
                                   const StateVec&   s,
                                   double ax_smooth,    // [m/s²] filtered long. accel
                                   double ay_smooth);   // [m/s²] filtered lat. accel

// ── Individual geometry functions (documented by source) ─────────────────────

// Dynamic mechanical trail (Cossalter §2.3, exact nonlinear):
//   t(δ,φ) = [R_f * cos(ε + Δε(φ)) - f_0 * sin(ε + Δε(φ))] / cos(ε + Δε(φ)) - tan(ε+Δε(φ)) * R_f * sin(φ_eff)
// Simplified to first-order in δ and φ for numerical stability:
//   t ≈ t_static * cos(φ) / cos(δ)
// This is accurate to < 1% for |φ| < 45°, |δ| < 15°.
double kinematic_trail(const BikeParams& p, double phi, double delta);

// Dynamic wheelbase (changes with steer angle):
//   L(δ) = L_0 - R_f * sin(|δ|) * sin(rake)    [approximate]
double kinematic_wheelbase(const BikeParams& p, double delta, double phi);

// Effective rake (caster angle) as function of lean:
//   ε_eff(φ) ≈ ε + φ * sin(ε)    [Foale Ch.2]
double kinematic_rake(const BikeParams& p, double phi);

// Contact patch lateral position offset from wheel plane centre:
//   y_offset = R * sin(phi)   (toroidal tire approximation)
double contact_patch_y_offset(double R, double phi);

} // namespace moto
