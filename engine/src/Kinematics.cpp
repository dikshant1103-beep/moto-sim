#include "moto/Kinematics.h"
#include <cmath>
#include <algorithm>

namespace moto {

// ── Individual geometry functions ────────────────────────────────────────────

double kinematic_trail(const BikeParams& p, double phi, double delta) {
    // Cossalter §2.3: mechanical trail varies with lean and steer.
    // Full formula: t = (R_f * cos(ε) - f_offset * sin(ε)) / cos(δ) * cos(φ)
    // which simplifies (for small fork offset) to:
    //   t ≈ t_static * cos(φ) / cos(delta)
    // Valid to ±1% for |φ| < 50°, |δ| < 20°.
    return p.trail * std::cos(phi) / std::max(std::cos(delta), 0.1);
}

double kinematic_wheelbase(const BikeParams& p, double delta, double phi) {
    // Wheelbase shortens when steering: front contact patch moves rearward.
    // L_eff = L_0 - R_f * sin(|δ|) * sin(ε)   [Foale Ch.2 geometric]
    double delta_effect = p.R_front * std::sin(std::abs(delta)) * std::sin(p.rake);
    return p.wheelbase - delta_effect;
}

double kinematic_rake(const BikeParams& p, double phi) {
    // Effective rake angle increases with lean: ε_eff ≈ ε + φ * sin(ε)
    return p.rake + phi * std::sin(p.rake);
}

double contact_patch_y_offset(double R, double phi) {
    return R * std::sin(phi);
}

// ── Full kinematic state ──────────────────────────────────────────────────────

KinematicState compute_kinematics(const BikeParams& p,
                                   const StateVec&   s,
                                   double ax_smooth,
                                   double ay_smooth) {
    KinematicState ks;

    const double phi   = s[PHI];
    const double theta = s[THETA];
    const double psi   = s[PSI];
    const double del   = s[DELTA];
    const double sf    = s[SF];
    const double sr    = s[SR];
    const double of    = s[OMEGA_F];
    const double or_   = s[OMEGA_R];
    const double phi_r = s[PHI_RIDER];

    const double vx_w  = s[VX_CG];
    const double vy_w  = s[VY_CG];
    const double psi_d = s[PSI_DOT];
    const double phi_d = s[PHI_DOT];

    // ── Dynamic geometry ─────────────────────────────────────────────────────
    ks.effective_trail     = kinematic_trail(p, phi, del);
    ks.effective_wheelbase = kinematic_wheelbase(p, del, phi);
    ks.effective_rake      = kinematic_rake(p, phi);

    // ── Camber angles ─────────────────────────────────────────────────────────
    // Tire camber = chassis roll (road is flat)
    ks.gamma_front = phi;
    ks.gamma_rear  = phi;

    // ── Contact patch positions (world frame, approximate) ────────────────────
    const double cp = std::cos(psi), sp = std::sin(psi);
    const double L_eff = ks.effective_wheelbase;

    ks.p_rear  = { s[X_CG] - p.cg_to_rear * cp,
                   s[Y_CG] - p.cg_to_rear * sp,
                   0.0 };
    ks.p_front = { s[X_CG] + (L_eff - p.cg_to_rear) * cp,
                   s[Y_CG] + (L_eff - p.cg_to_rear) * sp,
                   0.0 };

    // ── Slip angles (kinematic) ───────────────────────────────────────────────
    // Body-frame forward and lateral velocities
    const double vx_b =  vx_w * cp + vy_w * sp;
    const double vy_b = -vx_w * sp + vy_w * cp;

    const double v_safe = std::max(std::abs(vx_b), 0.5);  // avoid div/0 at standstill

    // Rear: alpha_r = -(vy_b - psi_dot * a) / vx_b
    double vy_r  = vy_b - psi_d * p.cg_to_rear;
    ks.alpha_r_kin = -std::atan2(vy_r, v_safe);

    // Front: alpha_f accounts for steering angle
    double vy_f  = vy_b + psi_d * (L_eff - p.cg_to_rear);
    // Standard SAE: alpha_f = delta - atan2(vy_f, vx)  (positive alpha → Fy positive = left)
    ks.alpha_f_kin = del - std::atan2(vy_f, v_safe);

    // ── Slip ratios ───────────────────────────────────────────────────────────
    const double v_f_contact = of * p.R_front;
    const double v_r_contact = or_ * p.R_rear;
    ks.kappa_f = (v_f_contact - vx_b) / (v_safe + 1e-3);
    ks.kappa_r = (v_r_contact - vx_b) / (v_safe + 1e-3);
    ks.v_f = vx_b;
    ks.v_r = vx_b;

    // ── Normal load split ─────────────────────────────────────────────────────
    // Longitudinal: from moment about rear/front axle
    // Fz_f = (M*g*(L_eff-a) - M*ax*h) / L_eff
    // Fz_r = (M*g*a + M*ax*h) / L_eff
    const double Mg  = p.total_mass * p.G;
    const double a   = p.cg_to_rear;
    const double b   = L_eff - a;  // CG to front axle
    const double h   = p.cg_height;

    // Static + longitudinal weight transfer
    double Fz_f = (Mg * b - p.total_mass * ax_smooth * h) / L_eff;
    double Fz_r = (Mg * a + p.total_mass * ax_smooth * h) / L_eff;

    // Lateral weight transfer (roll + lateral accel)
    const double dFz_lat = p.total_mass * ay_smooth * h * std::sin(phi) / L_eff;
    Fz_f -= dFz_lat;
    Fz_r += dFz_lat;

    // Suspension stiffness contribution
    const double F_susp_f = p.k_front * (sf + p.susp_front_rest)
                           + p.c_front * s[SF_DOT];
    const double F_susp_r = p.k_rear  * (sr + p.susp_rear_rest)
                           + p.c_rear  * s[SR_DOT];

    // Contact force = spring force + unsprung weight (quarter-car identity).
    // The old blend (0.7*quasi + 0.3*spring) was missing the unsprung weight
    // term, so sf_ddot ≠ 0 at t=0 and the fork extended until Fz_f → 10 N.
    ks.Fz_front = std::max(10.0, F_susp_f + p.front_unsprung * p.G);
    ks.Fz_rear  = std::max(10.0, F_susp_r + p.rear_unsprung  * p.G);

    return ks;
}

} // namespace moto
