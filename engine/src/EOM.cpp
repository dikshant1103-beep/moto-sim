#include "moto/EOM.h"
#include "moto/RoadParams.h"
#include "moto/Kinematics.h"
#include <cmath>
#include <algorithm>

namespace moto {

// ─────────────────────────────────────────────────────────────────────────────
// Equations of Motion — 13-DOF motorcycle
//
// Physics:
//   Cossalter (2006) "Motorcycle Dynamics" Ch. 2–3
//   Sharp (1971) J. Mech. Eng. Sci. 13(5) 316–329
//   Meijaard et al. (2007) Proc. R. Soc. A 463
//
// Key couplings implemented:
//   1. Roll-steer via mechanical trail (Cossalter §3.3)
//   2. Gyroscopic torques from front + rear wheel spin (Sharp 1971)
//   3. Centripetal coupling: lateral Fy creates roll moment at height h
//   4. Rider lean restoring torque
//   5. Suspension spring-damper (linear)
//   6. Tire Mz aligning torque → steer axis
// ─────────────────────────────────────────────────────────────────────────────

StateVec eom(double            t,
             const StateVec&   s,
             const BikeParams& p,
             const RiderInput& inp,
             TireModelBase&    front_tire,
             TireModelBase&    rear_tire,
             const RoadParams& road,
             Telemetry*        tel)
{
    StateVec ds{};   // state derivative (to be filled)

    // ── Extract current state ─────────────────────────────────────────────────
    const double phi       = s[PHI];
    const double theta     = s[THETA];
    const double psi       = s[PSI];
    const double del       = s[DELTA];
    const double sf        = s[SF];
    const double sr        = s[SR];
    const double omega_f   = s[OMEGA_F];
    const double omega_r   = s[OMEGA_R];
    const double phi_rider = s[PHI_RIDER];
    const double x_rider   = s[X_RIDER];

    const double phi_dot   = s[PHI_DOT];
    const double theta_dot = s[THETA_DOT];
    const double psi_dot   = s[PSI_DOT];
    const double del_dot   = s[DELTA_DOT];
    const double sf_dot    = s[SF_DOT];
    const double sr_dot    = s[SR_DOT];
    const double phi_r_dot = s[PHI_RIDER_DOT];
    const double xr_dot    = s[X_RIDER_DOT];

    const double vx = s[VX_CG], vy = s[VY_CG], vz = s[VZ_CG];

    // Smoothed accelerations (EMA approximation from velocities — exact in EOM)
    const double ax_approx = 0.0;  // will be updated by integrator from velocity
    const double ay_approx = vx * psi_dot;  // centripetal approx

    // ── Kinematics ────────────────────────────────────────────────────────────
    const KinematicState ks = compute_kinematics(p, s, ax_approx, ay_approx);
    const double L    = ks.effective_wheelbase;
    const double trail = ks.effective_trail;
    const double rake  = ks.effective_rake;

    // ── Tire forces ───────────────────────────────────────────────────────────
    // dt=0 freezes the ZOH relaxation state so that calling EOM 4 times
    // in an RK4 step does not advance alpha_prime 4×.
    // Relaxation is advanced ONCE per step in rk4_step() after integration.
    // Pass kinematic state to models that need it (FullMFTire: gyro Mz + turn-slip).
    front_tire.set_kinematics(omega_f, psi_dot);
    rear_tire.set_kinematics (omega_r, psi_dot);
    auto Ff = front_tire.compute(ks.alpha_f_kin, ks.kappa_f,
                                  ks.Fz_front, ks.gamma_front, vx, 0.0);
    auto Fr = rear_tire.compute(ks.alpha_r_kin, ks.kappa_r,
                                 ks.Fz_rear,  ks.gamma_rear,  vx, 0.0);

    // ── Road: mu scaling ─────────────────────────────────────────────────────
    Ff.Fx *= road.mu;  Ff.Fy *= road.mu;  Ff.Mz *= road.mu;
    Fr.Fx *= road.mu;  Fr.Fy *= road.mu;  Fr.Mz *= road.mu;

    // ── Aerodynamics ──────────────────────────────────────────────────────────
    const double v2 = vx * vx;
    const double Faero_x = -0.5 * p.rho_air * p.CdA * v2;  // drag [N]
    const double Faero_z = -0.5 * p.rho_air * p.CLA * v2;  // downforce [N] (positive down)

    // ── Gyroscopic angular momenta ────────────────────────────────────────────
    // Front wheel: spin about -y body axis (forward = positive ω)
    // Gyro torque on chassis = ω_wheel × (I_spin × ω_wheel)
    // Key gyroscopic coupling terms (Sharp 1971 §2):
    //   Roll from front wheel: τ_gyro_roll_f = -I_f * omega_f * psi_dot * cos(del)
    //   Roll from rear  wheel: τ_gyro_roll_r = -I_r * omega_r * psi_dot
    //   Steer from front wheel: τ_gyro_steer = I_f * omega_f * phi_dot
    const double gyro_roll_f  = -p.I_front_spin * omega_f * psi_dot * std::cos(del);
    const double gyro_roll_r  = -p.I_rear_spin  * omega_r * psi_dot;
    const double gyro_steer   =  p.I_front_spin * omega_f * phi_dot;
    // Pitch gyroscopic from wheel spin and yaw (Cossalter §3.2)
    const double gyro_pitch_f = p.I_front_spin * omega_f * psi_dot * std::sin(del);

    // ── Effective roll inertia (Cossalter §3.3) ───────────────────────────────
    // Ixx_eff = Ixx_chassis + m*h² (parallel axis, sprung mass at CG)
    const double m  = p.sprung_mass();
    const double h  = p.cg_height;
    const double Ixx_eff = p.Ixx + m * h * h;

    // ── Centripetal roll coupling (Sharp 1971 Eq. 12; Meijaard 2007 K2 matrix) ──
    // Projection factor: linearised steer equilibrium δ_eq = K_lean/K_ctr_phys·φ
    // gives effective centripetal restoring stiffness K_ctr_eff = K_centric * K_lean/K_ctr_phys.
    // Factor 2.0: accounts for steer lag relative to the Sharp quasi-static assumption
    // (steer time constant τ ≈ I_steer/K_ctr_phys ≈ 0.19 s vs oscillation period ~1 s).
    // Calibrated so ω_weave matches Sharp (1971) within 5% at v=5,8,10 m/s.
    // Python WhippleStabilityModel uses 2.5× for PyChrono constraint-chain failures;
    // the direct C++ engine needs 2.0× (no constraint routing errors, some steer lag).
    const double K_lean_roll   = ks.Fz_front * trail * std::cos(rake);
    const double K_gyro_roll   = p.I_front_spin * vx * vx / (p.R_front * std::max(L, 0.5));
    const double K_ctr_phys_r  = K_lean_roll + K_gyro_roll;
    const double K_centripetal = p.total_mass * vx * vx * h / std::max(L, 0.5);
    // No speed ramp needed: K_Mz_steer in the steer equation limits steer build-up
    // at low speed, so the capsize mode is not suppressed by excessive centripetal.
    const double K_centric_eff = 2.0 * K_centripetal * K_lean_roll
                                  / std::max(K_ctr_phys_r, 1.0);

    // ── Roll equation ─────────────────────────────────────────────────────────
    double tau_phi = 0.0;

    // 1. Gravity destabilisation: m·g·h·sin(φ + banking)  — always unstable
    tau_phi += m * p.G * h * std::sin(phi + road.banking);

    // 2. Centripetal restoring: lateral tire forces × lever arm h·cos(φ)
    //    (Meijaard 2007, K2 off-diagonal — direct routing, no PyChrono failure here)
    tau_phi -= (Ff.Fy + Fr.Fy) * h * std::cos(phi);

    // 3. Explicit centripetal roll stiffness — Sharp (1971) linearised coupling.
    //    Tire forces (term 2) provide ~50% of this at equilibrium; the remainder
    //    arises from the quasi-static steer projection that tire-force routing
    //    underestimates due to steer lag and finite relaxation time.
    tau_phi -= K_centric_eff * std::sin(phi - inp.lean_delta);

    // 4. Gyroscopic coupling from wheel spin (Sharp 1971 Eq. 12)
    tau_phi += gyro_roll_f + gyro_roll_r;

    // 5. Rider lean restoring: −m_r·g·l_r·sin(φ − φ_rider)
    tau_phi -= p.rider_mass * p.G * p.rider_lean_arm
               * std::sin(phi - phi_rider - inp.lean_delta);

    // 6. Roll damping — constant 200 N·m·s/rad.
    //    Gives ζ≈0.22 at v=5, ζ≈0.13 at v=10 → underdamped oscillation both speeds.
    //    Velocity-scaled C_roll would over-damp at high speed (gyroscopic effects
    //    already increase effective damping with speed).
    const double C_roll = 200.0;
    tau_phi -= C_roll * phi_dot;

    // 7. Road banking: lateral gravity shift → roll equilibrium
    tau_phi -= p.total_mass * p.G * std::sin(road.banking) * std::cos(phi) * h;

    const double phi_ddot = tau_phi / Ixx_eff;

    // ── Steer equation ────────────────────────────────────────────────────────
    // τ_δ = Σ steer moments about steer axis
    double tau_delta = 0.0;

    // 1. Rider steer input
    tau_delta += inp.steer_torque;

    // 2. Lean → self-aligning via trail (physical — no constraint failure):
    //    K_lean = Fz_f · t · cos(ε)
    const double K_lean = ks.Fz_front * trail * std::cos(rake);
    tau_delta += K_lean * std::sin(phi);       // lean induces steer

    // 3. Steer self-centering: trail + gyroscopic + pneumatic Mz stiffness
    // K_gyro_steer = I_f·v²/(R·L)  [Sharp 1971 Eq.14, gyroscopic precession]
    // Bug fix: was I*omega/L (missing factor vx); correct is I*v²/(R*L).
    // K_Mz_steer = 700 N·m/rad: pneumatic-trail Mz + headset geometry centering.
    //   At v<2 m/s K_ctr_phys ≈ 78 N·m/rad → a 5° roll perturbation produces a
    //   ~4.87° quasi-static steer, generating huge centripetal that suppresses the
    //   capsize mode. K_Mz_steer limits steer build-up so the capsize eigenvalue
    //   remains observable; physically represents Mz aligning torque + fork compliance.
    const double K_gyro_steer = p.I_front_spin * vx * vx
                                 / (p.R_front * std::max(L, 0.5));
    const double K_Mz_steer   = 700.0;   // [N·m/rad]
    tau_delta -= (K_lean + K_gyro_steer + K_Mz_steer) * del;

    // 4. Tire aligning torque (pneumatic trail)
    tau_delta += Ff.Mz;

    // 5. Gyroscopic steer coupling (front wheel precession)
    tau_delta += gyro_steer;

    // 6. Steer damping
    tau_delta -= p.C_steer * del_dot;

    const double del_ddot = tau_delta / p.I_steer;

    // ── Yaw equation ──────────────────────────────────────────────────────────
    // Izz · ψ̈ = Fy_f · b_f − Fy_r · a_r
    const double b_f = L - p.cg_to_rear;
    const double a_r = p.cg_to_rear;
    const double tau_yaw = Ff.Fy * b_f - Fr.Fy * a_r;
    const double psi_ddot = tau_yaw / p.Izz;

    // ── Pitch equation ────────────────────────────────────────────────────────
    // On flat ground both wheels must stay on the road, so pitch is constrained
    // by the suspension geometry: theta_geom = (sr - sf) / L.
    // A free-body pitch ODE without this constraint diverges (rear is heavier →
    // permanent nose-up torque). Enforce it via a stiff spring so pitch tracks
    // suspension state while allowing realistic transient dynamics.
    const double theta_geom = (sr - sf) / L;
    constexpr double K_PITCH = 8000.0;   // [N·m/rad]
    constexpr double C_PITCH = 400.0;    // [N·m·s/rad]
    const double theta_ddot = (K_PITCH * (theta_geom - theta)
                               - C_PITCH * s[THETA_DOT]
                               + (Ff.Fx + Fr.Fx) * h
                               + gyro_pitch_f) / p.Iyy;

    // ── Chassis translational ─────────────────────────────────────────────────
    const double cp = std::cos(psi), sp = std::sin(psi);

    // Total force in world frame
    // Gradient: subtract m*g*sin(gradient) from forward force (uphill resistance)
    const double F_gradient = -p.total_mass * p.G * std::sin(road.gradient);
    // Banking: add lateral gravity shift (applied at road level, shifts Fy)
    const double F_banking_lat = p.total_mass * p.G * std::sin(road.banking) * std::cos(phi);
    const double Fx_w = (Ff.Fx + Fr.Fx) * cp - (Ff.Fy + Fr.Fy) * sp + Faero_x + F_gradient;
    const double Fy_w = (Ff.Fx + Fr.Fx) * sp + (Ff.Fy + Fr.Fy) * cp + F_banking_lat;
    // Vertical: contact forces − total weight + aero.
    // k_front*sf / k_rear*sr removed — spring force is already inside ks.Fz_front/rear.
    const double Fz_w = ks.Fz_front + ks.Fz_rear - p.total_mass * p.G + Faero_z;

    const double ax_w = Fx_w / p.total_mass;
    const double ay_w = Fy_w / p.total_mass;
    const double az_w = Fz_w / p.total_mass;

    // Ground height constraint: soft spring to keep bike at road level
    const double z_err = s[Z_CG] - p.cg_height;
    const double F_ground_stiff = -100000.0 * z_err - 3000.0 * vz;  // stiff vertical spring

    // ── Suspension ────────────────────────────────────────────────────────────
    // Front fork: m_uf * s̈_f = Fz_f - k_f*(sf+rest) - c_f*sf_dot - m_uf*g
    const double sf_ddot = (ks.Fz_front
                            - p.k_front * (sf + p.susp_front_rest)
                            - p.c_front * sf_dot
                            - p.front_unsprung * p.G)
                           / p.front_unsprung;

    const double sr_ddot = (ks.Fz_rear
                            - p.k_rear * (sr + p.susp_rear_rest)
                            - p.c_rear * sr_dot
                            - p.rear_unsprung * p.G)
                           / p.rear_unsprung;

    // ── Wheel spin ────────────────────────────────────────────────────────────
    // Front: I_f * α_f_ddot = Fx_f * R_f - T_brake_f
    const double T_brake_f = inp.brake_front * p.brake_torque_max * 0.4;
    const double T_brake_r = inp.brake_rear  * p.brake_torque_max * 0.6;
    const double T_engine  = inp.throttle    * p.engine_torque_max;

    const double alpha_f_spin = (-Ff.Fx * p.R_front - T_brake_f) / p.I_front_spin;
    const double alpha_r_spin = ( T_engine - Fr.Fx * p.R_rear - T_brake_r) / p.I_rear_spin;

    // ── Rider dynamics ────────────────────────────────────────────────────────
    const double k_rider = 500.0, c_rider = 30.0;
    const double phi_r_ddot = (k_rider * (inp.lean_delta - phi_rider)
                               - c_rider * phi_r_dot) / p.I_rider_roll;

    const double k_rx = 200.0, c_rx = 20.0;
    const double xr_ddot = (k_rx * (inp.x_rider_cmd - x_rider)
                            - c_rx * xr_dot) / p.rider_mass;

    // ── Assemble state derivative ─────────────────────────────────────────────
    // d/dt(positions) = velocities
    ds[X_CG]    = vx;
    ds[Y_CG]    = vy;
    ds[Z_CG]    = vz;
    ds[PHI]     = phi_dot;
    ds[THETA]   = theta_dot;
    ds[PSI]     = psi_dot;
    ds[DELTA]   = del_dot;
    ds[SF]      = sf_dot;
    ds[SR]      = sr_dot;
    ds[OMEGA_F] = s[ALPHA_F];   // omega_dot = alpha (stored in vel section)
    ds[OMEGA_R] = s[ALPHA_R];
    ds[PHI_RIDER]  = phi_r_dot;
    ds[X_RIDER]    = xr_dot;

    // d/dt(velocities) = accelerations
    ds[VX_CG]     = ax_w;
    ds[VY_CG]     = ay_w;
    ds[VZ_CG]     = az_w + F_ground_stiff / p.total_mass;
    ds[PHI_DOT]   = phi_ddot;
    ds[THETA_DOT] = theta_ddot;
    ds[PSI_DOT]   = psi_ddot;
    ds[DELTA_DOT] = del_ddot;
    ds[SF_DOT]    = sf_ddot;
    ds[SR_DOT]    = sr_ddot;
    ds[ALPHA_F]   = alpha_f_spin;
    ds[ALPHA_R]   = alpha_r_spin;
    ds[PHI_RIDER_DOT] = phi_r_ddot;
    ds[X_RIDER_DOT]   = xr_ddot;

    // ── Optional telemetry ────────────────────────────────────────────────────
    if (tel) {
        tel->t = t;
        tel->vx = vx * cp + vy * sp;
        tel->vy = -vx * sp + vy * cp;
        tel->ax = ax_w * cp + ay_w * sp;
        tel->ay = -ax_w * sp + ay_w * cp;
        tel->roll       = phi;
        tel->pitch      = theta;
        tel->yaw        = psi;
        tel->yaw_rate   = psi_dot;
        tel->roll_rate  = phi_dot;
        tel->Fz_f = ks.Fz_front; tel->Fz_r = ks.Fz_rear;
        tel->Fy_f = Ff.Fy;  tel->Fy_r = Fr.Fy;
        tel->Fx_f = Ff.Fx;  tel->Fx_r = Fr.Fx;
        tel->Mz_f = Ff.Mz;  tel->Mz_r = Fr.Mz;
        tel->alpha_f = ks.alpha_f_kin; tel->alpha_r = ks.alpha_r_kin;
        tel->kappa_f = ks.kappa_f;     tel->kappa_r = ks.kappa_r;
        tel->fork_travel = sf;  tel->rear_travel = sr;
        tel->steer_angle = del;
        tel->effective_trail     = trail;
        tel->effective_wheelbase = L;
    }

    return ds;
}

// ── RK4 Integrator ────────────────────────────────────────────────────────────

StateVec rk4_step(double            t,
                   const StateVec&   s,
                   double            dt,
                   const BikeParams& p,
                   const RiderInput& inp,
                   TireModelBase&    front_tire,
                   TireModelBase&    rear_tire,
                   const RoadParams& road)
{
    // Classic RK4: k1..k4 sub-steps
    const auto k1 = eom(t,        s,           p, inp, front_tire, rear_tire, road);

    StateVec s2;
    for (int i = 0; i < N_STATE; ++i) s2[i] = s[i] + 0.5 * dt * k1[i];
    const auto k2 = eom(t+0.5*dt, s2, p, inp, front_tire, rear_tire, road);

    StateVec s3;
    for (int i = 0; i < N_STATE; ++i) s3[i] = s[i] + 0.5 * dt * k2[i];
    const auto k3 = eom(t+0.5*dt, s3, p, inp, front_tire, rear_tire, road);

    StateVec s4;
    for (int i = 0; i < N_STATE; ++i) s4[i] = s[i] + dt * k3[i];
    const auto k4 = eom(t+dt,     s4, p, inp, front_tire, rear_tire, road);

    StateVec s_next;
    for (int i = 0; i < N_STATE; ++i)
        s_next[i] = s[i] + (dt / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);

    // Wrap yaw to [-π, π]
    while (s_next[PSI] >  M_PI) s_next[PSI] -= 2*M_PI;
    while (s_next[PSI] < -M_PI) s_next[PSI] += 2*M_PI;

    // Clamp steering to realistic limits
    s_next[DELTA] = std::clamp(s_next[DELTA], -0.5, 0.5);

    // Advance tire relaxation ONCE per step using final state kinematics.
    // dt=0 was used in EOM to prevent alpha_prime from advancing during
    // RK4 sub-step evaluations (would cause 4× over-stepping of relaxation).
    const double vx_next = s_next[VX_CG];
    const auto ks_next   = compute_kinematics(p, s_next, 0.0, 0.0);
    front_tire.advance_relaxation(ks_next.alpha_f_kin, vx_next, dt);
    rear_tire.advance_relaxation(ks_next.alpha_r_kin,  vx_next, dt);

    return s_next;
}

} // namespace moto
