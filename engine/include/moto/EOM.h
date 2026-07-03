#pragma once
#include "State.h"
#include "BikeParams.h"
#include "RoadParams.h"
#include "TireModel.h"
#include <functional>

namespace moto {

// ── Rider control inputs ──────────────────────────────────────────────────────
struct RiderInput {
    double steer_torque = 0.0;  // [N·m]  direct steer torque (positive = right)
    double throttle     = 0.0;  // [0,1]
    double brake_front  = 0.0;  // [0,1]
    double brake_rear   = 0.0;  // [0,1]
    double lean_delta   = 0.0;  // [rad]  rider upper-body lean command
    double x_rider_cmd  = 0.0;  // [m]   rider longitudinal shift command
};

// ── Full telemetry snapshot ───────────────────────────────────────────────────
struct Telemetry {
    double t{};
    // Chassis
    double vx{}, vy{}, ax{}, ay{};
    double roll{}, pitch{}, yaw{}, yaw_rate{}, roll_rate{};
    // Tires
    double Fz_f{}, Fz_r{};
    double Fy_f{}, Fy_r{};
    double Fx_f{}, Fx_r{};
    double Mz_f{}, Mz_r{};
    double alpha_f{}, alpha_r{};
    double kappa_f{}, kappa_r{};
    // Suspension
    double fork_travel{}, rear_travel{};
    // Steering
    double steer_angle{};
    // Geometry
    double effective_trail{};
    double effective_wheelbase{};
};

// ── Equations of Motion ───────────────────────────────────────────────────────
// Returns the state derivative: f(t, s) = ds/dt
//
// Physics sources:
//   Roll coupling:    Cossalter (2006) §3.3, Eq. 3.15
//   Steer coupling:   Sharp (1971) Eq. 12–14; Meijaard (2007) K2 matrix
//   Gyroscopic:       Cossalter §3.2, gyroscopic matrix G
//   Suspension:       Cossalter §5.1 (linear spring-damper)
//   Tire forces:      Pacejka (2006) Ch. 11 motorcycle MF
//   Road conditions:  mu scaling, gradient, banking effects
StateVec eom(double           t,
             const StateVec&  s,
             const BikeParams& p,
             const RiderInput& inp,
             TireModelBase&   front_tire,
             TireModelBase&   rear_tire,
             const RoadParams& road,
             Telemetry*       telem_out = nullptr);  // optional telemetry capture

// ── RK4 Integrator ────────────────────────────────────────────────────────────
// Classic 4th-order Runge-Kutta — deterministic, O(h^4) error.
StateVec rk4_step(double           t,
                  const StateVec&  s,
                  double           dt,
                  const BikeParams& p,
                  const RiderInput& inp,
                  TireModelBase&   front_tire,
                  TireModelBase&   rear_tire,
                  const RoadParams& road);

} // namespace moto
