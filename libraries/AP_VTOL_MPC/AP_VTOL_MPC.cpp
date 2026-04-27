/*
 * AP_VTOL_MPC.cpp — Unified MPC controller for tilt-rotor VTOL UAV
 *
 * Reference: Chen et al., "A Unified MPC Strategy for a Tilt-rotor VTOL UAV
 *            Towards Seamless Mode Transitioning," AIAA SciTech 2024.
 *
 * See AP_VTOL_MPC.h for architecture overview and coordinate conventions.
 */

#include "AP_VTOL_MPC.h"

#if HAL_QUADPLANE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <cstring>
#include <cmath>

extern const AP_HAL::HAL &hal;

// ─────────────────────────────────────────────────────────────────────────────
// ArduPilot parameter table
// ─────────────────────────────────────────────────────────────────────────────
const AP_Param::GroupInfo AP_VTOL_MPC::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable VTOL MPC controller
    // @Description: Enables the unified MPC controller from Chen et al. 2024.
    //   0=disabled (standard PID), 1=enabled
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_VTOL_MPC, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HORIZON
    // @DisplayName: MPC prediction horizon
    // @Description: Number of time steps in the prediction horizon. Paper uses 25.
    // @Range: 5 20
    // @User: Advanced
    AP_GROUPINFO("HORIZON", 2, AP_VTOL_MPC, _horizon, VTOL_MPC_N),

    // @Param: DT_MS
    // @DisplayName: MPC solver time step (ms)
    // @Description: Prediction integration step.  20ms → 50 Hz MPC update.
    // @Range: 10 50
    // @User: Advanced
    AP_GROUPINFO("DT_MS", 3, AP_VTOL_MPC, _dt_ms, 20),

    // @Param: QVX
    // @DisplayName: Velocity tracking weight vN
    // @Description: Q_ref diagonal entry for North velocity (paper: 20)
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("QVX", 4, AP_VTOL_MPC, _Qvx, 20.0f),

    // @Param: QVY
    // @DisplayName: Velocity tracking weight vE
    // @Description: Q_ref diagonal entry for East velocity (paper: 10)
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("QVY", 5, AP_VTOL_MPC, _Qvy, 10.0f),

    // @Param: QVZ
    // @DisplayName: Velocity tracking weight vD
    // @Description: Q_ref diagonal entry for Down velocity (paper: 50)
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("QVZ", 6, AP_VTOL_MPC, _Qvz, 50.0f),

    // @Param: QROLL
    // @DisplayName: Attitude cost weight roll
    // @Description: Q_ψ diagonal entry for roll (paper: 10)
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("QROLL", 7, AP_VTOL_MPC, _Qroll, 10.0f),

    // @Param: QPITCH
    // @DisplayName: Attitude cost weight pitch
    // @Description: Q_ψ diagonal entry for pitch (paper: 20)
    // @Range: 1 200
    // @User: Advanced
    AP_GROUPINFO("QPITCH", 8, AP_VTOL_MPC, _Qpitch, 20.0f),

    // @Param: IQITR
    // @DisplayName: Gradient projection iterations
    // @Description: Maximum projected gradient descent iterations per solve step.
    // @Range: 5 50
    // @User: Advanced
    AP_GROUPINFO("IQITR", 9, AP_VTOL_MPC, _max_iter, 20),

    AP_GROUPEND
};

AP_VTOL_MPC *AP_VTOL_MPC::_singleton = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Default vehicle parameters  (Table 1 of the paper + tilthvec geometry)
// ─────────────────────────────────────────────────────────────────────────────
TilthvecParams default_tilthvec_params()
{
    TilthvecParams p{};
    p.mass_kg   = 7.427f;
    p.Ixx       = 10.685f;
    p.Iyy       = 5.7465f;
    p.Izz       = 4.6678f;
    p.arm_x     = 0.25f;
    p.arm_y     = 0.30f;
    p.c_F       = 8.004e-5f;  // thrust coefficient [N·s²/rad²], calibrated to T_max=27.36N
    p.c_K       = 1.6e-6f;   // torque coefficient [N·m·s²/rad²], c_K/c_F ratio ≈ 0.02
    p.T_max_N   = 27.36f;    // per rotor at 100% throttle
    p.S_wing     = 0.44f;
    p.S_aileron  = 0.036f;
    p.S_elevator = 0.1364f;
    p.S_rudder   = 0.004f;
    p.wingspan_b = 2.0f;
    p.chord_c    = 0.22f;
    p.CD0        = 0.35f;
    p.CD_alpha   = 0.11f;
    p.CZ0        = 0.03f;
    p.CZ_alpha   = 0.2f;
    p.CL_delta_a = 0.1173f;
    p.CM_delta_e = 0.556f;
    p.CN_delta_r = 0.0881f;
    p.chi_min_rad = -7.0f * DEG_TO_RAD;
    p.chi_max_rad =  95.0f * DEG_TO_RAD;
    p.chi_dot_max = M_PI / 4.0f;  // π/4 rad/s
    p.d_L = -1;  // left side CW net effect
    p.d_R =  1;  // right side CCW net effect
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────
// Default MPC weights (Table 2 of the paper)
// ─────────────────────────────────────────────────────────────────────────────
MPCWeights MPCWeights::defaults()
{
    MPCWeights w{};
    w.Qv[0] = 20.0f; w.Qv[1] = 10.0f; w.Qv[2] = 50.0f;  // Q_ref
    w.Qf[0] = 20.0f; w.Qf[1] = 10.0f; w.Qf[2] = 50.0f;  // Q_f
    w.Qpsi[0] = 10.0f; w.Qpsi[1] = 20.0f; w.Qpsi[2] = 10.0f; // Q_ψ
    w.Qomega[0] = 3.0f; w.Qomega[1] = 3.0f; w.Qomega[2] = 3.0f; // Q_ψ̇
    // R_u: [χ̇_L, χ̇_R, T, φ_d, θ_d, ψ_d]
    w.Ru[0] = 1.0f;  w.Ru[1] = 1.0f;  // tilt rates
    w.Ru[2] = 0.025f;                   // thrust
    w.Ru[3] = 10.0f; w.Ru[4] = 20.0f; w.Ru[5] = 10.0f; // attitude setpoints
    // Soft constraint parameters  (equation 20, constants from paper)
    w.soft_a = -0.33f; w.soft_b = 13.32f;
    w.soft_c =  1.80f; w.soft_d = -2.303f;
    return w;
}

// ─────────────────────────────────────────────────────────────────────────────
// Default MPC constraints  (equation 16 of the paper)
// ─────────────────────────────────────────────────────────────────────────────
MPCConstraints MPCConstraints::defaults()
{
    MPCConstraints c{};
    c.v_max[0] = 30.0f; c.v_max[1] = 30.0f; c.v_max[2] = 10.0f;
    c.chi_min  = -7.0f * DEG_TO_RAD;
    c.chi_max  =  95.0f * DEG_TO_RAD;
    c.euler_max[0] = M_PI / 4.0f;    // ±45°
    c.euler_max[1] = M_PI / 4.0f;    // ±45°
    c.euler_max[2] = 1e6f;           // unconstrained yaw
    c.omega_max[0] = M_PI;           // ±180°/s
    c.omega_max[1] = M_PI;
    c.omega_max[2] = M_PI;
    c.chi_dot_max  = M_PI / 4.0f;   // ±45°/s
    c.T_min  = 0.0f;
    c.T_max  = 4.0f * 27.36f;       // 4 rotors at max thrust
    c.euler_d_max[0] = M_PI / 2.0f; // ±90°
    c.euler_d_max[1] = M_PI / 3.0f; // ±60°
    c.euler_d_max[2] = M_PI / 3.0f; // ±60°
    return c;
}

// ─────────────────────────────────────────────────────────────────────────────
// MPCStateVec helpers
// ─────────────────────────────────────────────────────────────────────────────
void MPCStateVec::to_flat(float x[VTOL_MPC_NX]) const
{
    x[0]  = v[0];      x[1]  = v[1];       x[2]  = v[2];
    x[3]  = chi[0];    x[4]  = chi[1];
    x[5]  = euler[0];  x[6]  = euler[1];   x[7]  = euler[2];
    x[8]  = omega[0];  x[9]  = omega[1];   x[10] = omega[2];
    x[11] = omega_p[0];x[12] = omega_p[1]; x[13] = omega_p[2];
}

void MPCStateVec::from_flat(const float x[VTOL_MPC_NX])
{
    v[0] = x[0]; v[1] = x[1]; v[2] = x[2];
    chi[0] = x[3]; chi[1] = x[4];
    euler[0] = x[5]; euler[1] = x[6]; euler[2] = x[7];
    omega[0] = x[8]; omega[1] = x[9]; omega[2] = x[10];
    omega_p[0] = x[11]; omega_p[1] = x[12]; omega_p[2] = x[13];
}

void MPCInputVec::to_flat(float u[VTOL_MPC_NU]) const
{
    u[0] = chi_dot[0]; u[1] = chi_dot[1];
    u[2] = T;
    u[3] = euler_d[0]; u[4] = euler_d[1]; u[5] = euler_d[2];
}

void MPCInputVec::from_flat(const float u[VTOL_MPC_NU])
{
    chi_dot[0] = u[0]; chi_dot[1] = u[1];
    T          = u[2];
    euler_d[0] = u[3]; euler_d[1] = u[4]; euler_d[2] = u[5];
}

// ─────────────────────────────────────────────────────────────────────────────
// VTOLDynamics implementation
// ─────────────────────────────────────────────────────────────────────────────
VTOLDynamics::VTOLDynamics(const TilthvecParams &p) : _p(p) {}

// Build 3×3 DCM R_IB (body → inertial) from ZYX Euler angles φ,θ,ψ
void VTOLDynamics::R_IB(const float euler[3], float R[3][3]) const
{
    const float cphi   = cosf(euler[0]), sphi   = sinf(euler[0]);
    const float ctheta = cosf(euler[1]), stheta = sinf(euler[1]);
    const float cpsi   = cosf(euler[2]), spsi   = sinf(euler[2]);

    R[0][0] = ctheta * cpsi;
    R[0][1] = sphi * stheta * cpsi - cphi * spsi;
    R[0][2] = cphi * stheta * cpsi + sphi * spsi;

    R[1][0] = ctheta * spsi;
    R[1][1] = sphi * stheta * spsi + cphi * cpsi;
    R[1][2] = cphi * stheta * spsi - sphi * cpsi;

    R[2][0] = -stheta;
    R[2][1] = sphi * ctheta;
    R[2][2] = cphi * ctheta;
}

// Aerodynamic effectiveness factor  e(Va) (Figure 4 of paper)
// Sigmoid fitted to paper's Fig.4: e → 0 at Va=0, e → 1 at Va≥15 m/s
float VTOLDynamics::aero_effectiveness(float airspeed_ms) const
{
    // Use a smooth sigmoid: e = 1 / (1 + exp(-k*(Va - Va_half)))
    // k=0.5, Va_half=7 → matches paper's figure reasonably well
    const float k = 0.5f, Va_half = 7.0f;
    const float e = 1.0f / (1.0f + expf(-k * (airspeed_ms - Va_half)));
    return constrain_float(e, 0.0f, 1.0f);
}

// Inner-loop PD torque computation (equations 21–22 of the paper)
// τ = K_p_rate*(K_p_att*(Ψ_sp - Ψ) - Ψ̇) + K_d_rate*(Ψ̇⁻ - Ψ̇)
void VTOLDynamics::inner_loop_torque(const float euler[3],
                                      const float omega[3],
                                      const float omega_p[3],
                                      const float euler_d[3],
                                      float tau_B[3]) const
{
    for (int i = 0; i < 3; i++) {
        float att_err = euler_d[i] - euler[i];
        // Wrap yaw error to ±π
        if (i == 2) {
            att_err = wrap_PI(att_err);
        }
        float rate_sp = KP_ATT * att_err;
        tau_B[i] = KP_RATE * (rate_sp - omega[i])
                 + KD_RATE * (omega_p[i] - omega[i]);
    }
    // Scale torque by inertia (I * α)
    tau_B[0] *= _p.Ixx;
    tau_B[1] *= _p.Iyy;
    tau_B[2] *= _p.Izz;
}

// Continuous-time dynamics ẋ = f(x, u)   (equation 23 of the paper)
void VTOLDynamics::f(const float x[VTOL_MPC_NX],
                     const float u[VTOL_MPC_NU],
                     float xdot[VTOL_MPC_NX],
                     float airspeed_ms) const
{
    // Unpack state
    const float *chi    = &x[3];   // χ_L, χ_R
    const float *euler  = &x[5];   // φ, θ, ψ
    const float *omega  = &x[8];   // p, q, r
    const float *omega_p = &x[11]; // p⁻, q⁻, r⁻

    // Unpack input
    const float *chi_dot = &u[0];  // χ̇_L, χ̇_R
    const float T        = u[2];   // total thrust [N]
    const float *euler_d = &u[3];  // φ_d, θ_d, ψ_d

    // ── Rotation matrix body → inertial ────────────────────────────────────
    float R[3][3];
    R_IB(euler, R);

    // ── Rotor thrust force in body frame  (equations 2–3 of paper) ─────────
    // tilthvec: left pair (motors 1,4) share χ_L; right pair (2,3) share χ_R
    // Aggregate thrust T is split evenly across 4 motors: T_each = T/4
    const float T_left  = T * 0.5f; // total for left  pair
    const float T_right = T * 0.5f; // total for right pair

    // Body-frame thrust from left rotors (pointing direction [sin(χ_L), 0, -cos(χ_L)])
    float F_B_thrust[3];
    F_B_thrust[0] = T_left  * sinf(chi[0]) + T_right * sinf(chi[1]);
    F_B_thrust[1] = 0.0f;
    F_B_thrust[2] = -(T_left * cosf(chi[0]) + T_right * cosf(chi[1]));

    // ── Transform thrust to inertial frame ──────────────────────────────────
    float F_I_thrust[3];
    for (int i = 0; i < 3; i++) {
        F_I_thrust[i] = R[i][0] * F_B_thrust[0]
                      + R[i][1] * F_B_thrust[1]
                      + R[i][2] * F_B_thrust[2];
    }

    // ── Aerodynamic forces  (equations 5–11) ───────────────────────────────
    const float e    = aero_effectiveness(airspeed_ms);
    const float Va2  = airspeed_ms * airspeed_ms;
    const float q_bar = 0.5f * 1.225f * Va2; // dynamic pressure [Pa]

    // Angle of attack: approximate using body pitch angle (small angle regime)
    const float alpha = euler[1]; // body pitch ≈ AoA for level flight

    const float CD = _p.CD0 + _p.CD_alpha * alpha * alpha;
    const float CL = _p.CZ0 + _p.CZ_alpha * alpha;

    // Drag force (opposing forward motion, body x)
    // Lift force (opposing gravity, body z)
    float F_B_aero[3];
    F_B_aero[0] = -e * q_bar * _p.S_wing * CD; // drag (body forward = -drag)
    F_B_aero[1] = 0.0f;
    F_B_aero[2] = -e * q_bar * _p.S_wing * CL; // lift (body z is down, lift is -z)

    // Transform to inertial
    float F_I_aero[3];
    for (int i = 0; i < 3; i++) {
        F_I_aero[i] = R[i][0] * F_B_aero[0]
                    + R[i][1] * F_B_aero[1]
                    + R[i][2] * F_B_aero[2];
    }

    // ── Gravity in inertial frame (NED: positive down) ──────────────────────
    // Use ArduPilot's standard GRAVITY_MSS constant for consistency
    float F_I_grav[3] = {0.0f, 0.0f, _p.mass_kg * GRAVITY_MSS};

    // ── Linear acceleration in inertial frame ───────────────────────────────
    // v̇ = (1/m)*(F_thrust + F_aero) + g*e_d   (equation 23 term 1)
    // Note: gravity already included separately
    xdot[0] = (F_I_thrust[0] + F_I_aero[0]) / _p.mass_kg + F_I_grav[0] / _p.mass_kg;
    xdot[1] = (F_I_thrust[1] + F_I_aero[1]) / _p.mass_kg + F_I_grav[1] / _p.mass_kg;
    xdot[2] = (F_I_thrust[2] + F_I_aero[2]) / _p.mass_kg + F_I_grav[2] / _p.mass_kg;
    // ── Tilt rate (control input directly integrated) ──────────────────────
    xdot[3] = chi_dot[0]; // χ̇_L
    xdot[4] = chi_dot[1]; // χ̇_R

    // ── Euler angle kinematics  Ψ̇ = T(Ψ) * ω ─────────────────────────────
    const float sphi = sinf(euler[0]), cphi = cosf(euler[0]);
    const float ctheta = cosf(euler[1]);
    const float ttheta = tanf(euler[1]);

    xdot[5] = omega[0] + sphi * ttheta * omega[1] + cphi * ttheta * omega[2];
    xdot[6] = cphi * omega[1] - sphi * omega[2];
    xdot[7] = (ctheta > 1e-3f) ? (sphi / ctheta * omega[1] + cphi / ctheta * omega[2]) : 0.0f;

    // ── Angular acceleration from inner-loop torque ─────────────────────────
    float tau_B[3];
    inner_loop_torque(euler, omega, omega_p, euler_d, tau_B);

    // Gyroscopic effects (ω × I·ω) — small for this vehicle, included for accuracy
    const float Ixx = _p.Ixx, Iyy = _p.Iyy, Izz = _p.Izz;
    xdot[8]  = tau_B[0] / Ixx + (Iyy - Izz) / Ixx * omega[1] * omega[2];
    xdot[9]  = tau_B[1] / Iyy + (Izz - Ixx) / Iyy * omega[0] * omega[2];
    xdot[10] = tau_B[2] / Izz + (Ixx - Iyy) / Izz * omega[0] * omega[1];

    // ── Previous angular rate: copy current (for inner-loop delay model) ───
    xdot[11] = omega[0] - omega_p[0]; // drives omega_p → omega with 1-step delay
    xdot[12] = omega[1] - omega_p[1];
    xdot[13] = omega[2] - omega_p[2];
}

// Euler-forward integration
void VTOLDynamics::integrate(const float x[VTOL_MPC_NX],
                              const float u[VTOL_MPC_NU],
                              float dt,
                              float x_next[VTOL_MPC_NX],
                              float airspeed_ms) const
{
    float xdot[VTOL_MPC_NX];
    f(x, u, xdot, airspeed_ms);
    for (int i = 0; i < VTOL_MPC_NX; i++) {
        x_next[i] = x[i] + dt * xdot[i];
    }
    // Wrap yaw
    x_next[7] = wrap_PI(x_next[7]);
    // Clamp tilt to physical limits
    const float chi_min = _p.chi_min_rad, chi_max = _p.chi_max_rad;
    x_next[3] = constrain_float(x_next[3], chi_min, chi_max);
    x_next[4] = constrain_float(x_next[4], chi_min, chi_max);
}

// Numerical Jacobians via central finite differences
void VTOLDynamics::jacobians(const float x[VTOL_MPC_NX],
                              const float u[VTOL_MPC_NU],
                              float dt,
                              MPCMat_nn A,
                              MPCMat_nm B,
                              float airspeed_ms) const
{
    const float eps_x = 1e-4f;
    const float eps_u = 1e-4f;

    float xp[VTOL_MPC_NX], xm[VTOL_MPC_NX];
    float x_tmp[VTOL_MPC_NX], u_tmp[VTOL_MPC_NU];

    // ∂x_next/∂x
    for (int j = 0; j < VTOL_MPC_NX; j++) {
        memcpy(x_tmp, x, sizeof(float) * VTOL_MPC_NX);
        x_tmp[j] += eps_x;
        integrate(x_tmp, u, dt, xp, airspeed_ms);

        memcpy(x_tmp, x, sizeof(float) * VTOL_MPC_NX);
        x_tmp[j] -= eps_x;
        integrate(x_tmp, u, dt, xm, airspeed_ms);

        for (int i = 0; i < VTOL_MPC_NX; i++) {
            A[i][j] = (xp[i] - xm[i]) / (2.0f * eps_x);
        }
    }

    // ∂x_next/∂u
    for (int j = 0; j < VTOL_MPC_NU; j++) {
        memcpy(u_tmp, u, sizeof(float) * VTOL_MPC_NU);
        u_tmp[j] += eps_u;
        integrate(x, u_tmp, dt, xp, airspeed_ms);

        memcpy(u_tmp, u, sizeof(float) * VTOL_MPC_NU);
        u_tmp[j] -= eps_u;
        integrate(x, u_tmp, dt, xm, airspeed_ms);

        for (int i = 0; i < VTOL_MPC_NX; i++) {
            B[i][j] = (xp[i] - xm[i]) / (2.0f * eps_u);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MPCSolver implementation
// ─────────────────────────────────────────────────────────────────────────────
MPCSolver::MPCSolver(const VTOLDynamics &dyn,
                     const MPCWeights   &w,
                     const MPCConstraints &con,
                     float dt_s,
                     int max_iter)
    : _dyn(dyn), _w(w), _con(con), _dt(dt_s), _max_iter(max_iter)
{
    memset(_U, 0, sizeof(_U));
    memset(_X, 0, sizeof(_X));
}

void MPCSolver::reset(const float x0[VTOL_MPC_NX])
{
    memset(_U, 0, sizeof(_U));
    // Initialise warm-start: zero tilt rate, thrust = hover thrust, level attitude
    const float hover_thrust = _dyn.params().mass_kg * GRAVITY_MSS;
    for (int k = 0; k < VTOL_MPC_N; k++) {
        _U[k][2] = hover_thrust; // T
    }
    // Propagate nominal trajectory
    memcpy(_X[0], x0, sizeof(float) * VTOL_MPC_NX);
    for (int k = 0; k < VTOL_MPC_N; k++) {
        _dyn.integrate(_X[k], _U[k], _dt, _X[k + 1]);
    }
}

// Stage cost L(x_k, u_k) — equation 17 of the paper
float MPCSolver::stage_cost(const float x[VTOL_MPC_NX],
                             const float u[VTOL_MPC_NU],
                             const float v_ref[3],
                             float airspeed_ms) const
{
    float cost = 0.0f;

    // ── J_ref: velocity tracking (equation 18) ──────────────────────────────
    for (int i = 0; i < 3; i++) {
        float ev = x[i] - v_ref[i];
        cost += _w.Qv[i] * ev * ev;
    }

    // ── J_x,u: attitude + rate + input cost (equation 19) ──────────────────
    for (int i = 0; i < 3; i++) {
        cost += _w.Qpsi[i]   * x[5 + i] * x[5 + i]; // attitude
        cost += _w.Qomega[i] * x[8 + i] * x[8 + i]; // angular rate
    }
    for (int i = 0; i < VTOL_MPC_NU; i++) {
        cost += _w.Ru[i] * u[i] * u[i]; // input cost
    }

    // ── J_soft: soft constraint (equation 20) ──────────────────────────────
    // Average tilt angle in degrees (body-forward speed used as speed proxy)
    const float chi_avg_deg = (x[3] + x[4]) * 0.5f * RAD_TO_DEG;
    const float v_fwd = x[0]; // North velocity as proxy for body-forward speed
    const float soft = expf(_w.soft_a * v_fwd * chi_avg_deg / 90.0f
                           + _w.soft_b * chi_avg_deg / 90.0f
                           + _w.soft_c * v_fwd / 30.0f
                           + _w.soft_d);
    cost += soft;

    return cost;
}

// Terminal cost (equation 18, Q_f term)
float MPCSolver::terminal_cost(const float x[VTOL_MPC_NX],
                                const float v_ref[3]) const
{
    float cost = 0.0f;
    for (int i = 0; i < 3; i++) {
        float ev = x[i] - v_ref[i];
        cost += _w.Qf[i] * ev * ev;
    }
    return cost;
}

// Total cost over the horizon
float MPCSolver::compute_cost(const float v_ref[3],
                               float yaw_ref,
                               float airspeed_ms) const
{
    float total = 0.0f;
    for (int k = 0; k < VTOL_MPC_N; k++) {
        total += stage_cost(_X[k], _U[k], v_ref, airspeed_ms);
    }
    total += terminal_cost(_X[VTOL_MPC_N], v_ref);
    return total;
}

// Stage cost gradient w.r.t. x and u
void MPCSolver::stage_cost_grad(const float x[VTOL_MPC_NX],
                                 const float u[VTOL_MPC_NU],
                                 const float v_ref[3],
                                 float airspeed_ms,
                                 float dLdx[VTOL_MPC_NX],
                                 float dLdu[VTOL_MPC_NU]) const
{
    memset(dLdx, 0, sizeof(float) * VTOL_MPC_NX);
    memset(dLdu, 0, sizeof(float) * VTOL_MPC_NU);

    // dL/dv: velocity tracking
    for (int i = 0; i < 3; i++) {
        dLdx[i] = 2.0f * _w.Qv[i] * (x[i] - v_ref[i]);
    }

    // dL/dΨ: attitude
    for (int i = 0; i < 3; i++) {
        dLdx[5 + i] = 2.0f * _w.Qpsi[i] * x[5 + i];
    }

    // dL/dω: angular rate
    for (int i = 0; i < 3; i++) {
        dLdx[8 + i] = 2.0f * _w.Qomega[i] * x[8 + i];
    }

    // dL/du: input cost
    for (int i = 0; i < VTOL_MPC_NU; i++) {
        dLdu[i] = 2.0f * _w.Ru[i] * u[i];
    }

    // Soft constraint gradient (w.r.t. χ_L and χ_R, approximate)
    const float chi_avg_deg = (x[3] + x[4]) * 0.5f * RAD_TO_DEG;
    const float v_fwd = x[0];
    const float soft_val = expf(_w.soft_a * v_fwd * chi_avg_deg / 90.0f
                               + _w.soft_b * chi_avg_deg / 90.0f
                               + _w.soft_c * v_fwd / 30.0f
                               + _w.soft_d);
    // d(soft)/d(chi_avg) = soft * (soft_a * v_fwd / 90 + soft_b / 90) * RAD_TO_DEG * 0.5
    const float ds_dchi = soft_val * (_w.soft_a * v_fwd / 90.0f
                                    + _w.soft_b / 90.0f)
                        * RAD_TO_DEG * 0.5f;
    dLdx[3] += ds_dchi;
    dLdx[4] += ds_dchi;

    // d(soft)/d(v_fwd)
    const float ds_dvf = soft_val * (_w.soft_a * chi_avg_deg / 90.0f
                                   + _w.soft_c / 30.0f);
    dLdx[0] += ds_dvf;
}

// Terminal cost gradient
void MPCSolver::terminal_cost_grad(const float x[VTOL_MPC_NX],
                                    const float v_ref[3],
                                    float dLdx[VTOL_MPC_NX]) const
{
    memset(dLdx, 0, sizeof(float) * VTOL_MPC_NX);
    for (int i = 0; i < 3; i++) {
        dLdx[i] = 2.0f * _w.Qf[i] * (x[i] - v_ref[i]);
    }
}

// Project u onto box constraints  (equation 16)
void MPCSolver::project_u(float u[VTOL_MPC_NU]) const
{
    const MPCConstraints &c = _con;
    u[0] = constrain_float(u[0], -c.chi_dot_max, c.chi_dot_max); // χ̇_L
    u[1] = constrain_float(u[1], -c.chi_dot_max, c.chi_dot_max); // χ̇_R
    u[2] = constrain_float(u[2], c.T_min, c.T_max);              // T
    u[3] = constrain_float(u[3], -c.euler_d_max[0], c.euler_d_max[0]); // φ_d
    u[4] = constrain_float(u[4], -c.euler_d_max[1], c.euler_d_max[1]); // θ_d
    u[5] = constrain_float(u[5], -c.euler_d_max[2], c.euler_d_max[2]); // ψ_d
}

// Backward pass: compute gradient dJ/dU via adjoint method
void MPCSolver::backward_pass(const float v_ref[3],
                               float yaw_ref,
                               float airspeed_ms,
                               float grad[VTOL_MPC_N][VTOL_MPC_NU]) const
{
    // Adjoint variable λ_k = dJ/dx_k
    float lambda[VTOL_MPC_NX];

    // Terminal adjoint: λ_N = dL_N/dx_N
    terminal_cost_grad(_X[VTOL_MPC_N], v_ref, lambda);

    // Backward recursion
    for (int k = VTOL_MPC_N - 1; k >= 0; k--) {
        float dLdx_k[VTOL_MPC_NX], dLdu_k[VTOL_MPC_NU];
        stage_cost_grad(_X[k], _U[k], v_ref, airspeed_ms, dLdx_k, dLdu_k);

        // Jacobians A = ∂x_{k+1}/∂x_k,  B = ∂x_{k+1}/∂u_k
        MPCMat_nn A_k;
        MPCMat_nm B_k;
        _dyn.jacobians(_X[k], _U[k], _dt, A_k, B_k, airspeed_ms);

        // grad[k] = dLdu_k + B_k^T * λ_{k+1}
        for (int j = 0; j < VTOL_MPC_NU; j++) {
            float g = dLdu_k[j];
            for (int i = 0; i < VTOL_MPC_NX; i++) {
                g += B_k[i][j] * lambda[i];
            }
            grad[k][j] = g;
        }

        // Update λ_k = dLdx_k + A_k^T * λ_{k+1}
        float lambda_new[VTOL_MPC_NX];
        for (int i = 0; i < VTOL_MPC_NX; i++) {
            lambda_new[i] = dLdx_k[i];
            for (int j = 0; j < VTOL_MPC_NX; j++) {
                lambda_new[i] += A_k[j][i] * lambda[j]; // A_k^T
            }
        }
        memcpy(lambda, lambda_new, sizeof(lambda));
    }
}

// Main solver: projected gradient descent with Armijo line search
bool MPCSolver::solve(const float x0[VTOL_MPC_NX],
                      const float v_ref[3],
                      float yaw_ref,
                      float airspeed_ms,
                      float u_out[VTOL_MPC_NU])
{
    // ── Warm-start shift: move previous solution forward by one step ─────────
    for (int k = 0; k < VTOL_MPC_N - 1; k++) {
        memcpy(_U[k], _U[k + 1], sizeof(float) * VTOL_MPC_NU);
    }
    // Last input: hold previous
    // (already copied from shift)

    // ── Forward pass: propagate trajectory ──────────────────────────────────
    memcpy(_X[0], x0, sizeof(float) * VTOL_MPC_NX);
    for (int k = 0; k < VTOL_MPC_N; k++) {
        _dyn.integrate(_X[k], _U[k], _dt, _X[k + 1], airspeed_ms);
    }

    float cost_prev = compute_cost(v_ref, yaw_ref, airspeed_ms);

    // ── Projected gradient iterations ───────────────────────────────────────
    // Number of iterations is controlled by the Q_MPC_IQITR parameter
    float grad[VTOL_MPC_N][VTOL_MPC_NU];

    for (int iter = 0; iter < _max_iter; iter++) {
        // Backward pass: compute gradient
        backward_pass(v_ref, yaw_ref, airspeed_ms, grad);

        // Armijo line search on step size α
        float alpha = 0.1f;
        float U_try[VTOL_MPC_N][VTOL_MPC_NU];
        float X_try[VTOL_MPC_N + 1][VTOL_MPC_NX];

        for (int ls = 0; ls < 5; ls++) {
            // Gradient step + projection
            for (int k = 0; k < VTOL_MPC_N; k++) {
                for (int j = 0; j < VTOL_MPC_NU; j++) {
                    U_try[k][j] = _U[k][j] - alpha * grad[k][j];
                }
                project_u(U_try[k]);
            }

            // Forward pass with trial inputs
            memcpy(X_try[0], x0, sizeof(float) * VTOL_MPC_NX);
            for (int k = 0; k < VTOL_MPC_N; k++) {
                _dyn.integrate(X_try[k], U_try[k], _dt, X_try[k + 1], airspeed_ms);
            }

            // Compute trial cost
            float cost_try = 0.0f;
            for (int k = 0; k < VTOL_MPC_N; k++) {
                cost_try += stage_cost(X_try[k], U_try[k], v_ref, airspeed_ms);
            }
            cost_try += terminal_cost(X_try[VTOL_MPC_N], v_ref);

            if (cost_try < cost_prev) {
                memcpy(_U, U_try, sizeof(_U));
                memcpy(_X, X_try, sizeof(_X));
                cost_prev = cost_try;
                break;
            }
            alpha *= 0.5f;
        }

        // Convergence check: gradient norm
        float gnorm = 0.0f;
        for (int k = 0; k < VTOL_MPC_N; k++) {
            for (int j = 0; j < VTOL_MPC_NU; j++) {
                gnorm += grad[k][j] * grad[k][j];
            }
        }
        if (gnorm < 1e-4f) {
            break;
        }
    }

    // Output: first optimal input
    memcpy(u_out, _U[0], sizeof(float) * VTOL_MPC_NU);
    return true; // always return best found solution
}

// ─────────────────────────────────────────────────────────────────────────────
// ControlAllocator implementation  (Section III.C of the paper)
// ─────────────────────────────────────────────────────────────────────────────
ControlAllocator::ControlAllocator(const TilthvecParams &p) : _p(p)
{
    _chi_L = 0.0f;
    _chi_R = 0.0f;
    memset(_u_prev, 0, sizeof(_u_prev));
    // Initial trim: hover at 50% throttle (normalised rotor speed)
    _u_prev[0] = 0.5f;
    _u_prev[1] = 0.5f;
}

void ControlAllocator::set_tilt(float chi_L_rad, float chi_R_rad)
{
    _chi_L = chi_L_rad;
    _chi_R = chi_R_rad;
}

// Build 6×5 effectiveness matrix A at current tilt + airspeed
// Actuators: [ω_L_norm², ω_R_norm², δ_ail, δ_elev, δ_rud]
// Wrench:    [Fx, Fy, Fz, τx, τy, τz]  body frame
void ControlAllocator::build_effectiveness(float airspeed_ms,
                                            float A_eff[6][5]) const
{
    memset(A_eff, 0, sizeof(float) * 6 * 5);

    const TilthvecParams &p = _p;
    const float T_max = p.T_max_N; // per rotor

    // ── Motor force contributions ────────────────────────────────────────────
    // Left pair (motors 1,4): thrust direction [sin(χ_L), 0, -cos(χ_L)]
    // Right pair (motors 2,3): thrust direction [sin(χ_R), 0, -cos(χ_R)]
    // Each pair contributes T_max * norm² to the body force

    const float sL = sinf(_chi_L), cL = cosf(_chi_L);
    const float sR = sinf(_chi_R), cR = cosf(_chi_R);

    // Force columns [Fx, Fy, Fz]
    // ω_L_norm² column
    A_eff[0][0] = 2.0f * T_max * sL;    // Fx contribution
    A_eff[1][0] = 0.0f;
    A_eff[2][0] = -2.0f * T_max * cL;  // Fz contribution (upward = negative z)

    // ω_R_norm² column
    A_eff[0][1] = 2.0f * T_max * sR;
    A_eff[1][1] = 0.0f;
    A_eff[2][1] = -2.0f * T_max * cR;

    // Torque contributions (arm geometry)
    // τ = r × F (moment arm from CoG)
    // Left rotors at ±arm_y from centre in y, and ±arm_x in x
    // Roll torque from thrust: r_y × Fz
    A_eff[3][0] = -p.arm_y * (-2.0f * T_max * cL); // τx from left pair
    A_eff[3][1] =  p.arm_y * (-2.0f * T_max * cR); // τx from right pair
    // Pitch torque: -r_x × Fz  (front pair positive pitch, rear negative)
    A_eff[4][0] = 0.0f; // balanced front/rear (same χ)
    A_eff[4][1] = 0.0f;
    // Yaw torque from motor spin  (reaction torque)
    A_eff[5][0] = float(p.d_L) * 2.0f * p.c_K / (p.c_F * 1.0f) * T_max;
    A_eff[5][1] = float(p.d_R) * 2.0f * p.c_K / (p.c_F * 1.0f) * T_max;

    // ── Aerodynamic surface contributions ────────────────────────────────────
    const float Va2  = airspeed_ms * airspeed_ms;
    const float qbar = 0.5f * 1.225f * Va2;

    // Aileron → roll torque
    A_eff[3][2] = qbar * p.S_aileron * p.wingspan_b * p.CL_delta_a;
    // Elevator → pitch torque
    A_eff[4][3] = qbar * p.S_elevator * p.chord_c * p.CM_delta_e;
    // Rudder → yaw torque
    A_eff[5][4] = qbar * p.S_rudder * p.wingspan_b * p.CN_delta_r;
}

// Gradient projection solver for constrained least-squares allocation
// Minimises 0.5 * ||A*Δu - ΔW||²  s.t.  lb ≤ Δu ≤ ub
void ControlAllocator::solve_alloc_qp(const float A_eff[6][5],
                                       const float dW[6],
                                       const float lb[5],
                                       const float ub[5],
                                       float du[5]) const
{
    // Gradient:  g = A^T * (A*du - dW)
    // Hessian:   H = A^T * A  (positive semi-definite)
    // Iterate:   du_new = clip(du - α * g, lb, ub)

    memset(du, 0, sizeof(float) * 5);

    // Pre-compute A^T * A (5×5) for step-size selection
    float AtA_diag[5] = {};
    for (int j = 0; j < 5; j++) {
        for (int i = 0; i < 6; i++) {
            AtA_diag[j] += A_eff[i][j] * A_eff[i][j];
        }
        AtA_diag[j] = MAX(AtA_diag[j], 1e-6f);
    }

    for (int iter = 0; iter < 30; iter++) {
        // Compute residual r = A*du - dW
        float r[6] = {};
        for (int i = 0; i < 6; i++) {
            r[i] = -dW[i];
            for (int j = 0; j < 5; j++) {
                r[i] += A_eff[i][j] * du[j];
            }
        }

        // Gradient g = A^T * r
        float g[5] = {};
        for (int j = 0; j < 5; j++) {
            for (int i = 0; i < 6; i++) {
                g[j] += A_eff[i][j] * r[i];
            }
        }

        // Check convergence
        float gnorm = 0.0f;
        for (int j = 0; j < 5; j++) gnorm += g[j] * g[j];
        if (gnorm < 1e-8f) break;

        // Steepest descent step with per-variable step size (diagonal precond.)
        for (int j = 0; j < 5; j++) {
            du[j] = constrain_float(du[j] - g[j] / AtA_diag[j], lb[j], ub[j]);
        }
    }
}

void ControlAllocator::allocate(const float desired_accel_body[3],
                                 const float desired_torque_body[3],
                                 float airspeed_ms,
                                 float u_motor_out[2],
                                 float u_surface_out[3])
{
    // Desired wrench in body frame
    const float W_sp[6] = {
        desired_accel_body[0] * _p.mass_kg,
        desired_accel_body[1] * _p.mass_kg,
        desired_accel_body[2] * _p.mass_kg,
        desired_torque_body[0],
        desired_torque_body[1],
        desired_torque_body[2]
    };

    // Current wrench from previous actuator setpoint
    float A_eff[6][5];
    build_effectiveness(airspeed_ms, A_eff);

    float W_curr[6] = {};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 5; j++) {
            W_curr[i] += A_eff[i][j] * _u_prev[j];
        }
    }

    // Desired wrench change
    float dW[6];
    for (int i = 0; i < 6; i++) dW[i] = W_sp[i] - W_curr[i];

    // Bounds on Δu based on current setpoint (equation 29)
    float lb[5], ub[5];
    // Bounds: motors [0..1], control surfaces [-1..1]
    const float lb_abs[5] = {0.0f, 0.0f, -1.0f, -1.0f, -1.0f};
    const float ub_abs[5] = {1.0f, 1.0f,  1.0f,  1.0f,  1.0f};
    for (int j = 0; j < 5; j++) {
        lb[j] = lb_abs[j] - _u_prev[j];
        ub[j] = ub_abs[j] - _u_prev[j];
    }

    float du[5];
    solve_alloc_qp(A_eff, dW, lb, ub, du);

    // Update actuator setpoint (equation 24)
    for (int j = 0; j < 5; j++) {
        _u_prev[j] = constrain_float(_u_prev[j] + du[j], lb_abs[j], ub_abs[j]);
    }

    u_motor_out[0]    = _u_prev[0]; // left motor pair  [0..1]
    u_motor_out[1]    = _u_prev[1]; // right motor pair [0..1]
    u_surface_out[0]  = _u_prev[2]; // aileron  [-1..1]
    u_surface_out[1]  = _u_prev[3]; // elevator [-1..1]
    u_surface_out[2]  = _u_prev[4]; // rudder   [-1..1]
}

// ─────────────────────────────────────────────────────────────────────────────
// AP_VTOL_MPC implementation
// ─────────────────────────────────────────────────────────────────────────────
AP_VTOL_MPC::AP_VTOL_MPC()
    : _dyn(nullptr), _solver(nullptr), _alloc(nullptr), _initialised(false)
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);

    _params     = default_tilthvec_params();
    _weights    = MPCWeights::defaults();
    _constraints = MPCConstraints::defaults();

    memset(_x_current, 0, sizeof(_x_current));
    _airspeed_ms = 0.0f;
}

void AP_VTOL_MPC::init()
{
    if (_initialised) return;

    update_weights();

    _dyn   = NEW_NOTHROW VTOLDynamics(_params);
    _solver = NEW_NOTHROW MPCSolver(*_dyn, _weights, _constraints,
                                    _dt_ms * 0.001f, (int)_max_iter);
    _alloc  = NEW_NOTHROW ControlAllocator(_params);

    if (!_dyn || !_solver || !_alloc) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_VTOL_MPC: allocation failed");
        return;
    }

    _solver->reset(_x_current);
    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_VTOL_MPC: MPC controller initialised (Chen 2024)");
}

void AP_VTOL_MPC::update_weights()
{
    _weights.Qv[0] = _Qvx; _weights.Qv[1] = _Qvy; _weights.Qv[2] = _Qvz;
    _weights.Qf[0] = _Qvx; _weights.Qf[1] = _Qvy; _weights.Qf[2] = _Qvz;
    _weights.Qpsi[0] = _Qroll; _weights.Qpsi[1] = _Qpitch; _weights.Qpsi[2] = 10.0f;
    // Propagate configurable iteration count to solver
    if (_solver) _solver->set_max_iter((int)_max_iter);
}

void AP_VTOL_MPC::reset()
{
    if (_solver) _solver->reset(_x_current);
    if (_alloc) {
        // Reset allocation state — use tilt from current state
        _alloc->set_tilt(_x_current[3], _x_current[4]);
    }
}

void AP_VTOL_MPC::set_state(const Vector3f &vel_ned,
                             const Vector3f &euler_rad,
                             const Vector3f &omega_rads,
                             const Vector3f &omega_prev_rads,
                             float chi_L_rad,
                             float chi_R_rad,
                             float airspeed_ms)
{
    _x_current[0]  = vel_ned.x;
    _x_current[1]  = vel_ned.y;
    _x_current[2]  = vel_ned.z;
    _x_current[3]  = chi_L_rad;
    _x_current[4]  = chi_R_rad;
    _x_current[5]  = euler_rad.x;
    _x_current[6]  = euler_rad.y;
    _x_current[7]  = euler_rad.z;
    _x_current[8]  = omega_rads.x;
    _x_current[9]  = omega_rads.y;
    _x_current[10] = omega_rads.z;
    _x_current[11] = omega_prev_rads.x;
    _x_current[12] = omega_prev_rads.y;
    _x_current[13] = omega_prev_rads.z;
    _airspeed_ms   = airspeed_ms;

    if (_alloc) {
        _alloc->set_tilt(chi_L_rad, chi_R_rad);
    }
}

bool AP_VTOL_MPC::update(const Vector3f &vel_ref_ned,
                          float yaw_ref_rad,
                          int32_t att_out_cd[3],
                          float  &throttle_out,
                          float   chi_out_rad[2])
{
    if (!_initialised || !_enable) return false;

    // Update weights from parameters (allows runtime tuning)
    update_weights();
    if (_solver) {
        // Re-create solver if parameters changed significantly (not done here for speed)
    }

    // Velocity reference as flat array
    float v_ref[3] = {vel_ref_ned.x, vel_ref_ned.y, vel_ref_ned.z};

    // Run MPC solver
    float u_opt[VTOL_MPC_NU];
    if (!_solver->solve(_x_current, v_ref, yaw_ref_rad, _airspeed_ms, u_opt)) {
        return false;
    }

    MPCInputVec u_vec;
    u_vec.from_flat(u_opt);

    // ── Extract outputs ───────────────────────────────────────────────────────
    // Tilt angle commands: integrate one step forward
    const float dt_s = _dt_ms * 0.001f;
    chi_out_rad[0] = constrain_float(_x_current[3] + u_vec.chi_dot[0] * dt_s,
                                     _params.chi_min_rad, _params.chi_max_rad);
    chi_out_rad[1] = constrain_float(_x_current[4] + u_vec.chi_dot[1] * dt_s,
                                     _params.chi_min_rad, _params.chi_max_rad);

    // Attitude setpoints → centidegrees for ArduPilot attitude controller
    att_out_cd[0] = (int32_t)(u_vec.euler_d[0] * RAD_TO_DEG * 100.0f); // roll_cd
    att_out_cd[1] = (int32_t)(u_vec.euler_d[1] * RAD_TO_DEG * 100.0f); // pitch_cd
    // Yaw: use reference yaw wrapped to match current
    float yaw_err = wrap_PI(yaw_ref_rad - _x_current[7]);
    float yaw_d = _x_current[7] + constrain_float(yaw_err, -0.2f, 0.2f);
    att_out_cd[2] = (int32_t)(yaw_d * RAD_TO_DEG * 100.0f); // yaw_cd

    // Thrust → normalised [0..1]
    const float T_total = u_vec.T;
    throttle_out = constrain_float(T_total / (_params.T_max_N * 4.0f), 0.0f, 1.0f);

    return true;
}

#endif // HAL_QUADPLANE_ENABLED
