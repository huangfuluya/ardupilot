/*
 * AP_VTOL_MPC.h — Unified MPC controller for tilt-rotor VTOL UAV
 *
 * Implements the control strategy from:
 *   Chen et al., "A Unified MPC Strategy for a Tilt-rotor VTOL UAV
 *   Towards Seamless Mode Transitioning," AIAA SciTech 2024.
 *
 * Architecture (Figure 5 of the paper):
 *   Outer loop: MPC velocity controller (10 Hz in embedded, 100 Hz in SITL)
 *     - Input:  velocity setpoint v_sp in NED frame
 *     - Output: desired attitude Ψ_d, aggregate thrust T, tilt rate χ̇
 *   Inner loop: existing ArduPilot PID attitude controller (unchanged)
 *   Control allocation: QP maps wrench → individual actuator commands
 *
 * Coordinate conventions (consistent with ArduPilot and the paper):
 *   Inertial frame: NED  (North, East, Down)
 *   Body frame:  x → forward, y → right, z → down
 *   Attitude: roll (φ) / pitch (θ) / yaw (ψ) ZYX Euler angles
 *   Tilt angle χ: 0° = rotors pointing up (hover), 90° = rotors pointing fwd
 *
 * tilthvec SITL adaptation:
 *   The paper uses 4 independent tilt servos (χ1..χ4).
 *   ArduPilot's tilthvec model has 2 tilt servos (left=k_tiltMotorLeft,
 *   right=k_tiltMotorRight).  We therefore track χ_L (left) and χ_R (right).
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_QUADPLANE_ENABLED
#define HAL_QUADPLANE_ENABLED 1
#endif

#if HAL_QUADPLANE_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>

// ─────────────────────────────────────────────────────────────────────────────
// Compile-time dimensions
// ─────────────────────────────────────────────────────────────────────────────

// State vector: [vN, vE, vD, χ_L, χ_R, φ, θ, ψ, p, q, r, p⁻, q⁻, r⁻]  (n=14)
#define VTOL_MPC_NX   14
// Input vector: [χ̇_L, χ̇_R, T, φ_d, θ_d, ψ_d]  (m=6)
#define VTOL_MPC_NU   6
// Prediction horizon (paper uses N=25 at 100 Hz; we use 20 at 50 Hz update)
#define VTOL_MPC_N    20

// ─────────────────────────────────────────────────────────────────────────────
// Plain C-array helpers
// ─────────────────────────────────────────────────────────────────────────────

// nx×nx square matrix stored row-major
typedef float MPCMat_nn[VTOL_MPC_NX][VTOL_MPC_NX];
// nx×nu matrix
typedef float MPCMat_nm[VTOL_MPC_NX][VTOL_MPC_NU];
// nu×nu matrix
typedef float MPCMat_mm[VTOL_MPC_NU][VTOL_MPC_NU];

// ─────────────────────────────────────────────────────────────────────────────
// Vehicle physical parameters  (Table 1 of the paper)
// ─────────────────────────────────────────────────────────────────────────────
struct TilthvecParams {
    // Inertial
    float mass_kg;          // 7.427 kg
    float Ixx, Iyy, Izz;    // 10.685, 5.7465, 4.6678 kg·m²

    // Rotor geometry — 4 rotors in an H-frame, motors at arm tips
    // r_i: position of rotor i from CoG in body frame [m]
    // Arm length from centre (approximated from paper's geometry)
    float arm_x;            // 0.25 m  (half arm chord, approximate)
    float arm_y;            // 0.30 m  (half arm span, approximate)

    // Rotor aerodynamics
    float c_F;              // thrust coefficient  T = c_F * ω²  [N/(rad/s)²]
    float c_K;              // torque coefficient  τ = c_K * ω²  [N·m/(rad/s)²]
    float T_max_N;          // 27.36 N per rotor at 100% throttle
    // Normalised: 0..1 → 0..T_max  →  ω² = (u * T_max) / c_F

    // Aerodynamics (wing)
    float S_wing;           // 0.44 m²
    float S_aileron;        // 0.036 m²
    float S_elevator;       // 0.1364 m²
    float S_rudder;         // 0.004 m²
    float wingspan_b;       // 2.0 m
    float chord_c;          // 0.22 m
    float CD0;              // 0.35
    float CD_alpha;         // 0.11
    float CZ0;              // 0.03
    float CZ_alpha;         // 0.2
    float CL_delta_a;       // 0.1173
    float CM_delta_e;       // 0.556
    float CN_delta_r;       // 0.0881

    // Tilt servo constraints
    float chi_min_rad;      // -7° in rad
    float chi_max_rad;      // 95° in rad
    float chi_dot_max;      // π/4 rad/s

    // Rotor CW/CCW directions  (1 = CCW, -1 = CW)
    // Motor layout (tilthvec, H-frame, motors at 45°/135° etc):
    //   1 (FL) CW,  2 (BR) CW,  3 (BR) CCW,  4 (BL) CCW
    int8_t d_L;  // left side yaw factor  (average of motor 1 & 4)
    int8_t d_R;  // right side yaw factor (average of motor 2 & 3)
};

// Returns default paper / tilthvec parameters
TilthvecParams default_tilthvec_params();

// ─────────────────────────────────────────────────────────────────────────────
// MPC state and input structures
// ─────────────────────────────────────────────────────────────────────────────

struct MPCStateVec {
    float v[3];     // velocity NED [m/s]: v[0]=vN, v[1]=vE, v[2]=vD
    float chi[2];   // tilt angle [rad]: chi[0]=χ_L, chi[1]=χ_R
    float euler[3]; // roll φ, pitch θ, yaw ψ [rad]
    float omega[3]; // body angular rate p, q, r [rad/s]
    float omega_p[3]; // previous angular rate p⁻, q⁻, r⁻ [rad/s]

    // Helper to pack into a flat array
    void to_flat(float x[VTOL_MPC_NX]) const;
    void from_flat(const float x[VTOL_MPC_NX]);
};

struct MPCInputVec {
    float chi_dot[2]; // tilt rate [rad/s]: chi_dot[0]=χ̇_L, chi_dot[1]=χ̇_R
    float T;          // total aggregate thrust [N]
    float euler_d[3]; // desired attitude [rad]: φ_d, θ_d, ψ_d

    void to_flat(float u[VTOL_MPC_NU]) const;
    void from_flat(const float u[VTOL_MPC_NU]);
};

// ─────────────────────────────────────────────────────────────────────────────
// Dynamics model
// ─────────────────────────────────────────────────────────────────────────────

class VTOLDynamics {
public:
    explicit VTOLDynamics(const TilthvecParams &p);

    // Continuous-time dynamics: ẋ = f(x, u)
    void f(const float x[VTOL_MPC_NX],
           const float u[VTOL_MPC_NU],
           float xdot[VTOL_MPC_NX],
           float airspeed_ms = 0.0f) const;

    // Euler-forward integration  x_next = x + dt * f(x,u)
    void integrate(const float x[VTOL_MPC_NX],
                   const float u[VTOL_MPC_NU],
                   float dt,
                   float x_next[VTOL_MPC_NX],
                   float airspeed_ms = 0.0f) const;

    // Numerical Jacobians  (finite differences, used in iLQR backward pass)
    void jacobians(const float x[VTOL_MPC_NX],
                   const float u[VTOL_MPC_NU],
                   float dt,
                   MPCMat_nn A,   // ∂(x_next)/∂x
                   MPCMat_nm B,   // ∂(x_next)/∂u
                   float airspeed_ms = 0.0f) const;

    const TilthvecParams& params() const { return _p; }

private:
    TilthvecParams _p;

    // Aerodynamic effectiveness factor  (Figure 4 of paper)
    float aero_effectiveness(float airspeed_ms) const;

    // Rotation matrix body → inertial (DCM from Euler angles)
    void R_IB(const float euler[3], float R[3][3]) const;

    // Inner-loop PD attitude torque (equations 21-22)
    void inner_loop_torque(const float euler[3],
                            const float omega[3],
                            const float omega_p[3],
                            const float euler_d[3],
                            float tau_B[3]) const;

    // Gains for embedded inner-loop simulation in MPC state update
    // These match ArduPilot's Q_A_ANG_P and Q_A_RAT_P / Q_A_RAT_D
    static constexpr float KP_ATT  = 4.5f;   // attitude P gain  [1/s]
    static constexpr float KP_RATE = 0.15f;  // rate P gain
    static constexpr float KD_RATE = 0.002f; // rate D gain
};

// ─────────────────────────────────────────────────────────────────────────────
// Cost function weights  (Table 2 of the paper)
// ─────────────────────────────────────────────────────────────────────────────

struct MPCWeights {
    // Velocity tracking weights  Q_ref diagonal
    float Qv[3];   // [20, 10, 50]  (vN, vE, vD)
    // Terminal velocity weight  Q_f diagonal
    float Qf[3];   // [20, 10, 50]
    // Attitude weights  Q_ψ diagonal
    float Qpsi[3]; // [10, 20, 10]  (φ, θ, ψ)
    // Angular rate weights  Q_ψ̇ diagonal
    float Qomega[3]; // [3, 3, 3]
    // Input weights  R_u diagonal  [χ̇_L, χ̇_R, T, φ_d, θ_d, ψ_d]
    float Ru[VTOL_MPC_NU]; // [1, 1, 0.025, 10, 20, 10]

    // Soft constraint parameters  (equation 20 of paper)
    float soft_a, soft_b, soft_c, soft_d;

    static MPCWeights defaults();
};

// ─────────────────────────────────────────────────────────────────────────────
// MPC constraint bounds  (equation 16 of paper)
// ─────────────────────────────────────────────────────────────────────────────

struct MPCConstraints {
    // State bounds
    float v_max[3];      // [30, 30, 10] m/s
    float chi_min, chi_max;       // tilt angle per rotor [rad]
    float euler_max[3];  // |Ψ| ≤ [π/4, π/4, ∞]
    float omega_max[3];  // [π, π, π] rad/s

    // Input bounds
    float chi_dot_max;   // π/4 rad/s
    float T_min, T_max;  // [0, 4*27.36] total thrust [N]
    float euler_d_max[3]; // |Ψ_d| ≤ [π/2, π/3, π/3] rad

    static MPCConstraints defaults();
};

// ─────────────────────────────────────────────────────────────────────────────
// Gradient-based iLQR / receding-horizon solver
// ─────────────────────────────────────────────────────────────────────────────
// Uses gradient projection (first-order projected gradient descent) to solve
// the box-constrained receding-horizon optimal control problem.
// This is a practical approximation to the CasADi NLP used in the paper,
// suitable for 50–100 Hz real-time execution on SITL Linux targets.
// ─────────────────────────────────────────────────────────────────────────────

class MPCSolver {
public:
    MPCSolver(const VTOLDynamics &dyn,
              const MPCWeights   &w,
              const MPCConstraints &con,
              float dt_s);

    // Solve receding-horizon problem.
    //   x0        : current state (flat array, VTOL_MPC_NX)
    //   v_ref     : velocity reference NED [m/s]  (3 elements)
    //   yaw_ref   : yaw reference [rad]
    //   airspeed  : current airspeed [m/s]  (for aerodynamic model)
    //   u_out     : optimal first input (flat array, VTOL_MPC_NU) [output]
    // Returns true if solver converged, false if fallback needed.
    bool solve(const float x0[VTOL_MPC_NX],
               const float v_ref[3],
               float yaw_ref,
               float airspeed_ms,
               float u_out[VTOL_MPC_NU]);

    // Reset warm-start trajectory (e.g. on mode entry)
    void reset(const float x0[VTOL_MPC_NX]);

private:
    const VTOLDynamics   &_dyn;
    const MPCWeights     &_w;
    const MPCConstraints &_con;
    float _dt;

    // Warm-start input trajectory  U[k][m]
    float _U[VTOL_MPC_N][VTOL_MPC_NU];
    // Predicted state trajectory   X[k][n]  (k=0..N inclusive)
    float _X[VTOL_MPC_N + 1][VTOL_MPC_NX];

    // Compute total cost given predicted trajectory + reference
    float compute_cost(const float v_ref[3], float yaw_ref, float airspeed_ms) const;

    // Compute gradient dJ/dU[k] via adjoint (backward pass)
    void backward_pass(const float v_ref[3],
                       float yaw_ref,
                       float airspeed_ms,
                       float grad[VTOL_MPC_N][VTOL_MPC_NU]) const;

    // Project single input vector onto box constraints
    void project_u(float u[VTOL_MPC_NU]) const;

    // Project single state onto soft state bounds (for feasibility check)
    void clip_state(float x[VTOL_MPC_NX]) const;

    // Stage cost  L(x_k, u_k)
    float stage_cost(const float x[VTOL_MPC_NX],
                     const float u[VTOL_MPC_NU],
                     const float v_ref[3],
                     float airspeed_ms) const;

    // Terminal cost  L_N(x_N)
    float terminal_cost(const float x[VTOL_MPC_NX],
                        const float v_ref[3]) const;

    // Stage cost gradient  dL/dx and dL/du
    void stage_cost_grad(const float x[VTOL_MPC_NX],
                         const float u[VTOL_MPC_NU],
                         const float v_ref[3],
                         float airspeed_ms,
                         float dLdx[VTOL_MPC_NX],
                         float dLdu[VTOL_MPC_NU]) const;

    // Terminal cost gradient  dL_N/dx
    void terminal_cost_grad(const float x[VTOL_MPC_NX],
                            const float v_ref[3],
                            float dLdx[VTOL_MPC_NX]) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// Control allocation (Section III.C of the paper)
// ─────────────────────────────────────────────────────────────────────────────
// Maps desired wrench [Fx, Fy, Fz, τx, τy, τz] to actuator commands.
// Actuators for tilthvec: [ω_L², ω_R², δ_aileron, δ_elevator, δ_rudder]
// Uses gradient projection to minimise ||A·Δu - ΔW||²  s.t. bounds.
// ─────────────────────────────────────────────────────────────────────────────

class ControlAllocator {
public:
    explicit ControlAllocator(const TilthvecParams &p);

    // Update tilt state used in effectiveness matrix
    void set_tilt(float chi_L_rad, float chi_R_rad);

    // Compute actuator commands.
    //   desired_accel_body  : desired linear acceleration in body [m/s²] (3)
    //   desired_torque_body : desired angular acceleration × I [N·m]   (3)
    //   airspeed_ms         : for aero surface effectiveness
    //   u_motor_out         : normalised [0..1] throttle per left/right motor pair
    //   u_surface_out       : control surface deflections [-1..1] [ail, elev, rud]
    void allocate(const float desired_accel_body[3],
                  const float desired_torque_body[3],
                  float airspeed_ms,
                  float u_motor_out[2],
                  float u_surface_out[3]);

private:
    const TilthvecParams &_p;
    float _chi_L, _chi_R; // current tilt angles [rad]

    // Previous actuator setpoint (for incremental allocation, eq.24)
    float _u_prev[5]; // [ω_L², ω_R², δ_ail, δ_elev, δ_rud]

    // Build 6×5 effectiveness matrix A at current tilt + airspeed
    void build_effectiveness(float airspeed_ms,
                             float A_eff[6][5]) const;

    // Solve constrained least-squares  min||A*Δu - ΔW||²  s.t. lb≤Δu≤ub
    void solve_alloc_qp(const float A_eff[6][5],
                        const float dW[6],
                        const float lb[5],
                        const float ub[5],
                        float du[5]) const;
};

// ─────────────────────────────────────────────────────────────────────────────
// AP_VTOL_MPC — top-level ArduPilot-integrated controller
// ─────────────────────────────────────────────────────────────────────────────

class AP_VTOL_MPC {
public:
    AP_VTOL_MPC();

    static AP_VTOL_MPC *get_singleton() { return _singleton; }

    // ArduPilot parameter info
    static const AP_Param::GroupInfo var_info[];

    // Called once during system initialisation
    void init();

    // Called at the velocity controller rate (~50 Hz in real, up to 100 Hz SITL).
    // Returns true on success; false means caller should fall back.
    //
    //   vel_ref_ned   : desired velocity NED [m/s]
    //   yaw_ref_rad   : desired yaw [rad]
    //
    // Outputs (valid only when function returns true):
    //   att_out_cd    : roll_cd, pitch_cd, yaw_cd for attitude controller
    //   throttle_out  : normalised thrust [0..1] for QuadPlane motor system
    //   chi_out_rad   : tilt angle commands [rad]: [left, right]
    bool update(const Vector3f &vel_ref_ned,
                float yaw_ref_rad,
                int32_t att_out_cd[3],
                float  &throttle_out,
                float   chi_out_rad[2]);

    // Notify controller of current state (called each iteration before update)
    void set_state(const Vector3f &vel_ned,
                   const Vector3f &euler_rad,
                   const Vector3f &omega_rads,
                   const Vector3f &omega_prev_rads,
                   float chi_L_rad,
                   float chi_R_rad,
                   float airspeed_ms);

    // Reset (called on mode entry)
    void reset();

    // True if enabled by parameter
    bool enabled() const { return _enable > 0; }

    // ── ArduPilot parameters ──────────────────────────────────────────────────
    // @Param: ENABLE
    // @DisplayName: Enable VTOL MPC controller
    // @Description: Enables the unified MPC controller (Chen 2024). 0=disabled (use standard PID), 1=enabled
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    // @RebootRequired: True
    AP_Int8 _enable;

    // @Param: HORIZON
    // @DisplayName: MPC prediction horizon steps
    // @Description: Number of steps in the receding horizon. Paper uses 25 at 100Hz.
    // @Range: 5 25
    // @User: Advanced
    AP_Int8 _horizon;

    // @Param: DT_MS
    // @DisplayName: MPC solver time step (ms)
    // @Description: Integration step used inside the MPC prediction. Default 20ms (50Hz).
    // @Range: 10 50
    // @User: Advanced
    AP_Int8 _dt_ms;

    // @Param: QVX
    // @DisplayName: MPC velocity tracking weight Vx
    // @Description: Q_ref weight for North velocity tracking (paper: 20)
    // @Range: 1 100
    // @User: Advanced
    AP_Float _Qvx;

    // @Param: QVY
    // @DisplayName: MPC velocity tracking weight Vy
    // @Description: Q_ref weight for East velocity tracking (paper: 10)
    // @Range: 1 100
    // @User: Advanced
    AP_Float _Qvy;

    // @Param: QVZ
    // @DisplayName: MPC velocity tracking weight Vz
    // @Description: Q_ref weight for Down velocity tracking (paper: 50)
    // @Range: 1 100
    // @User: Advanced
    AP_Float _Qvz;

    // @Param: QROLL
    // @DisplayName: MPC attitude weight roll
    // @Description: Q_ψ weight for roll cost (paper: 10)
    // @Range: 1 100
    // @User: Advanced
    AP_Float _Qroll;

    // @Param: QPITCH
    // @DisplayName: MPC attitude weight pitch
    // @Description: Q_ψ weight for pitch cost (paper: 20)
    // @Range: 1 100
    // @User: Advanced
    AP_Float _Qpitch;

    // @Param: IQITR
    // @DisplayName: MPC gradient descent iterations
    // @Description: Maximum gradient projection iterations per solve call
    // @Range: 5 50
    // @User: Advanced
    AP_Int8 _max_iter;

private:
    static AP_VTOL_MPC *_singleton;

    // Sub-components
    TilthvecParams   _params;
    MPCWeights       _weights;
    MPCConstraints   _constraints;
    VTOLDynamics    *_dyn;
    MPCSolver       *_solver;
    ControlAllocator *_alloc;

    // Current vehicle state (set by set_state())
    float _x_current[VTOL_MPC_NX];
    float _airspeed_ms;

    bool _initialised;

    // Build weights from ArduPilot parameters
    void update_weights();
};

#endif // HAL_QUADPLANE_ENABLED
