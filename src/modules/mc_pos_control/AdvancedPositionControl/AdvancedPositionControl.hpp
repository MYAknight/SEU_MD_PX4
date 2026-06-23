/****************************************************************************
 *
 *   Advanced Position Control for Morphing Quadrotor
 *   Supports: Gain-Scheduled PID, LQR, MPC (phased implementation)
 *
 ****************************************************************************/

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

// Include original PositionControl for fallback mode 0
#include "../PositionControl/PositionControl.hpp"
#include "../FlatnessFeedforward/FlatnessFeedforward.hpp"

struct AdvancedControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * Advanced Position Control for morphing quadrotor.
 * Mode 0: Original PositionControl (fallback)
 * Mode 1: Gain-Scheduled PID (interpolates gains based on arm_angle)
 * Mode 2: LQR Gain Scheduling (precomputed gains, state feedback)
 * Mode 3: Linear MPC (embedded QP solver)
 */
class AdvancedPositionControl
{
public:
	AdvancedPositionControl();
	~AdvancedPositionControl() = default;

	// Same interface as PositionControl for drop-in replacement
	void setPositionGains(const matrix::Vector3f &P);
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);
	void setThrustLimits(const float min, const float max);
	void setHorizontalThrustMargin(const float margin);
	void setTiltLimit(const float tilt);
	void setHoverThrust(const float hover_thrust);
	void updateHoverThrust(const float hover_thrust_new);

	void setState(const AdvancedControlStates &states);
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	bool update(const float dt);

	void resetIntegral();

	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	// Advanced-specific methods
	void setMode(int mode) { _mode = mode; }
	void setArmAngle(float arm_angle) { _arm_angle = arm_angle; }
	void setMpcAlpha(float alpha) { _mpc_alpha = alpha; }
	void setMpcRDelta(float r_delta) { _mpc_r_delta = r_delta; }

	// Flatness feedforward control
	void setUseFlatnessFeedforward(bool use) { _use_flatness_ff = use; }
	void setFlatnessBlend(float blend) { _flatness_blend = math::constrain(blend, 0.0f, 1.0f); }
	void setVehicleMass(float mass) { _vehicle_mass = math::max(mass, 0.1f); }
	void setFlatnessInput(const FlatnessFeedforward::FlatOutput &flat_output) { _flat_output = flat_output; _flat_output_valid = true; }

	// Gain-Scheduled PID: precomputed gain LUT
	struct GainSet {
		float pos_p_xy;
		float pos_p_z;
		float vel_p_xy;
		float vel_i_xy;
		float vel_d_xy;
		float vel_p_z;
		float vel_i_z;
		float vel_d_z;
	};

	static constexpr int GS_LUT_SIZE = 11; // matches HUAQICCC_LUT_SIZE
	void setGainLUT(const GainSet lut[GS_LUT_SIZE], float step, float max_angle);

	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	int _mode{0}; // 0=original, 1=GS-PID, 2=LQR, 3=MPC
	float _arm_angle{0.0f};
	float _arm_angle_prev{0.0f};
	float _morph_ff_z{-3.0f}; // feedforward gain for morphing-induced Z disturbance

	// Mode 0: original PID controller (wrapped)
	PositionControl _original_control;

	// Mode 1: Gain-Scheduled PID
	void _updateGainScheduled(const float dt);
	GainSet _gain_lut[GS_LUT_SIZE];
	float _gs_lut_step{0.05f};
	float _gs_lut_max_angle{-0.5f};
	bool _gs_lut_initialized{false};

	// Mode 2: LQR Gain Scheduling
	void _updateLQR(const float dt);
	GainSet _lqr_lut[GS_LUT_SIZE];
	bool _lqr_lut_initialized{false};

	// Mode 3: Linear MPC with gradient-projection QP solver
	static constexpr int MPC_N = 5;
	static constexpr int MPC_MAX_ITER = 50;
	static constexpr float MPC_TOL = 1e-3f;

	void _mpcControl(const float dt);
	float _mpcSolveAxis(float pos_err, float vel_err, float vel_sp_ref,
			    float u_min, float u_max,
			    const float H[MPC_N][MPC_N],
			    const float M[MPC_N][2],
			    const float g_const[MPC_N],
			    float alpha, float u_prev);

	// MPC precomputed matrices (per-axis, XY and Z may differ)
	float _mpc_H[MPC_N][MPC_N];
	float _mpc_M[MPC_N][2];
	float _mpc_g_const[MPC_N];
	// SITL uses more aggressive MPC defaults; real hardware keeps original conservative values.
#ifdef __PX4_POSIX
	float _mpc_alpha{5.0f};     // gradient step size matched to SITL-tuned H (max ~57)
	float _mpc_u_min_xy{-5.0f}; // SITL: less conservative XY acceleration bounds
	float _mpc_u_max_xy{5.0f};
#else
	float _mpc_alpha{20.0f};    // original HW step size (2/lambda_max ≈ 30.7)
	float _mpc_u_min_xy{-1.5f}; // original HW conservative bounds
	float _mpc_u_max_xy{1.5f};
#endif
	float _mpc_r_delta{0.005f};  // Batch 6: delta-u penalty weight for control smoothing
	float _mpc_u_prev[3]{0.0f, 0.0f, 0.0f};  // Batch 6: previous control for delta-u warm-start

	float _mpc_u_min_z{-8.0f};
	float _mpc_u_max_z{8.0f};

	// States and setpoints (shared across modes)
	matrix::Vector3f _pos;
	matrix::Vector3f _vel;
	matrix::Vector3f _vel_dot;
	matrix::Vector3f _vel_int;
	float _yaw{0.f};

	matrix::Vector3f _pos_sp;
	matrix::Vector3f _vel_sp;
	matrix::Vector3f _acc_sp;
	matrix::Vector3f _thr_sp;
	// (reserved)
	float _yaw_sp{0.f};
	float _yawspeed_sp{0.f};

	// Limits
	float _lim_vel_horizontal{10.f};
	float _lim_vel_up{3.f};
	float _lim_vel_down{1.f};
	float _lim_thr_min{0.1f};
	float _lim_thr_max{1.0f};
	float _lim_thr_xy_margin{0.3f};
	float _lim_tilt{M_PI_F / 4.f};
	float _hover_thrust{0.5f};

	// Gains (current, after interpolation for GS mode)
	matrix::Vector3f _gain_pos_p;
	matrix::Vector3f _gain_vel_p;
	matrix::Vector3f _gain_vel_i;
	matrix::Vector3f _gain_vel_d;

	// Flatness feedforward
	FlatnessFeedforward _flatness_ff;
	bool _use_flatness_ff{false};
	float _flatness_blend{0.3f};
	float _vehicle_mass{1.5f};
	FlatnessFeedforward::FlatOutput _flat_output{};
	bool _flat_output_valid{false};

	// Feedforward outputs
	matrix::Vector3f _ang_vel_ff{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _ang_acc_ff{0.0f, 0.0f, 0.0f};
	matrix::Vector3f _acc_sp_ff{0.0f, 0.0f, 0.0f}; // flatness acceleration feedforward [m/s^2]

	// Inertia LUT for flatness (J_xx, J_yy, J_zz at each arm_angle)
	float _J_lut[GS_LUT_SIZE][3];
	bool _J_lut_initialized{false};

	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid() const;
	void _positionControl();
	void _velocityControl(const float dt);
	void _accelerationControl();
	void _interpolateGainsFromLUT();
};
