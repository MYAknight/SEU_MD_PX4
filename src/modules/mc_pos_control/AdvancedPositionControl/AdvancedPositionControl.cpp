/****************************************************************************
 *
 *   Advanced Position Control for Morphing Quadrotor
 *
 ****************************************************************************/

#include "AdvancedPositionControl.hpp"
#include "../PositionControl/ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s AdvancedPositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

AdvancedPositionControl::AdvancedPositionControl()
{
	// Initialize all state/setpoint vectors to safe defaults
	_pos.setZero();
	_vel.setZero();
	_vel_dot.setZero();
	_vel_int.setZero();
	_yaw = 0.f;

	_pos_sp.setAll(NAN);
	_vel_sp.setAll(NAN);
	_acc_sp.setAll(NAN);
	_thr_sp.setZero();
	_yaw_sp = NAN;
	_yawspeed_sp = NAN;

	_gain_pos_p.setZero();
	_gain_vel_p.setZero();
	_gain_vel_i.setZero();
	_gain_vel_d.setZero();

	// Pre-computed gain schedule LUT for huaqiccc morphing drone
	// Tuned from huaqiccc ROMFS defaults: XY_P=4.0, Z_P=1.0, XY_VEL_P=2.2, XY_VEL_I=0.4, XY_VEL_D=0.2
	//                                        Z_VEL_P=4.0, Z_VEL_I=2.0, Z_VEL_D=0.0
	// Index 0 = angle 0.0 (closed, near huaqiccc defaults)
	// Index 10 = angle -0.50 (fully expanded, +20~25% more aggressive)
	// Gains interpolated linearly between grid points
	static constexpr GainSet default_lut[GS_LUT_SIZE] = {
		/* idx=0, angle=0.00 */ {4.000f, 1.000f, 2.200f, 0.600f, 0.200f, 4.000f, 4.000f, 0.000f},
		/* idx=1, angle=-0.05 */ {4.080f, 1.020f, 2.240f, 0.615f, 0.200f, 4.080f, 4.080f, 0.000f},
		/* idx=2, angle=-0.10 */ {4.160f, 1.040f, 2.280f, 0.630f, 0.210f, 4.160f, 4.160f, 0.000f},
		/* idx=3, angle=-0.15 */ {4.240f, 1.060f, 2.320f, 0.645f, 0.210f, 4.240f, 4.240f, 0.000f},
		/* idx=4, angle=-0.20 */ {4.320f, 1.080f, 2.360f, 0.660f, 0.220f, 4.320f, 4.320f, 0.000f},
		/* idx=5, angle=-0.25 */ {4.400f, 1.100f, 2.400f, 0.675f, 0.220f, 4.400f, 4.400f, 0.000f},
		/* idx=6, angle=-0.30 */ {4.480f, 1.120f, 2.440f, 0.690f, 0.230f, 4.480f, 4.480f, 0.000f},
		/* idx=7, angle=-0.35 */ {4.560f, 1.140f, 2.480f, 0.705f, 0.230f, 4.560f, 4.560f, 0.000f},
		/* idx=8, angle=-0.40 */ {4.640f, 1.160f, 2.520f, 0.720f, 0.240f, 4.640f, 4.640f, 0.000f},
		/* idx=9, angle=-0.45 */ {4.720f, 1.180f, 2.560f, 0.735f, 0.240f, 4.720f, 4.720f, 0.000f},
		/* idx=10, angle=-0.50 */ {4.800f, 1.200f, 2.600f, 0.750f, 0.250f, 4.800f, 4.800f, 0.000f}
	};
	setGainLUT(default_lut, 0.05f, -0.5f);

	// Pre-computed LQR gain schedule LUT
	// Designed from continuous-time LQR for double integrator:
	//   A=[[0,1],[0,0]], B=[[0],[1]], Q=diag(q_pos,q_vel), R=r
	//   k_pos = sqrt(q_pos/r), k_vel = sqrt((2*sqrt(q_pos*r)+q_vel)/r)
	// As arm_angle→-0.5 (expanded), inertia increases → larger Q for compensation
	static constexpr GainSet lqr_lut[GS_LUT_SIZE] = {
		/* idx= 0, angle=  0.00 */ {4.000f, 1.000f, 2.915f, 0.600f, 0.200f, 2.000f, 3.000f, 0.000f},
		/* idx= 1, angle= -0.05 */ {4.087f, 1.025f, 2.953f, 0.615f, 0.205f, 2.037f, 3.060f, 0.000f},
		/* idx= 2, angle= -0.10 */ {4.171f, 1.049f, 2.990f, 0.630f, 0.210f, 2.073f, 3.120f, 0.000f},
		/* idx= 3, angle= -0.15 */ {4.254f, 1.072f, 3.026f, 0.645f, 0.215f, 2.108f, 3.180f, 0.000f},
		/* idx= 4, angle= -0.20 */ {4.336f, 1.095f, 3.061f, 0.660f, 0.220f, 2.143f, 3.240f, 0.000f},
		/* idx= 5, angle= -0.25 */ {4.416f, 1.118f, 3.095f, 0.675f, 0.225f, 2.176f, 3.300f, 0.000f},
		/* idx= 6, angle= -0.30 */ {4.494f, 1.140f, 3.129f, 0.690f, 0.230f, 2.209f, 3.360f, 0.000f},
		/* idx= 7, angle= -0.35 */ {4.572f, 1.162f, 3.161f, 0.705f, 0.235f, 2.241f, 3.420f, 0.000f},
		/* idx= 8, angle= -0.40 */ {4.648f, 1.183f, 3.193f, 0.720f, 0.240f, 2.273f, 3.480f, 0.000f},
		/* idx= 9, angle= -0.45 */ {4.722f, 1.204f, 3.224f, 0.735f, 0.245f, 2.304f, 3.540f, 0.000f},
		/* idx=10, angle= -0.50 */ {4.796f, 1.225f, 3.254f, 0.750f, 0.250f, 2.334f, 3.600f, 0.000f}
	};
	for (int i = 0; i < GS_LUT_SIZE; ++i) {
		_lqr_lut[i] = lqr_lut[i];
	}
	_lqr_lut_initialized = true;

	// Pre-computed MPC matrices for gradient-projection QP solver
	// Model: x_{k+1} = A*x_k + B*u_k + W*vel_sp,  N=5, dt=0.05s
	// H = Gamma^T Q_bar Gamma + R_bar  (5x5 Hessian)
	// Q=diag(16.0, 0.5), R=0.01. Diagonal corrected: removed erroneous +1.0 offset.
	// Batch 7 fix: recompute H with dt=0.02s (actual loop rate) and R=0.03
	// to eliminate steady-state chattering caused by R=0.01 being too small.
	static constexpr float mpc_H_init[MPC_N][MPC_N] = {
		{0.031106f, 0.000874f, 0.000645f, 0.000422f, 0.000206f},
		{0.000874f, 0.030854f, 0.000634f, 0.000417f, 0.000204f},
		{0.000645f, 0.000634f, 0.030622f, 0.000412f, 0.000203f},
		{0.000422f, 0.000417f, 0.000412f, 0.030406f, 0.000202f},
		{0.000206f, 0.000204f, 0.000203f, 0.000202f, 0.030201f}
	};
	// M = Gamma^T Q_bar Phi  (5x2 state-to-gradient mapping)
	// CORRECTED: sign flipped + q_vel=0.5
	static constexpr float mpc_M_init[MPC_N][2] = {
		{-0.080000f, -0.054480f},
		{-0.051200f, -0.043200f},
		{-0.028800f, -0.031984f},
		{-0.012800f, -0.020960f},
		{-0.003200f, -0.010256f}
	};
	// g_const = Gamma^T Q_bar Omega  (5x1 vel_sp-to-gradient mapping)
	static constexpr float mpc_g_const_init[MPC_N] = {
		0.004480f, 0.003200f, 0.001984f, 0.000960f, 0.000256f
	};
	for (int i = 0; i < MPC_N; ++i) {
		_mpc_g_const[i] = mpc_g_const_init[i];
		for (int j = 0; j < MPC_N; ++j) {
			_mpc_H[i][j] = mpc_H_init[i][j];
		}
		for (int j = 0; j < 2; ++j) {
			_mpc_M[i][j] = mpc_M_init[i][j];
		}
	}
	_mpc_alpha = 20.0f;  // Batch 7: matched to dt=0.02s H matrix eigenvalues

	// Initialize inertia LUT for flatness feedforward (placeholder values)
	// TODO: replace with measured inertia values for each arm_angle
	for (int i = 0; i < GS_LUT_SIZE; ++i) {
		// Linear interpolation from folded to expanded inertia
		float t = static_cast<float>(i) / (GS_LUT_SIZE - 1);
		_J_lut[i][0] = 0.015f + 0.005f * t;  // J_xx
		_J_lut[i][1] = 0.015f + 0.005f * t;  // J_yy
		_J_lut[i][2] = 0.025f + 0.008f * t;  // J_zz
	}
	_J_lut_initialized = true;
}

void AdvancedPositionControl::setPositionGains(const Vector3f &P)
{
	_gain_pos_p = P;
	_original_control.setPositionGains(P);
}

void AdvancedPositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
	_original_control.setVelocityGains(P, I, D);
}

void AdvancedPositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
	_original_control.setVelocityLimits(vel_horizontal, vel_up, vel_down);
}

void AdvancedPositionControl::setThrustLimits(const float min, const float max)
{
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
	_original_control.setThrustLimits(min, max);
}

void AdvancedPositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
	_original_control.setHorizontalThrustMargin(margin);
}

void AdvancedPositionControl::setTiltLimit(const float tilt)
{
	_lim_tilt = tilt;
	_original_control.setTiltLimit(tilt);
}

void AdvancedPositionControl::setHoverThrust(const float hover_thrust)
{
	_hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX);
	_original_control.setHoverThrust(hover_thrust);
}

void AdvancedPositionControl::updateHoverThrust(const float hover_thrust_new)
{
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	if (_mode == 0) {
		_original_control.updateHoverThrust(hover_thrust_new);

	} else {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
			       + CONSTANTS_ONE_G - _acc_sp(2);
	}
}

void AdvancedPositionControl::setState(const AdvancedControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;

	PositionControlStates original_states;
	original_states.position = states.position;
	original_states.velocity = states.velocity;
	original_states.acceleration = states.acceleration;
	original_states.yaw = states.yaw;
	_original_control.setState(original_states);
}

void AdvancedPositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	_original_control.setInputSetpoint(setpoint);
}

void AdvancedPositionControl::setGainLUT(const GainSet lut[GS_LUT_SIZE], float step, float max_angle)
{
	for (int i = 0; i < GS_LUT_SIZE; ++i) {
		_gain_lut[i] = lut[i];
	}

	_gs_lut_step = step;
	_gs_lut_max_angle = max_angle;
	_gs_lut_initialized = true;
}

void AdvancedPositionControl::_interpolateGainsFromLUT()
{
	bool initialized = false;
	const GainSet *lut = nullptr;

	if (_mode == 2) {
		initialized = _lqr_lut_initialized;
		lut = _lqr_lut;
	} else {
		initialized = _gs_lut_initialized;
		lut = _gain_lut;
	}

	if (!initialized || lut == nullptr) {
		return;
	}

	// Clamp angle to valid range (negative = expanded)
	float angle = _arm_angle;

	if (angle > 0.0f) { angle = 0.0f; }

	if (angle < _gs_lut_max_angle) { angle = _gs_lut_max_angle; }

	// Compute interpolation index and weight
	float abs_angle = fabsf(angle);
	float idx_f = abs_angle / _gs_lut_step;
	int idx_low = static_cast<int>(floorf(idx_f));
	int idx_high = idx_low + 1;
	float w_high = idx_f - static_cast<float>(idx_low);

	if (idx_low < 0) { idx_low = 0; }

	if (idx_high >= GS_LUT_SIZE) { idx_high = GS_LUT_SIZE - 1; }

	if (idx_low >= GS_LUT_SIZE) { idx_low = GS_LUT_SIZE - 1; }

	const GainSet &g_low = lut[idx_low];
	const GainSet &g_high = lut[idx_high];

	auto lerp = [&](float v_low, float v_high, float w) { return v_low * (1.f - w) + v_high * w; };

	_gain_pos_p(0) = lerp(g_low.pos_p_xy, g_high.pos_p_xy, w_high);
	_gain_pos_p(1) = lerp(g_low.pos_p_xy, g_high.pos_p_xy, w_high);
	_gain_pos_p(2) = lerp(g_low.pos_p_z, g_high.pos_p_z, w_high);

	_gain_vel_p(0) = lerp(g_low.vel_p_xy, g_high.vel_p_xy, w_high);
	_gain_vel_p(1) = lerp(g_low.vel_p_xy, g_high.vel_p_xy, w_high);
	_gain_vel_p(2) = lerp(g_low.vel_p_z, g_high.vel_p_z, w_high);

	_gain_vel_i(0) = lerp(g_low.vel_i_xy, g_high.vel_i_xy, w_high);
	_gain_vel_i(1) = lerp(g_low.vel_i_xy, g_high.vel_i_xy, w_high);
	_gain_vel_i(2) = lerp(g_low.vel_i_z, g_high.vel_i_z, w_high);

	_gain_vel_d(0) = lerp(g_low.vel_d_xy, g_high.vel_d_xy, w_high);
	_gain_vel_d(1) = lerp(g_low.vel_d_xy, g_high.vel_d_xy, w_high);
	_gain_vel_d(2) = lerp(g_low.vel_d_z, g_high.vel_d_z, w_high);
}

bool AdvancedPositionControl::update(const float dt)
{
	_arm_angle_prev = _arm_angle;

	if (_mode == 0) {
		return _original_control.update(dt);
	}

	bool valid = _inputValid();

	if (valid) {
		// FIX: sanitize persistent state before solve to prevent permanent
		// failure after a failsafe setpoint (position=NaN) pollutes accumulators.
		for (int i = 0; i < 3; ++i) {
			if (!PX4_ISFINITE(_mpc_u_prev[i])) {
				_mpc_u_prev[i] = 0.0f;
			}

			if (!PX4_ISFINITE(_vel_int(i))) {
				_vel_int(i) = 0.0f;
			}
		}

		if (_mode == 1 || _mode == 2) {
			_interpolateGainsFromLUT();
			_positionControl();
			_velocityControl(dt);

		} else if (_mode == 3) {
			// Pre-compute flatness acceleration feedforward before MPC
			_acc_sp_ff.zero();
			_ang_vel_ff.zero();
			_ang_acc_ff.zero();

			if (_use_flatness_ff && _flat_output_valid) {
				FlatnessFeedforward::Feedforward ff;
				// Interpolate inertia from LUT based on arm_angle
				float J_vals[3] = {_J_lut[0][0], _J_lut[0][1], _J_lut[0][2]};
				if (_J_lut_initialized) {
					float abs_angle = fabsf(_arm_angle);
					float idx_f = abs_angle / _gs_lut_step;
					int idx_low = static_cast<int>(floorf(idx_f));
					int idx_high = idx_low + 1;
					float w_high = idx_f - static_cast<float>(idx_low);
					if (idx_low < 0) { idx_low = 0; }
					if (idx_high >= GS_LUT_SIZE) { idx_high = GS_LUT_SIZE - 1; }
					if (idx_low >= GS_LUT_SIZE) { idx_low = GS_LUT_SIZE - 1; }
					for (int j = 0; j < 3; ++j) {
						J_vals[j] = _J_lut[idx_low][j] * (1.0f - w_high) + _J_lut[idx_high][j] * w_high;
					}
				}
				matrix::SquareMatrix3f J;
				J.setZero();
				J(0, 0) = J_vals[0];
				J(1, 1) = J_vals[1];
				J(2, 2) = J_vals[2];

				if (_flatness_ff.compute(_flat_output, _vehicle_mass, J, 0.0f, ff)) {
					// Flatness epsilon = [ẍ, ÿ, z̈+g] in NED
					// body_z = epsilon / ||epsilon|| (thrust direction, down in NED)
					// collective_thrust = m * ||epsilon||
					// Desired acceleration feedforward (trajectory acceleration):
					_acc_sp_ff = Vector3f(_flat_output.acc) * _flatness_blend;
					_ang_vel_ff = ff.angular_velocity * _flatness_blend;
					_ang_acc_ff = ff.angular_acceleration * _flatness_blend;
				}
			}

			_positionControl();
			_mpcControl(dt);
		}

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw;
	}

	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void AdvancedPositionControl::_positionControl()
{
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void AdvancedPositionControl::_velocityControl(const float dt)
{
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// Batch 4: Morphing feedforward compensation for Z-axis disturbance
	// Constrain morph_rate to avoid spikes from discrete MAVLink angle updates (20Hz vs 100Hz control)
	float morph_rate = (_arm_angle - _arm_angle_prev) / dt;
	morph_rate = math::constrain(morph_rate, -0.5f, 0.5f);
	_acc_sp(2) += morph_rate * _morph_ff_z;

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Tracking Anti-Windup for horizontal direction
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	const Vector2f acc_sp_xy = _acc_sp.xy();
	const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	ControlMath::setZeroIfNanVector3f(vel_error);
	// Use reduced integrator gain for PID/LQR mode to avoid windup during trajectory
	_vel_int += vel_error.emult(_gain_vel_i) * dt * 0.1f;
}

void AdvancedPositionControl::_accelerationControl()
{
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

float AdvancedPositionControl::_mpcSolveAxis(float pos_err, float vel_err, float vel_sp_ref,
		float u_min, float u_max,
		const float H[MPC_N][MPC_N],
		const float M[MPC_N][2],
		const float g_const[MPC_N],
		float alpha, float u_prev)
{
	float x0[2] = {pos_err, vel_err};

	// Compute g = M * x0 + g_const * vel_sp_ref
	float g[MPC_N];
	for (int i = 0; i < MPC_N; ++i) {
		g[i] = g_const[i] * vel_sp_ref + M[i][0] * x0[0] + M[i][1] * x0[1];
	}

	// Batch 8: removed manual delta-u penalty; R is already baked into H matrix
	(void)u_prev;  // still used for warm-start below

	// Gradient projection iterations (warm-start from previous solution)
	float u[MPC_N];
	u[0] = math::constrain(u_prev, u_min, u_max);
	for (int i = 1; i < MPC_N; ++i) {
		u[i] = math::constrain(u_prev, u_min, u_max);
	}

	for (int iter = 0; iter < MPC_MAX_ITER; ++iter) {
		float grad[MPC_N];
		float max_diff = 0.0f;

		for (int i = 0; i < MPC_N; ++i) {
			grad[i] = g[i];
			for (int j = 0; j < MPC_N; ++j) {
				grad[i] += H[i][j] * u[j];
			}
		}

		// Batch 8: no manual delta-u penalty; control smoothing via H matrix only

		for (int i = 0; i < MPC_N; ++i) {
			float u_new = math::constrain(u[i] - alpha * grad[i], u_min, u_max);
			max_diff = math::max(max_diff, fabsf(u_new - u[i]));
			u[i] = u_new;
		}

		if (max_diff < MPC_TOL) {
			break;
		}
	}

	return u[0];
}

void AdvancedPositionControl::_mpcControl(const float dt)
{
	// Constrain vertical velocity integral (same as _velocityControl)
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// Use interpolated gains for integral and derivative terms
	_interpolateGainsFromLUT();


	// Solve MPC for each axis independently
	Vector3f mpc_acc_sp;
	for (int axis = 0; axis < 3; ++axis) {
		float pos_err = _pos_sp(axis) - _pos(axis);
		float vel_sp_axis = PX4_ISFINITE(_vel_sp(axis)) ? _vel_sp(axis) : 0.0f;
		float vel_err = vel_sp_axis - _vel(axis);
		float u_min = (axis < 2) ? _mpc_u_min_xy : _mpc_u_min_z;
		float u_max = (axis < 2) ? _mpc_u_max_xy : _mpc_u_max_z;
		mpc_acc_sp(axis) = _mpcSolveAxis(pos_err, vel_err, vel_sp_axis,
						 u_min, u_max,
						 _mpc_H, _mpc_M, _mpc_g_const, _mpc_alpha, _mpc_u_prev[axis]);
		_mpc_u_prev[axis] = mpc_acc_sp(axis);  // save raw for warm-start
	}

	// Assemble acceleration setpoint: boost MPC output + small integrator + derivative
	Vector3f vel_error;
	for (int i = 0; i < 3; ++i) {
		vel_error(i) = PX4_ISFINITE(_vel_sp(i)) ? (_vel_sp(i) - _vel(i)) : 0.0f;
	}

	// MPC acceleration setpoint + reduced integrator + derivative
	Vector3f acc_sp_velocity = mpc_acc_sp + _vel_int * 0.3f - _vel_dot.emult(_gain_vel_d);

	_acc_sp = Vector3f(empty_trajectory_setpoint.acceleration);
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// Batch 4: Morphing feedforward compensation for Z-axis disturbance
	// Constrain morph_rate to avoid spikes from discrete MAVLink angle updates (20Hz vs 100Hz control)
	float morph_rate = (_arm_angle - _arm_angle_prev) / dt;
	morph_rate = math::constrain(morph_rate, -0.5f, 0.5f);
	_acc_sp(2) += morph_rate * _morph_ff_z;

	// Add flatness acceleration feedforward (trajectory acceleration)
	ControlMath::addIfNotNanVector3f(_acc_sp, _acc_sp_ff);

	_accelerationControl();

	// ---- Thrust allocation and anti-windup (same as _velocityControl) ----
	// Vertical integrator anti-windup
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.0f;

	if (thrust_max_xy_squared > 0.0f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Tracking Anti-Windup for horizontal direction
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);

	const Vector2f acc_sp_xy = _acc_sp.xy();
	const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	ControlMath::setZeroIfNanVector3f(vel_error);
	// Use full integrator gain for MPC to eliminate steady-state position error
	// (MPC effective position gain is ~12.5, requiring integrator to compensate)
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

bool AdvancedPositionControl::_inputValid() const
{
	bool valid = true;

	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void AdvancedPositionControl::resetIntegral()
{
	_vel_int.setZero();
	_original_control.resetIntegral();
}

void AdvancedPositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	if (_mode == 0) {
		_original_control.getLocalPositionSetpoint(local_position_setpoint);
		return;
	}

	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void AdvancedPositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	if (_mode == 0) {
		_original_control.getAttitudeSetpoint(attitude_setpoint);
		return;
	}

	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
