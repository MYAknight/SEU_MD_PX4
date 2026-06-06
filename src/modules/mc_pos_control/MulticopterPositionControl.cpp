/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD")
{
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		// Apply impedance control: reduce position stiffness during compliant contact
		Vector3f pos_gain(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get());
		if (_perching_phase == PerchingPhase::COMPLIANT && _param_mpca_pc_en.get() >= 3) {
			pos_gain *= _param_mpca_pc_k_soft.get();
		}
		_control.setPositionGains(pos_gain);
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());

		// Mirror settings to advanced controller (with impedance control)
		Vector3f adv_pos_gain(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get());
		if (_perching_phase == PerchingPhase::COMPLIANT && _param_mpca_pc_en.get() >= 3) {
			adv_pos_gain *= _param_mpca_pc_k_soft.get();
		}
		_advanced_control.setPositionGains(adv_pos_gain);
		_advanced_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		_advanced_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		_advanced_control.setMode(_param_mpca_mode.get());
		_advanced_control.setMpcAlpha(_param_mpca_mpc_alpha.get());
		_advanced_control.setMpcRDelta(_param_mpca_mpc_r_delta.get());
		_advanced_control.setUseFlatnessFeedforward(_param_mpca_ff_en.get() > 0);
		_advanced_control.setFlatnessBlend(_param_mpca_ff_blend.get());
		_advanced_control.setVehicleMass(_param_mpca_ff_mass.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_advanced_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		states.velocity.xy() = velocity_xy;
		states.acceleration(0) = _vel_x_deriv.update(velocity_xy(0));
		states.acceleration(1) = _vel_y_deriv.update(velocity_xy(1));

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {
		states.velocity(2) = vehicle_local_position.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		// set _dt in controllib Block for BlockDerivative
		setDt(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// Read current attitude for perching pitch tracking
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		// Record arm time for perching hysteresis
		if (_vehicle_control_mode.flag_armed && !_was_armed) {
			_perching_armed_time = hrt_absolute_time();
		}
		_was_armed = _vehicle_control_mode.flag_armed;

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		_trajectory_setpoint_sub.update(&_setpoint);

		// adjust existing (or older) setpoint with any EKF reset deltas
		if ((_setpoint.timestamp != 0) && (_setpoint.timestamp < vehicle_local_position.timestamp)) {
			if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
				_setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
				_setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
			}

			if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
				_setpoint.velocity[2] += vehicle_local_position.delta_vz;
			}

			if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
				_setpoint.position[0] += vehicle_local_position.delta_xy[0];
				_setpoint.position[1] += vehicle_local_position.delta_xy[1];
			}

			if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
				_setpoint.position[2] += vehicle_local_position.delta_z;
			}

			if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
				_setpoint.yaw = wrap_pi(_setpoint.yaw + vehicle_local_position.delta_heading);
			}
		}

		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			_vel_x_deriv.reset();
			_vel_y_deriv.reset();
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			_vel_z_deriv.reset();
		}

		// save latest reset counters
		_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
		_vz_reset_counter = vehicle_local_position.vz_reset_counter;
		_xy_reset_counter = vehicle_local_position.xy_reset_counter;
		_z_reset_counter = vehicle_local_position.z_reset_counter;
		_heading_reset_counter = vehicle_local_position.heading_reset_counter;


		PositionControlStates states{set_vehicle_states(vehicle_local_position)};


		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
				_advanced_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
				_advanced_control.resetIntegral();
			}

			// === GMO Contact Detection → Perching Grasp Trigger ===
			contact_state_s contact_state{};
			bool imu_contact_stable = false;
			if (_contact_state_sub.update(&contact_state)) {
				int pc_en = _param_mpca_pc_en.get();
				int pc_trig = _param_mpca_pc_trig.get();
				// MONITOR level (>=1): log impact for diagnosis
				if (pc_en >= 1 && contact_state.state == contact_state_s::STATE_POSSIBLE) {
					PX4_INFO("PC_MON: IMU impact force=%.2f", (double)sqrtf(contact_state.contact_force[0]*contact_state.contact_force[0]+contact_state.contact_force[1]*contact_state.contact_force[1]+contact_state.contact_force[2]*contact_state.contact_force[2]));
				}
				// DETECT level (>=2) and trigger source includes IMU (trig != 1)
				if (pc_en >= 2 && pc_trig != 1) {
					const bool perching_allowed = flying
						      && (_perching_armed_time > 0)
						      && (hrt_absolute_time() - _perching_armed_time > 8_s)
					      && (states.position(1) > _param_mpca_pc_gate.get());

					if (perching_allowed
					    && contact_state.state == contact_state_s::STATE_STABLE
					    && contact_state.should_close
					    && !_perching_active) {
						_perching_active = true;
							_perching_contact_x = states.position(1);
						_perching_start_time = hrt_absolute_time();
						mavlink_log_info(&_mavlink_log_pub, "Perching: contact stable, pushing forward to grasp");
					}

					if (_perching_active && contact_state.state == contact_state_s::STATE_NO_CONTACT) {
						const float perching_elapsed = (hrt_absolute_time() - _perching_start_time) * 1e-6f;
						if (perching_elapsed > 8.0f) {
							_perching_active = false;
							mavlink_log_info(&_mavlink_log_pub, "Perching: contact lost, releasing");
						}
					}
				}
				// Record IMU stable state for FSM when detection is enabled
				if (pc_en >= 2) {
					imu_contact_stable = (contact_state.state == contact_state_s::STATE_STABLE);
				}
			}

			// === Position-based Stall Detection (drone blocked by obstacle) ===
			// Detects when the drone cannot advance toward the setpoint,
			// indicating contact with the pole. No prior knowledge of pole
			// position is required — works purely on local motion.
			bool stall_detected = false;
			int pc_en = _param_mpca_pc_en.get();
			int pc_trig = _param_mpca_pc_trig.get();
			if (pc_en >= 2 && pc_trig != 2
			    && _vehicle_control_mode.flag_control_offboard_enabled
			    && flying) {
				// Use Y-axis (position[1]) for stall detection because the test
				// trajectory flies along X. For general use, this should be the
				// axis with the largest setpoint movement.
				float y_error = _setpoint.position[1] - states.position(1);
				float y_vel = states.velocity(1);

				// Require setpoint ahead and nearly stopped
				bool approaching = y_error > _param_mpca_pc_stall_err.get();
				bool nearly_stopped = fabsf(y_vel) < _param_mpca_pc_stall_vel.get();
				// Position gate: optional absolute position threshold.
				// Set MPCA_PC_GATE = 0.0 to disable and rely purely on local motion.
				bool near_pole_surface = true;
				float gate = _param_mpca_pc_gate.get();
				if (gate > 0.01f) {
					near_pole_surface = states.position(1) > gate;
				}

				// Additional check: position has barely moved in the last stall_t seconds
				if (approaching && nearly_stopped && near_pole_surface) {
					if (_stall_start_time == 0) {
						_stall_start_time = hrt_absolute_time();
						_stall_start_y = states.position(1);
						if (_perching_phase == PerchingPhase::NONE) {
							mavlink_log_info(&_mavlink_log_pub, "STALL_START");
						}
					} else {
						float stall_elapsed = (hrt_absolute_time() - _stall_start_time) * 1e-6f;
						float dy = states.position(1) - _stall_start_y;
						// Must be blocked for stall_t and moved < stall_d
						if (stall_elapsed > _param_mpca_pc_stall_t.get() && fabsf(dy) < _param_mpca_pc_stall_d.get()) {
							stall_detected = true;
							if (_perching_phase == PerchingPhase::NONE) {
								mavlink_log_info(&_mavlink_log_pub, "STALL_DETECTED");
							}
						}
					}
				} else {
					if (_stall_start_time != 0) {
						if (_perching_phase == PerchingPhase::NONE) {
							mavlink_log_info(&_mavlink_log_pub, "STALL_RESET");
						}
					}
					_stall_start_time = 0;
				}
			}

			// === Perching Phase State Machine ===
			// Auto-reset if perching disabled mid-flight
			if (pc_en < 2 && _perching_phase != PerchingPhase::NONE) {
				_perching_phase = PerchingPhase::NONE;
				_stall_start_time = 0;
				_grasp_secure = false;
				_perching_active = false;
				_imu_stable_ever = false;
			}
			if (pc_en >= 2 && _perching_phase == PerchingPhase::NONE && (stall_detected || (_perching_active && imu_contact_stable))) {
				_perching_phase = PerchingPhase::CONTACT;
				_perching_active = true;  // ensure existing perching logic also triggers
				_perching_contact_x = states.position(1);
				_perching_start_z = states.position(2);
				_perching_start_time = hrt_absolute_time();
				_grasp_secure = false;
				_imu_stable_ever = false;
				_grasp_check_x = states.position(1);
				_grasp_check_z = states.position(2);
				_grasp_check_time = hrt_absolute_time();
				mavlink_log_info(&_mavlink_log_pub, "Perching: contact detected, entering compliance");
			}

			if (_perching_phase == PerchingPhase::CONTACT) {
				float contact_elapsed = (hrt_absolute_time() - _perching_start_time) * 1e-6f;
				if (contact_elapsed > 1.0f) {
					_perching_phase = PerchingPhase::COMPLIANT;
					_compliant_sp_x = states.position(1) + _param_mpca_pc_preload.get();
					_compliant_integral_reset = false;
					_compliant_thrust_sum = 0.0f;
					_compliant_thrust_count = 0;
					_compliant_max_pitch = 0.0f;
					_compliant_motor_sum = 0.0f;
					_compliant_motor_count = 0;
					mavlink_log_info(&_mavlink_log_pub, "Perching: entering soft contact, impedance k_soft=%.2f",
						 (double)_param_mpca_pc_k_soft.get());
				}
			}

			if (_perching_phase == PerchingPhase::COMPLIANT) {
				float compliant_elapsed = (hrt_absolute_time() - _perching_start_time) * 1e-6f;

				// Reset integral once when entering COMPLIANT to eliminate windup
				if (!_compliant_integral_reset) {
					_control.resetIntegral();
					_advanced_control.resetIntegral();
					_compliant_integral_reset = true;
					mavlink_log_info(&_mavlink_log_pub, "Perching: integral reset for soft contact");
				}

				// Track max pitch during COMPLIANT (from vehicle attitude quaternion)
				if (vehicle_attitude.timestamp > 0) {
					matrix::Quatf q_att(vehicle_attitude.q);
					matrix::Eulerf euler(q_att);
					_compliant_max_pitch = math::max(_compliant_max_pitch, fabsf(euler.theta()));
				}

				// Grasp secure detection: require BOTH arms contracted AND position stable.
				// This ensures thrust is not reduced before the gripper has clamped the pole.
				if (!_grasp_secure && compliant_elapsed > 2.0f) {
					// Check every 1 second
					if (hrt_absolute_time() - _grasp_check_time > 1_s) {
						// Read latest IMU contact state
						contact_state_s cs{};
						bool imu_stable_now = false;
						if (_contact_state_sub.copy(&cs)) {
							imu_stable_now = (cs.state == contact_state_s::STATE_STABLE);
						}
						if (imu_stable_now) {
							_imu_stable_ever = true;
						}

						// Check arm angle if available
						huaqiccc_morph_angle_s morph_msg{};
						bool arms_contracted = false;
						bool have_arm_data = _huaqiccc_morph_angle_sub.copy(&morph_msg);
						if (have_arm_data) {
							// GRASP_ANGLE is typically -0.15 rad (from -0.45 expanded)
							// Arms are considered contracted if angle > -0.20 rad
							arms_contracted = (morph_msg.arm_angle > -0.30f);
						}

						float dx = states.position(1) - _grasp_check_x;
						float dz = states.position(2) - _grasp_check_z;
						bool pos_stable = (fabsf(dx) < 0.03f && fabsf(dz) < 0.05f);

						// Grasp secure when position is stable AND
						// either arms are contracted OR we have been compliant for >10s.
						// Time fallback always works regardless of arm data availability.
						bool grasp_ok = pos_stable && (arms_contracted || compliant_elapsed > 6.0f);

						if (grasp_ok) {
							_grasp_secure = true;
							mavlink_log_info(&_mavlink_log_pub, "Perching: grasp secure confirmed (arms=%d have_data=%d pos=%d elapsed=%.1f)",
							         (int)arms_contracted, (int)have_arm_data, (int)pos_stable, (double)compliant_elapsed);
						} else {
							_grasp_check_x = states.position(1);
							_grasp_check_z = states.position(2);
							_grasp_check_time = hrt_absolute_time();
							float dbg_angle = have_arm_data ? morph_msg.arm_angle : 99.0f;
							mavlink_log_info(&_mavlink_log_pub, "Perching: grasp check failed, arms=%d have_data=%d pos=%d angle=%.3f dx=%.3f dz=%.3f",
							         (int)arms_contracted, (int)have_arm_data, (int)pos_stable,
							         (double)dbg_angle, (double)dx, (double)dz);
						}
					}
				}

				// Minimum 6s in compliant before ramp-down (simulation relaxed)
				if (compliant_elapsed > 6.0f && _grasp_secure) {
					_perching_phase = PerchingPhase::RAMP_DOWN;
					_ramp_start_time = hrt_absolute_time();
					// Output COMPLIANT phase statistics
					if (_compliant_thrust_count > 0) {
						float avg_thrust = _compliant_thrust_sum / _compliant_thrust_count;
						float avg_motor = 0.0f;
						if (_compliant_motor_count > 0) {
							avg_motor = _compliant_motor_sum / _compliant_motor_count;
						}
						mavlink_log_info(&_mavlink_log_pub, "Perching: COMPLIANT stats avg_thrust=%.3f avg_motor=%.3f max_pitch=%.1fdeg samples=%d",
							 (double)avg_thrust, (double)avg_motor, (double)math::degrees(_compliant_max_pitch),
							 _compliant_thrust_count);
					}
					mavlink_log_info(&_mavlink_log_pub, "Perching: thrust ramp-down started");
				}
				// Safety timeout: abort if grasp never secures
				else if (compliant_elapsed > 20.0f) {
					_perching_phase = PerchingPhase::NONE;
					_stall_start_time = 0;
					_grasp_secure = false;
					// Print COMPLIANT stats on timeout for A/B validation
					if (_compliant_thrust_count > 0) {
						float avg_thrust = _compliant_thrust_sum / _compliant_thrust_count;
						float avg_motor = 0.0f;
						if (_compliant_motor_count > 0) {
							avg_motor = _compliant_motor_sum / _compliant_motor_count;
						}
						mavlink_log_info(&_mavlink_log_pub, "Perching: COMPLIANT stats avg_thrust=%.3f avg_motor=%.3f max_pitch=%.1fdeg samples=%d",
							 (double)avg_thrust, (double)avg_motor, (double)math::degrees(_compliant_max_pitch),
							 _compliant_thrust_count);
					}
					mavlink_log_info(&_mavlink_log_pub, "Perching: grasp timeout, aborting");
				}
			}

			if (_perching_phase == PerchingPhase::RAMP_DOWN) {
				float ramp_elapsed = (hrt_absolute_time() - _ramp_start_time) * 1e-6f;
				if (ramp_elapsed > _param_mpca_pc_ramp_t.get()) {
					_perching_phase = PerchingPhase::PERCHED;
					mavlink_log_info(&_mavlink_log_pub, "Perching: zero thrust, arms holding");
				}
			}

			// Safety: release if contact lost or height anomaly
			if (_perching_phase != PerchingPhase::NONE) {
				bool contact_lost = false;
				if (_perching_phase == PerchingPhase::CONTACT || _perching_phase == PerchingPhase::COMPLIANT) {
					// Early phase: only height drop can abort.
					// Do NOT abort based on IMU/stall because setpoint is locked
					// and stall detection naturally becomes false.
					contact_lost = false;
				} else {
					// RAMP_DOWN / PERCHED: only height drop can abort
					contact_lost = false;
				}
				bool height_drop = (states.position(2) < (_perching_start_z - 0.3f));
				if (contact_lost || height_drop) {
					_perching_phase = PerchingPhase::NONE;
					_stall_start_time = 0;
					_grasp_secure = false;
					_imu_stable_ever = false;
					mavlink_log_info(&_mavlink_log_pub, "Perching: abort, safety triggered");
				}
			}

				// limit tilt during takeoff ramupup
				const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();

				// Choose active controller
				const bool use_advanced = (_param_mpca_mode.get() > 0);

				// Read morph angle for advanced controller
				if (use_advanced) {
					huaqiccc_morph_angle_s morph_msg{};
					if (_huaqiccc_morph_angle_sub.copy(&morph_msg)) {
						_advanced_control.setArmAngle(morph_msg.arm_angle);
					}
				}

				// Perching active: override setpoint to push forward past the pole
				// Only do this during offboard control so landing mode is not affected
				if (_param_mpca_pc_en.get() >= 3
				    && (_perching_active || _perching_phase != PerchingPhase::NONE)
				    && _vehicle_control_mode.flag_control_offboard_enabled) {
					float orig_sp = _setpoint.position[1];
					if (_perching_phase == PerchingPhase::CONTACT) {
						// CONTACT: push 5cm past contact point
						_setpoint.position[1] = _perching_contact_x + 0.05f;
					} else if (_perching_phase == PerchingPhase::COMPLIANT) {
						// COMPLIANT: spring preload — current position + small offset
						_setpoint.position[1] = states.position(1) + _param_mpca_pc_preload.get();
					} else if (_perching_phase == PerchingPhase::RAMP_DOWN
						   || _perching_phase == PerchingPhase::PERCHED) {
						// Maintain position at contact point, no forward push
						_setpoint.position[1] = _perching_contact_x;
					} else {
						// Fallback to original 0.25m push
						_setpoint.position[1] = _perching_contact_x + 0.25f;
					}
					PX4_INFO("PERCHING: phase=%d, orig_sp=%.2f, new_sp=%.2f, contact_pos=%.2f",
					         (int)_perching_phase, (double)orig_sp, (double)_setpoint.position[1], (double)_perching_contact_x);
					// Keep original velocity/accel to avoid NAN mismatch with type_mask
					// The position override is sufficient to create the stall/compliance behavior
				}

				if (use_advanced) {
					_advanced_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

					const float speed_up = _takeoff.updateRamp(dt,
						       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
					const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
							 _param_mpc_z_vel_max_dn.get();

					const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
					_advanced_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

					float max_speed_xy = _param_mpc_xy_vel_max.get();

					if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
						max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
					}

					_advanced_control.setVelocityLimits(
						max_speed_xy,
						math::min(speed_up, _param_mpc_z_vel_max_up.get()),
						math::max(speed_down, 0.f));

					_advanced_control.setInputSetpoint(_setpoint);

					if (!PX4_ISFINITE(_setpoint.position[2])
					    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
					    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
						float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
						states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
					}

					AdvancedControlStates adv_states;
					adv_states.position = states.position;
					adv_states.velocity = states.velocity;
					adv_states.acceleration = states.acceleration;
					adv_states.yaw = states.yaw;
					_advanced_control.setState(adv_states);

					// Populate flatness feedforward inputs from trajectory setpoint
					FlatnessFeedforward::FlatOutput flat_out{};
					flat_out.pos = Vector3f(_setpoint.position);
					flat_out.vel = Vector3f(_setpoint.velocity);
					flat_out.acc = Vector3f(_setpoint.acceleration);
					flat_out.jerk = Vector3f(_setpoint.jerk);
					flat_out.snap.zero();
					flat_out.yaw = _setpoint.yaw;
					flat_out.yaw_dot = _setpoint.yawspeed;
					flat_out.yaw_ddot = 0.0f;
					// Arm angle: use latest morph angle
					huaqiccc_morph_angle_s morph_msg{};
					if (_huaqiccc_morph_angle_sub.copy(&morph_msg)) {
						flat_out.arm_angle = morph_msg.arm_angle;
					}
					flat_out.arm_angle_dot = 0.0f;
					flat_out.arm_angle_ddot = 0.0f;
					_advanced_control.setFlatnessInput(flat_out);

					if (!_advanced_control.update(dt)) {
						_vehicle_constraints = {0, NAN, NAN, false, {}};
						trajectory_setpoint_s fs_sp = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true);
						_advanced_control.setInputSetpoint(fs_sp);
						_advanced_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
						_advanced_control.update(dt);
					}

					vehicle_local_position_setpoint_s local_pos_sp{};
					_advanced_control.getLocalPositionSetpoint(local_pos_sp);
					local_pos_sp.timestamp = hrt_absolute_time();
					_local_pos_sp_pub.publish(local_pos_sp);

					vehicle_attitude_setpoint_s attitude_setpoint{};
					_advanced_control.getAttitudeSetpoint(attitude_setpoint);
					// Perching thrust management — Spring Model
					float hover = _param_mpc_thr_hover.get();
					if (_param_mpca_pc_en.get() >= 3) {
						if (_perching_phase == PerchingPhase::COMPLIANT) {
						// 1. Track thrust stats
						float thrust_mag = fabsf(attitude_setpoint.thrust_body[2]);
						_compliant_thrust_sum += thrust_mag;
						_compliant_thrust_count++;
						// 2. Spring model: recompute thrust so vertical component = hover thrust
						if (_param_mpca_pc_spring_en.get() > 0) {
							matrix::Quatf q_sp(attitude_setpoint.q_d);
							matrix::Dcmf R_sp(q_sp);
							float cos_tilt = R_sp(2, 2);  // body_z dot world_z
							if (cos_tilt > 0.01f) {
								float target_thrust = hover / cos_tilt;
								// Cap total thrust at 1.3x hover to avoid excessive thrust
								float max_thrust = hover * 1.3f;
								target_thrust = math::min(target_thrust, max_thrust);
								attitude_setpoint.thrust_body[2] = -target_thrust;
								PX4_INFO("COMPLIANT: spring mode, tilt=%.1fdeg cos=%.3f target_thrust=%.3f",
									 (double)math::degrees(acosf(cos_tilt)), (double)cos_tilt, (double)target_thrust);
							}
						} else {
							// Hard-push mode: disable spring correction, use raw controller output
							// Integrator windup will produce sustained high thrust
							PX4_INFO("COMPLIANT: hard-push mode, raw thrust=%.3f", (double)attitude_setpoint.thrust_body[2]);
						}
						// 3. Record motor outputs for closed-loop validation
						actuator_outputs_s act_out{};
						bool have_act = _actuator_outputs_sub.copy(&act_out);
						if (!have_act) { have_act = _actuator_outputs_hw_sub.copy(&act_out); }
						if (have_act && act_out.noutputs >= 4) {
							float avg_motor = (act_out.output[0] + act_out.output[1]
								   + act_out.output[2] + act_out.output[3]) / 4.0f;
							_compliant_motor_sum += avg_motor;
							_compliant_motor_count++;
						}
					} else if (_perching_phase == PerchingPhase::RAMP_DOWN) {
						float elapsed = (hrt_absolute_time() - _ramp_start_time) * 1e-6f;
						float tau = _param_mpca_pc_ramp_t.get();
						float alpha = math::constrain(elapsed / tau, 0.0f, 1.0f);
						// Exponential decay from hover thrust to zero
						float blend = expf(-3.0f * alpha);
						attitude_setpoint.thrust_body[2] = -hover * blend;
					} else if (_perching_phase == PerchingPhase::PERCHED) {
						// Mechanical arms provide holding force — no thrust needed
						attitude_setpoint.thrust_body[2] = 0.0f;
					}
					}
					attitude_setpoint.timestamp = hrt_absolute_time();
					_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

				} else {
					// Original PID path
					_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

					const float speed_up = _takeoff.updateRamp(dt,
						       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
					const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
							 _param_mpc_z_vel_max_dn.get();

					const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
					_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

					float max_speed_xy = _param_mpc_xy_vel_max.get();

					if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
						max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
					}

					_control.setVelocityLimits(
						max_speed_xy,
						math::min(speed_up, _param_mpc_z_vel_max_up.get()),
						math::max(speed_down, 0.f));

					_control.setInputSetpoint(_setpoint);

					if (!PX4_ISFINITE(_setpoint.position[2])
					    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
					    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
						float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
						states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
					}

					_control.setState(states);

					if (!_control.update(dt)) {
						_vehicle_constraints = {0, NAN, NAN, false, {}};
						_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
						_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
						_control.update(dt);
					}

					vehicle_local_position_setpoint_s local_pos_sp{};
					_control.getLocalPositionSetpoint(local_pos_sp);
					local_pos_sp.timestamp = hrt_absolute_time();
					_local_pos_sp_pub.publish(local_pos_sp);

					vehicle_attitude_setpoint_s attitude_setpoint{};
					_control.getAttitudeSetpoint(attitude_setpoint);
					// Perching thrust management — Spring Model
					float hover = _param_mpc_thr_hover.get();
					if (_param_mpca_pc_en.get() >= 3) {
						if (_perching_phase == PerchingPhase::COMPLIANT) {
						// 1. Track thrust stats
						float thrust_mag = fabsf(attitude_setpoint.thrust_body[2]);
						_compliant_thrust_sum += thrust_mag;
						_compliant_thrust_count++;
						// 2. Spring model: recompute thrust so vertical component = hover thrust
						if (_param_mpca_pc_spring_en.get() > 0) {
							matrix::Quatf q_sp(attitude_setpoint.q_d);
							matrix::Dcmf R_sp(q_sp);
							float cos_tilt = R_sp(2, 2);  // body_z dot world_z
							if (cos_tilt > 0.01f) {
								float target_thrust = hover / cos_tilt;
								// Cap total thrust at 1.3x hover to avoid excessive thrust
								float max_thrust = hover * 1.3f;
								target_thrust = math::min(target_thrust, max_thrust);
								attitude_setpoint.thrust_body[2] = -target_thrust;
								PX4_INFO("COMPLIANT: spring mode, tilt=%.1fdeg cos=%.3f target_thrust=%.3f",
									 (double)math::degrees(acosf(cos_tilt)), (double)cos_tilt, (double)target_thrust);
							}
						} else {
							// Hard-push mode: disable spring correction, use raw controller output
							// Integrator windup will produce sustained high thrust
							PX4_INFO("COMPLIANT: hard-push mode, raw thrust=%.3f", (double)attitude_setpoint.thrust_body[2]);
						}
						// 3. Record motor outputs for closed-loop validation
						actuator_outputs_s act_out{};
						bool have_act = _actuator_outputs_sub.copy(&act_out);
						if (!have_act) { have_act = _actuator_outputs_hw_sub.copy(&act_out); }
						if (have_act && act_out.noutputs >= 4) {
							float avg_motor = (act_out.output[0] + act_out.output[1]
								   + act_out.output[2] + act_out.output[3]) / 4.0f;
							_compliant_motor_sum += avg_motor;
							_compliant_motor_count++;
						}
					} else if (_perching_phase == PerchingPhase::RAMP_DOWN) {
						float elapsed = (hrt_absolute_time() - _ramp_start_time) * 1e-6f;
						float tau = _param_mpca_pc_ramp_t.get();
						float alpha = math::constrain(elapsed / tau, 0.0f, 1.0f);
						// Exponential decay from hover thrust to zero
						float blend = expf(-3.0f * alpha);
						attitude_setpoint.thrust_body[2] = -hover * blend;
					} else if (_perching_phase == PerchingPhase::PERCHED) {
						// Mechanical arms provide holding force — no thrust needed
						attitude_setpoint.thrust_body[2] = 0.0f;
					}
					}
					attitude_setpoint.timestamp = hrt_absolute_time();
					_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
				}

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
}

trajectory_setpoint_s MulticopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// don't move along xy
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// descend with land speed since we can't stop
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// don't move along z if we can stop in all dimensions
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		// emergency descend with a bit below hover thrust
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
