/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file mc_pos_control_params.c
 * Multicopter position controller parameters.
 *
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Minimum collective thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 * Note: Without airmode zero thrust leads to zero roll/pitch control authority. (see MC_AIRMODE)
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 0.12f);

/**
 * Hover thrust
 *
 * Vertical thrust required to hover.
 * This value is mapped to center stick for manual throttle control.
 * With this value set to the thrust required to hover, transition
 * from manual to Altitude or Position mode while hovering will occur with the
 * throttle stick near center, which is then interpreted as (near)
 * zero demand for vertical speed.
 *
 * This parameter is also important for the landing detection to work correctly.
 *
 * @unit norm
 * @min 0.1
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_HOVER, 0.5f);

/**
 * Hover thrust source selector
 *
 * Set false to use the fixed parameter MPC_THR_HOVER
 * Set true to use the value computed by the hover thrust estimator
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_USE_HTE, 1);

/**
 * Thrust curve in Manual Mode
 *
 * This parameter defines how the throttle stick input is mapped to commanded thrust
 * in Manual/Stabilized flight mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see MPC_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: In case MPC_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_THR_CURVE, 0);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when prioritizing vertical thrust.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_XY_MARG, 0.3f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 1.0f);

/**
 * Minimum manual thrust
 *
 * Minimum vertical thrust. It's recommended to set it > 0 to avoid free fall with zero thrust.
 * With MC_AIRMODE set to 1, this can safely be set to 0.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MANTHR_MIN, 0.08f);

/**
 * Proportional gain for vertical position error
 *
 * @min 0.0
 * @max 1.5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_P, 1.0f);

/**
 * Proportional gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s velocity error
 *
 * @min 2.0
 * @max 15.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_P_ACC, 4.0f);

/**
 * Integral gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m velocity integral
 *
 * Non zero value allows hovering thrust estimation on stabilized or autonomous takeoff.
 *
 * @min 0.2
 * @max 3.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_I_ACC, 2.0f);

/**
 * Differential gain for vertical velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_D_ACC, 0.0f);

/**
 * Automatic ascent velocity
 *
 * Ascent velocity in auto modes.
 * For manual modes and offboard, see MPC_Z_VEL_MAX_UP
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_UP, 3.f);

/**
 * Maximum ascent velocity
 *
 * Ascent velocity in manual modes and offboard.
 * For auto modes, see MPC_Z_V_AUTO_UP
 *
 * @unit m/s
 * @min 0.5
 * @max 8.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 3.f);

/**
 * Automatic descent velocity
 *
 * Descent velocity in auto modes.
 * For manual modes and offboard, see MPC_Z_VEL_MAX_DN
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_V_AUTO_DN, 1.5f);

/**
 * Maximum descent velocity
 *
 * Descent velocity in manual modes and offboard.
 * For auto modes, see MPC_Z_V_AUTO_DN
 *
 * @unit m/s
 * @min 0.5
 * @max 4.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 1.5f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_P, 0.95f);

/**
 * Proportional gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_P_ACC, 1.8f);

/**
 * Integral gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m velocity integral
 * Non-zero value allows to eliminate steady state errors in the presence of disturbances like wind.
 *
 * @min 0.0
 * @max 60.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_I_ACC, 0.4f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * defined as correction acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.1
 * @max 2.0
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_D_ACC, 0.2f);

/**
 * Default horizontal velocity in mission
 *
 * Horizontal velocity used when flying autonomously in e.g. Missions, RTL, Goto.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_CRUISE, 5.0f);

/**
 * Proportional gain for horizontal trajectory position error
 *
 * @min 0.1
 * @max 1.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_TRAJ_P, 0.5f);

/**
 * Maximum horizontal error allowed by the trajectory generator
 *
 * The integration speed of the trajectory setpoint is linearly
 * reduced with the horizontal position tracking error. When the
 * error is above this parameter, the integration of the
 * trajectory is stopped to wait for the drone.
 *
 * This value can be adjusted depending on the tracking
 * capabilities of the vehicle.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_ERR_MAX, 2.0f);

/**
 * Maximum horizontal velocity setpoint in Position mode
 *
 * If velocity setpoint larger than MPC_XY_VEL_MAX is set, then
 * the setpoint will be capped to MPC_XY_VEL_MAX
 *
 * The maximum sideways and backward speed can be set differently
 * using MPC_VEL_MAN_SIDE and MPC_VEL_MAN_BACK, respectively.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MANUAL, 10.0f);

/**
 * Maximum sideways velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_SIDE, -1.0f);

/**
 * Maximum backward velocity in Position mode
 *
 * If set to a negative value or larger than
 * MPC_VEL_MANUAL then MPC_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VEL_MAN_BACK, -1.0f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 12.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.0f);

/**
 * Maximum tilt during landing
 *
 * Limits maximum tilt angle on landing.
 *
 * @unit deg
 * @min 10.0
 * @max 89.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND, 12.0f);

/**
 * Landing descend rate
 *
 * @unit m/s
 * @min 0.6
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_SPEED, 0.7f);

/**
 * Land crawl descend rate
 *
 * Used below MPC_LAND_ALT3 if distance sensor data is availabe.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_CRWL, 0.3f);

/**
 * Enable user assisted descent for autonomous land routine
 *
 * When enabled, descent speed will be:
 * stick full up - 0
 * stick centered - MPC_LAND_SPEED
 * stick full down - 2 * MPC_LAND_SPEED
 *
 * Additionally, the vehicle can be yawed and moved laterally using the other sticks.
 * Manual override during auto modes has to be disabled to use this feature (see COM_RC_OVERRIDE).
 *
 * @min 0
 * @max 1
 * @value 0 Fixed descent speed of MPC_LAND_SPEED
 * @value 1 User assisted descent speed
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_LAND_RC_HELP, 0);

/**
 * User assisted landing radius
 *
 * When user assisted descent is enabled (see MPC_LAND_RC_HELP),
 * this parameter controls the maximum position adjustment
 * allowed from the original landing point.
 *
 * @unit m
 * @min 0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_RADIUS, 1000.f);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_SPEED, 1.5f);

/**
 * Maximal tilt angle in manual or altitude mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_TILT_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_MAX, 150.0f);

/**
 * Manual yaw rate input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 0.08f);

/**
 * Deadzone of sticks where position hold is enabled
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_DZ, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_VELD_LP, 5.0f);

/**
 * Maximum horizontal acceleration for auto mode and for manual mode
 *
 * MPC_POS_MODE
 * 1 just deceleration
 * 3 acceleration and deceleration
 * 4 just acceleration
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_HOR_MAX, 5.0f);

/**
 * Acceleration for auto and for manual
 *
 * Note: In manual, this parameter is only used in MPC_POS_MODE 4.
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */

PARAM_DEFINE_FLOAT(MPC_ACC_HOR, 3.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes upward
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 4.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes down
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum jerk limit
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility (how fast it can change directions or break).
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Note: This is only used when MPC_POS_MODE is set to a smoothing mode 3 or 4.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500.0
 * @increment 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_MAX, 8.0f);

/**
 * Jerk limit in auto mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility.
 *
 * @unit m/s^3
 * @min 1.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_JERK_AUTO, 4.0f);

/**
 * Altitude control mode.
 *
 * Set to 0 to control height relative to the earth frame origin. This origin may move up and down in
 * flight due to sensor drift.
 * Set to 1 to control height relative to estimated distance to ground. The vehicle will move up and down
 * with terrain height variation. Requires a distance to ground sensor. The height controller will
 * revert to using height above origin if the distance to ground estimate becomes invalid as indicated
 * by the local_position.distance_bottom_valid message being false.
 * Set to 2 to control height relative to ground (requires a distance sensor) when stationary and relative
 * to earth frame origin when moving horizontally.
 * The speed threshold is controlled by the MPC_HOLD_MAX_XY parameter.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 0);

/**
 * Manual position control stick exponential curve sensitivity
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_MAN_EXPO, 0.6f);

/**
 * Manual control stick vertical exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_MAN_EXPO, 0.6f);

/**
 * Manual control stick yaw rotation exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_YAW_EXPO, 0.6f);

/**
 * Max yaw rate in auto mode
 *
 * Limit the rate of change of the yaw setpoint in autonomous mode
 * to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MPC_YAWRAUTO_MAX, 45.0f);

/**
 * Altitude for 1. step of slow landing (descend)
 *
 * Below this altitude descending velocity gets limited to a value
 * between "MPC_Z_VEL_MAX_DN" (or "MPC_Z_V_AUTO_DN") and "MPC_LAND_SPEED"
 * Value needs to be higher than "MPC_LAND_ALT2"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT1, 10.0f);

/**
 * Altitude for 2. step of slow landing (landing)
 *
 * Below this altitude descending velocity gets
 * limited to "MPC_LAND_SPEED"
 * Value needs to be lower than "MPC_LAND_ALT1"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT2, 5.0f);

/**
 * Altitude for 3. step of slow landing
 *
 * Below this altitude descending velocity gets
 * limited to "MPC_LAND_CRWL", if LIDAR available.
 * No effect if LIDAR not available
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_ALT3, 1.0f);

/**
 * Position control smooth takeoff ramp time constant
 *
 * Increasing this value will make automatic and manual takeoff slower.
 * If it's too slow the drone might scratch the ground and tip over.
 * A time constant of 0 disables the ramp
 *
 * @min 0
 * @max 5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TKO_RAMP_T, 3.0f);

/**
 * Manual-Position control sub-mode
 *
 * The supported sub-modes are:
 * 0 Simple position control where sticks map directly to velocity setpoints
 *   without smoothing. Useful for velocity control tuning.
 * 3 Smooth position control with maximum acceleration and jerk limits based on
 *   jerk optimized trajectory generator (different algorithm than 1).
 * 4 Smooth position control where sticks map to acceleration and there's a virtual brake drag
 *
 * @value 0 Simple position control
 * @value 3 Smooth position control (Jerk optimized)
 * @value 4 Acceleration based input
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_POS_MODE, 4);

/**
 * Yaw mode.
 *
 * Specifies the heading in Auto.
 *
 * @min 0
 * @max 4
 * @value 0 towards waypoint
 * @value 1 towards home
 * @value 2 away from home
 * @value 3 along trajectory
 * @value 4 towards waypoint (yaw first)
 * @group Mission
 */
PARAM_DEFINE_INT32(MPC_YAW_MODE, 0);

/**
 * Responsiveness
 *
 * Changes the overall responsiveness of the vehicle.
 * The higher the value, the faster the vehicle will react.
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * the acceleration or jerk limits).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -1
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(SYS_VEHICLE_RESP, -0.4f);

/**
 * Overall Horizontal Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * MPC_XY_VEL_MAX or MPC_VEL_MANUAL).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_ALL, -10.0f);

/**
 * Overall Vertical Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * MPC_Z_VEL_MAX_UP or MPC_LAND_SPEED).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -3
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_ALL, -3.0f);

/**
 * Advanced Position Control Mode
 *
 * Selects the position controller algorithm.
 * 0: Original cascaded PID (default)
 * 1: Gain-Scheduled PID (interpolates gains based on arm_angle)
 * 2: LQR Gain Scheduling
 * 3: Linear MPC
 *
 * @min 0
 * @max 3
 * @value 0 Original PID
 * @value 1 Gain-Scheduled PID
 * @value 2 LQR
 * @value 3 MPC
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPCA_MODE, 0);

/**
 * MPC gradient-projection step size (alpha)
 *
 * Larger values converge faster but may oscillate.
 * Must be tuned together with MPCA_MPC_R_DELTA.
 *
 * @min 1.0
 * @max 30.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_MPC_ALPHA, 20.0f);

/**
 * MPC delta-u penalty weight
 *
 * Penalizes control changes across MPC steps.
 * Higher values smoother control but slower response.
 *
 * @min 0.0
 * @max 0.1
 * @decimal 4
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_MPC_R_DELTA, 0.005f);

/**
 * Enable differential flatness feedforward in MPC mode
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPCA_FF_EN, 1);

/**
 * Flatness feedforward blending factor
 *
 * 0 = pure MPC feedback, 1 = pure flatness feedforward.
 * Start with 0.3 for safe tuning.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_FF_BLEND, 0.30f);

/**
 * Vehicle mass for flatness feedforward [kg]
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_FF_MASS, 1.50f);

/**
 * Perching/contact-detection master enable
 *
 * Master switch for the perching/contact-detection subsystem.
 * The previous IMU-ICD based detection (external_force_estimator) has been
 * deprecated on the real vehicle because raw IMU acceleration showed no
 * useful change during contact. Detection is now performed inside
 * mc_pos_control using position error, forward velocity and pitch attitude.
 *
 * 0 = OFF: No contact detection, no FSM, no setpoint override.
 * 1 = DETECT: Contact detection active and events are logged, but the
 *             perching FSM, setpoint override and thrust ramp-down are
 *             DISABLED. Use this mode to validate detection accuracy.
 * 2 = FULL: Complete perching system (detect + FSM + setpoint override + Spring Model).
 *
 * @min 0
 * @max 2
 * @value 0 OFF
 * @value 1 DETECT
 * @value 2 FULL
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPCA_PC_EN, 0);

/**
 * Perching compliance soft gain factor
 *
 * Multiplier for XY position P-gain during compliant contact phase.
 * Z-axis P-gain is kept at full value to hold height.
 * 0.5 = 50% of normal XY stiffness.
 *
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_K_SOFT, 0.50f);

/**
 * Perching thrust ramp-down time
 *
 * Time in seconds to linearly reduce thrust from hover to minimum.
 *
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_RAMP_T, 5.0f);

/**
 * Perching idle thrust ratio
 *
 * DEPRECATED: PERCHED phase now uses zero thrust (mechanical arms hold the drone).
 * This parameter only affects the RAMP_DOWN end-point for backward compatibility.
 * Set to 0 for pure spring-model behavior.
 *
 * @min 0.00
 * @max 0.50
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_IDLE_THR, 0.00f);

/**
 * Perching spring preload offset
 *
 * During COMPLIANT phase, the position setpoint is set to contact_position + preload.
 * This creates a small constant position error that acts as spring pre-compression,
 * generating a gentle restoring force to keep the drone against the pole.
 * A value of 0.05m means the drone tries to stay 5cm deeper than the contact point,
 * which matches manual-flight experience and prevents the drone from bouncing back.
 *
 * @unit m
 * @min 0.00
 * @max 0.10
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_PRELOAD, 0.030f);

/**
 * Perching arm contraction threshold
 *
 * Mechanical arm angle above which the gripper is considered contracted/grasping.
 * Typical AS5600 range: -0.45 rad (expanded) to 0.0 rad (fully closed).
 * Default -0.08 rad is tighter than before to avoid false positives from transient
 * contact deformation; the controller additionally requires the angle to stay above
 * this threshold for 0.5 seconds before declaring grasp secure.
 *
 * @unit rad
 * @min -0.50
 * @max 0.00
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ARM_THR, -0.08f);

/**
 * Perching spring model enable
 *
 * When enabled (1), COMPLIANT phase uses spring-model thrust recomputation
 * (thrust = hover / cos(tilt)) to keep vertical component at hover.
 * When disabled (0), the Z-axis position controller is used directly to hold height.
 * With the new compliant logic (Z gain normal, no integral reset), disabled is the
 * recommended setting and avoids the old 1.3x hover thrust cap issue.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPCA_PC_SPRING, 0);

/**
 * Perching trigger source selection
 *
 * Select which contact detection sources are allowed to trigger perching.
 * The IMU-ICD source (external_force_estimator) is deprecated on the real
 * vehicle and is no longer started by default. The position/attitude based
 * detector inside mc_pos_control is now the only active source.
 *
 * 0 = POSITION_ATTITUDE: Position-error + low forward velocity + pitch based detector.
 *
 * @min 0
 * @max 0
 * @value 0 POSITION_ATTITUDE
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPCA_PC_TRIG, 0);

/**
 * [DEPRECATED] Old stall-detection distance threshold
 *
 * The legacy stall detector has been replaced by a position/attitude based
 * contact detector. This parameter is kept only for parameter-format
 * compatibility and is no longer used.
 *
 * @unit m
 * @min 0.01
 * @max 0.50
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_STALL_D, 0.03f);

/**
 * [DEPRECATED] Old stall-detection timeout
 *
 * The legacy stall detector has been replaced by a position/attitude based
 * contact detector. This parameter is kept only for parameter-format
 * compatibility and is no longer used.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_STALL_T, 1.0f);

/**
 * [DEPRECATED] Old NED-Y stall-detection position gate
 *
 * The legacy stall detector was hard-coded to NED Y. The new detector uses
 * the setpoint-to-current-position direction and does not need an absolute
 * position gate. This parameter is kept only for parameter-format
 * compatibility and is no longer used.
 *
 * @unit m
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_GATE, 4.90f);

/**
 * Contact detection: position error threshold.
 *
 * Minimum setpoint-to-actual position difference along the forward direction
 * to consider the drone blocked by the pole. Used by the position/attitude
 * based contact detector.
 *
 * @unit m
 * @min 0.01
 * @max 0.50
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_SERR, 0.05f);

/**
 * Contact detection: forward velocity threshold.
 *
 * Maximum forward velocity magnitude along the setpoint direction to
 * consider the drone blocked by the pole. Used by the position/attitude
 * based contact detector.
 *
 * @unit m/s
 * @min 0.01
 * @max 0.50
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_SVEL, 0.20f);

/**
 * Contact detection: pitch threshold.
 *
 * Minimum forward pitch angle (negative in NED convention) required to
 * confirm contact. During manual perching tests the pitch reached about
 * -8 to -12 degrees when the airframe pressed against the pole.
 *
 * @unit deg
 * @min -30.0
 * @max 0.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_PIT_THR, -10.0f);

/**
 * Contact detection: confirmation duration.
 *
 * Time that all contact conditions (position error, low forward velocity,
 * pitch threshold) must be continuously satisfied before contact is declared.
 * A short duration filters out attitude transients from normal flight.
 *
 * @unit s
 * @min 0.05
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_DUR_THR, 0.30f);

/**
 * Perching admittance: vehicle mass used for force estimation.
 *
 * Used by scheme A (IMU acceleration residual) to convert horizontal
 * acceleration into an estimated contact force. Set to the actual takeoff
 * mass (SDF/sim or real vehicle weight).
 *
 * @unit kg
 * @min 0.5
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_MASS, 1.50f);

/**
 * Perching admittance: scheme A gain.
 *
 * Position correction per unit force error (m/N). When zero, scheme A is
 * disabled and the controller falls back to the fixed preload baseline.
 * Positive value: increase preload when estimated force is below the
 * desired value, decrease preload when above.
 *
 * @unit m
 * @min 0.0
 * @max 0.10
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_KA, 0.0f);

/**
 * Perching admittance: scheme A desired contact force.
 *
 * Target horizontal contact force used in the admittance law.
 *
 * @unit N
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_FD, 1.0f);

/**
 * Perching admittance: maximum position correction.
 *
 * Hard limit applied to the total additional position offset delta_p.
 * The final preload is clamped to [0, 0.10] m regardless of this limit.
 *
 * @unit m
 * @min 0.0
 * @max 0.10
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_LIM, 0.03f);

/**
 * Perching compliance: scheme B position-error gain.
 *
 * Position correction per meter of forward position error. Positive value
 * reduces preload when the pole pushes the drone backward (positive error
 * along approach direction).
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_KP, 0.0f);

/**
 * Perching compliance: scheme B velocity-damping gain.
 *
 * Damping term that reduces preload when the drone moves forward into the
 * pole along the approach direction. Helps absorb impact energy.
 *
 * @unit s
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_KV, 0.0f);

/**
 * Perching compliance: scheme B thrust-proxy gain.
 *
 * Position correction per unit normalized collective thrust. Higher motor
 * output (larger upward thrust needed to hold position) is interpreted as
 * larger contact force, reducing preload.
 *
 * @unit m
 * @min 0.0
 * @max 0.10
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_KT, 0.0f);

/**
 * Perching adaptive: scheme C gain.
 *
 * Overall gain for the pitch+position-error adaptive preload. When zero,
 * scheme C is disabled.
 *
 * @unit m
 * @min 0.0
 * @max 0.10
 * @decimal 3
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_KC, 0.0f);

/**
 * Perching adaptive: scheme C pitch weight.
 *
 * Weight of the normalized pitch term in the adaptive metric.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_W1, 1.0f);

/**
 * Perching adaptive: scheme C position-error weight.
 *
 * Weight of the normalized position-error term in the adaptive metric.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPCA_PC_ADM_W2, 1.0f);
