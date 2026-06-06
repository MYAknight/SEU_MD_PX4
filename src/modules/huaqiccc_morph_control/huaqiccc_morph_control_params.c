/**
 * huaqiccc Morphing Arm Control Parameters
 *
 * Controls a single linear actuator (PWM servo on AUX1) with AS5600 magnetic
 * encoder feedback (I2C). Simple P-control with deadband is sufficient due
 * to slow morphing dynamics.
 */

/**
 * Morph control enable
 *
 * Set to 1 to enable the huaqiccc morphing arm control module.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_EN, 0);

/**
 * Morph control proportional gain
 *
 * P gain in the range of actuator_servos output (-1 to +1) per normalized
 * position error. A value of 1.0 means full output at 100% error.
 *
 * @unit norm
 * @min 0.0
 * @max 10.0
 * @group Morphing Arm
 */
PARAM_DEFINE_FLOAT(MORPH_KP, 2.0f);

/**
 * Morph control deadband
 *
 * Normalized position deadband. When |target - current| < deadband,
 * actuator output is set to neutral (0.0).
 *
 * @unit norm
 * @min 0.0
 * @max 0.5
 * @group Morphing Arm
 */
PARAM_DEFINE_FLOAT(MORPH_DB, 0.02f);

/**
 * AS5600 encoder raw value at closed position
 *
 * 12-bit raw angle (0-4095) when the morphing arms are fully closed.
 * This corresponds to arm_angle = 0 rad.
 *
 * @min 0
 * @max 4095
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_EMIN, 0);

/**
 * AS5600 encoder raw value at open position
 *
 * 12-bit raw angle (0-4095) when the morphing arms are fully expanded.
 * This corresponds to arm_angle = -0.5 rad.
 *
 * @min 0
 * @max 4095
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_EMAX, 4095);

/**
 * Morph control loop rate
 *
 * Update rate of the control loop in Hz.
 *
 * @unit Hz
 * @min 10
 * @max 200
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_RATE, 50);

/**
 * I2C bus for AS5600 encoder
 *
 * I2C bus number where the AS5600 magnetic encoder is connected.
 * On Pixhawk 4: 2 = I2C A (external), 4 = I2C B (external).
 *
 * @min 0
 * @max 4
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_BUS, 2);
