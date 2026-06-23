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
 * AS5600 encoder raw value at closed position
 *
 * 12-bit raw angle (0-4095) when the morphing arms are fully closed.
 * This corresponds to arm_angle = 0 rad.
 * Default value matches current huaqiccc hardware calibration; override in airframe if different.
 *
 * @min 0
 * @max 4095
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_EMIN, 518);

/**
 * AS5600 encoder raw value at open position
 *
 * 12-bit raw angle (0-4095) when the morphing arms are fully expanded.
 * This corresponds to arm_angle = -0.4 rad.
 * Default value matches current huaqiccc hardware calibration; override in airframe if different.
 *
 * @min 0
 * @max 4095
 * @group Morphing Arm
 */
PARAM_DEFINE_INT32(MORPH_EMAX, 780);

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

