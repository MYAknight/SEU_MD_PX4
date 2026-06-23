/**
 * @file external_force_estimator_params.c
 *
 * Parameters for the external force estimator (IMU-ICD contact detection).
 *
 * NOTE: This IMU-only approach has been deprecated on the real huaqiccc
 * morphing quadrotor because raw IMU acceleration showed no useful change
 * during pole contact. The module is kept for reference but is no longer
 * started by default. Active contact detection is in mc_pos_control.
 */

/**
 * IMU-ICD module enable switch
 *
 * When disabled, the module still runs but skips all contact detection logic
 * and publishes STATE_NO_CONTACT continuously.
 *
 * NOTE: This IMU-only detector is deprecated on the real vehicle. It is
 * kept for reference but is not started by default. Leave enabled only for
 * logging/diagnostic purposes when the module is manually started.
 *
 * @boolean
 * @group External Force Estimator
 */
PARAM_DEFINE_INT32(EFO_ENABLE, 1);

/**
 * Vehicle mass for GMO external force estimation
 *
 * Used to convert acceleration residual to force estimate.
 *
 * @unit kg
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_MASS, 1.173);

/**
 * IMU-ICD impact metric threshold for contact detection
 *
 * Impact metric = |a_hpf| + 2*stddev(|a_hpf|). Above this threshold,
 * an impact is declared. Must be high enough to avoid false positives
 * from normal flight vibrations, but low enough to catch collisions.
 *
 * @unit m/s^2
 * @min 0.5
 * @max 20.0
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_FTHR, 12.0);

/**
 * Time threshold for contact confirmation
 *
 * Minimum duration above force threshold to confirm contact.
 *
 * @unit s
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_TTHR, 0.10);

/**
 * Gyroscope threshold for stable perching detection
 *
 * Maximum angular velocity magnitude for perching to be considered stable.
 *
 * @unit rad/s
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_GTHR, 0.15);

/**
 * Long window duration for GMO baseline
 *
 * Duration of the long window for slow baseline estimation.
 *
 * @unit s
 * @min 0.5
 * @max 10.0
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_LWIN, 2.0);

/**
 * Short window duration for GMO residual
 *
 * Duration of the short window for instantaneous state estimation.
 *
 * @unit s
 * @min 0.02
 * @max 1.0
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_SWIN, 0.1);

/**
 * Low-pass filter alpha for force estimate
 *
 * Higher values give more smoothing but slower response.
 *
 * @min 0.0
 * @max 0.99
 * @decimal 2
 * @group External Force Estimator
 */
PARAM_DEFINE_FLOAT(EFO_LPF, 0.8);
