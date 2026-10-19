/**
 * Maximum normalized T500 bidirectional-thrust setpoint.
 *
 * Center stick commands zero, upper stick commands forward thrust, and lower
 * stick commands reverse thrust.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_THR_MAX, 0.35f);

/**
 * Maximum normalized yaw-torque setpoint.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_YAW_MAX, 1.00f);

/**
 * Additional quad-motor collective idle thrust in Water mode.
 *
 * The ESC output minimum keeps all four motors spinning. This additional
 * collective provides control-allocation headroom for differential yaw.
 *
 * @unit norm
 * @min 0.0
 * @max 0.25
 * @decimal 2
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_Q_IDLE, 0.22f);

/**
 * Maximum manual-input age accepted by Water mode.
 *
 * Stale input stops the T500 and yaw command while preserving quad idle.
 *
 * @unit s
 * @min 0.1
 * @max 2.0
 * @decimal 1
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_IN_T, 0.5f);

/**
 * Maximum T500 setpoint change per second in Water mode.
 *
 * @unit normalized_thrust/s
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_THR_SLEW, 0.20f);

/**
 * Maximum yaw-torque setpoint change per second in Water mode.
 *
 * @unit 1/s
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group HAUV Water Control
 */
PARAM_DEFINE_FLOAT(HAUV_W_YAW_SLEW, 0.50f);
