/**
 * ATS Timeout threshold
 *
 * Time threshold for considering the flight controller as not sending data.
 *
 * @group Aviant
 * @unit ms
 * @decimal 1
 * @min 0
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_ATS_TIMEOUT, 150);

/**
 * ATS Acceleration norm threshold
 *
 * Acceleration norm threshold for determining when to deploy the parachute
 *
 * @group Aviant
 * @unit m/s^2
 * @decimal 1
 * @min 0
 * @max 1000
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_ACC_NORM, 5);

/**
 * ATS Roll angle threshold
 *
 * If the roll angle exceeds this threshold, this criterion is considered met.
 *
 * @group Aviant
 * @unit deg
 * @decimal 1
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_ROLL_ANG, 80);

/**
 * ATS Pitch angle threshold
 *
 * If the pitch angle exceeds this threshold, this criterion is considered met.
 *
 * @group Aviant
 * @unit deg
 * @decimal 1
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(AV_ATS_PITCH_ANG, 60);

/**
 * ATS active
 *
 * When the ATS is active, it will check the trigger conditions and
 * command flight termination / parachute deployment
 *
 *
 * @group Aviant
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_ATS_ACTIVE, 0);

/**
 * ATS Main Power low voltage threshold
 *
 * Both main power rails must drop below this threshold
 * for a low-voltage condition to be detected.
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(AV_ATS_MP_LOWV, 20.0f);

/**
 * ATS UPS low voltage threshold
 *
 * If the UPS voltage is at or below this threshold, the
 * voltage measurement is considered unreliable and will
 * not trigger a parachute deploy.
 *
 * @group Aviant
 * @unit V
 * @decimal 1
 * @min 0
 * @max 100
 */
PARAM_DEFINE_FLOAT(AV_ATS_UPS_LOWV, 4.0f);

/**
 * ATS voltage-based deployment enable
 *
 * When enabled, the ATS will deploy the parachute if both main
 * power rails drop below AV_ATS_MP_LOWV while the UPS voltage
 * remains above AV_ATS_UPS_LOWV.
 *
 * @group Aviant
 * @boolean
 */
PARAM_DEFINE_INT32(AV_ATS_V_EN, 0);

/**
 * ATS deployment hysteresis
 *
 * Time that the deployment condition must be continuously true
 * before the parachute is deployed. Does not apply to voltage-based
 * deployment which is immediate.
 *
 * @group Aviant
 * @unit s
 * @decimal 2
 * @min 0
 * @max 5
 */
PARAM_DEFINE_FLOAT(AV_ATS_TTRI, 0.15f);
