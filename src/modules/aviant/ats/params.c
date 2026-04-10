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
 * ATS enabled
 *
 * When the ATS is enabled, it will check the trigger conditions and
 * command flight termination / parachute deployment
 *
 *
 * @group Aviant
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(AV_ATS_EN, 0);

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
PARAM_DEFINE_FLOAT(AV_ATS_MP_LOWV, 30.0f);

/**
 * ATS UPS low voltage threshold
 *
 * Below this threshold the UPS is reported unhealthy (status flag only;
 * does not block voltage-based deploy).
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
 * When enabled, deploy when both main rails are below AV_ATS_MP_LOWV
 * (armed or rebooted-while-armed)
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

/**
 * Main Power 1 ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_ATS_MP1_CH, -1);

/**
 * Main Power 1 voltage divider
 *
 * Multiplier applied to ADC voltage to get actual voltage.
 *
 * @group Aviant
 * @decimal 3
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_MP1_DV, 1.0f);

/**
 * Main Power 2 ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_ATS_MP2_CH, -1);

/**
 * Main Power 2 voltage divider
 *
 * Multiplier applied to ADC voltage to get actual voltage.
 *
 * @group Aviant
 * @decimal 3
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_MP2_DV, 1.0f);

/**
 * UPS ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_ATS_UPS_CH, -1);

/**
 * UPS voltage divider
 *
 * Multiplier applied to ADC voltage to get actual voltage.
 *
 * @group Aviant
 * @decimal 3
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_UPS_DV, 1.0f);

/**
 * Parachute supply ADC channel
 *
 * Voltage sense for the parachute (release) supply rail.
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_ATS_PARA_CH, -1);

/**
 * Parachute supply voltage divider
 *
 * Multiplier applied to ADC voltage to get actual voltage.
 *
 * @group Aviant
 * @decimal 3
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_PARA_DV, 1.0f);

/**
 * ATS FC BATTERY_STATUS vs main ADC tolerance
 *
 * When > 0, ATS compares FC BATTERY_STATUS voltage to each main ADC rail,
 * and the ADC rails to each other.
 * Absolute difference above this sets UPS_STATUS_(FC/MP)_MISMATCH.
 * When 0, comparison is off.
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_BAT_V_TOL, 0.0f);

/**
 * ATS FC battery_status max age
 *
 * FC battery_status is considered invalid if older than this
 * Ignored when set to 0.
 *
 * @group Aviant
 * @unit ms
 * @min 0
 * @max 60000
 */
PARAM_DEFINE_INT32(AV_ATS_BAT_TOUT, 1000);
