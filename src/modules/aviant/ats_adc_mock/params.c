/**
 * Simulated Main Power 1 voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_MP1_SM, 50.0f);

/**
 * Simulated Main Power 2 voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_MP2_SM, 50.0f);

/**
 * Simulated UPS voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_UPS_SM, 5.0f);

/**
 * Simulated UPS glitch length (SITL)
 *
 * When set > 0, the UPS channel reads 0 V for this many ADC samples (10 ms each),
 * then the parameter is reset to 0.
 *
 * @group Aviant
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(AV_ATS_UPS_GLT, 0);

/**
 * Simulated parachute capacitor voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_ATS_PARA_SM, 24.0f);
