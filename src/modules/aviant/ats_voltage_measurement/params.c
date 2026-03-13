/**
 * Main Power 1 ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_V_MP1_CH, -1);

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
PARAM_DEFINE_FLOAT(AV_V_MP1_DIV, 1.0f);

/**
 * Main Power 2 ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_V_MP2_CH, -1);

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
PARAM_DEFINE_FLOAT(AV_V_MP2_DIV, 1.0f);

/**
 * UPS ADC channel
 *
 * @group Aviant
 * @min -1
 * @max 15
 */
PARAM_DEFINE_INT32(AV_V_UPS_CH, -1);

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
PARAM_DEFINE_FLOAT(AV_V_UPS_DIV, 1.0f);

/**
 * Simulated Main Power 1 voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_V_MP1_SIM, 25.2f);

/**
 * Simulated Main Power 2 voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_V_MP2_SIM, 25.2f);

/**
 * Simulated UPS voltage (SITL)
 *
 * @group Aviant
 * @unit V
 * @decimal 2
 * @min 0.0
 * @max 100.0
 */
PARAM_DEFINE_FLOAT(AV_V_UPS_SIM, 12.6f);
