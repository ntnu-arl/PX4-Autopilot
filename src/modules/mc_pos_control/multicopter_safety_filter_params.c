/**
 * CBF Epsilon
 *
 * ...
 *
 * @min 0.0
 * @max 10000
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_EPSILON, 0.5f);

/**
 * CBF Pole 0
 *
 * ...
 *
 * @min -1000
 * @max 0.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_POLE0, -2.f);

/**
 * CBF Kappa
 *
 * ...
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_KAPPA, 10.f);

/**
 * CBF Gamma
 *
 * ...
 *
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_GAMMA, 40.f);

/**
 * CBF Alpha
 *
 * ...
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_ALPHA, 1.f);

/**
 * CBF Alpha for FoV constraints
 *
 * ...
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CBF_FOV_ALPHA, 1.f);

/**
 * FoV constraints slack gain
 *
 * ...
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_FOV_SLACK, 50.f);

/**
 * CBF input low pass filter gain
 *
 * ...
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.001
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_LP_GAIN_IN, 0.5f);

/**
 * CBF ouput low pass filter gain
 *
 * ...
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.001
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_LP_GAIN_OUT, 0.5f);

/**
 * Gain on the x axis error in the CBF-QP cost function
 *
 * ...
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_QP_GAIN_X, 1.f);

/**
 * Gain on the y axis error in the CBF-QP cost function
 *
 * ...
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_QP_GAIN_Y, 1.f);

/**
 * Gain on the z axis error in the CBF-QP cost function
 *
 * ...
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_QP_GAIN_Z, 1.f);


/**
 * CBF output clamping on xy axis
*
 * ...
*
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_CLAMP_XY, 3.f);

/**
 * CBF output clamping on z axis
 *
 * ...
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(CBF_CLAMP_Z, 3.f);

/**
 * Enable CBF safety filter
 *
 * ...
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CBF_ENABLED, 0);
