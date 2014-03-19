/*
 * servo12c_params.c
 *
 *  Created on: Dec 1, 2013
 *      Author: acebulla
 *
 */

#include <drivers/drv_servo12c.h>

/* controller parameters */
PARAM_DEFINE_FLOAT(SERVO_PAN_SLOPE, -61.0f);
PARAM_DEFINE_FLOAT(SERVO_TILT_SLOPE, -61.0f);

PARAM_DEFINE_FLOAT(SERVO_PAN_YINT, 124.0f);
PARAM_DEFINE_FLOAT(SERVO_TILT_YINT, 124.0f);
