/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PWM servo output interface.
 *
 * Servo values can be set with the PWM_SERVO_SET ioctl, by writing a
 * pwm_output_values structure to the device, or by publishing to the
 * output_pwm ORB topic.
 * Writing a value of 0 to a channel suppresses any output for that
 * channel.
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>
#include <math.h>

#include "drv_orb_dev.h"

__BEGIN_DECLS

#define PI 3.14159F
/**
 * Path for the Servo12c chip.
 *
 */
#define SERVO12C_DEVICE_PATH	"/dev/servo12c"

/**
 * Maximum number of servo output channels supported by the device.
 */
#define MAXIMUM_NUMBER_SERVOS	12

/**
 * Number of servos attached.
 */
#define SERVOS_ATTACHED	2

/**
 * Servo output signal type.
 * 	Value may be:
 * 		-) absolute: 0 .. 255
 * 		-) degree: 0° .. 180°
 * 		-) radian: 0*pi .. 2*pi
 */
typedef float	servo_position_f;

/** Measured characteristics of the individual servo */
const servo_position_f SERVO_MAX_ABS[SERVOS_ATTACHED] = {255, 255};
const servo_position_f SERVO_MAX_DEG[SERVOS_ATTACHED] = {180, 180};
const servo_position_f SERVO_MAX_RAD[SERVOS_ATTACHED] = {PI, PI};

const servo_position_f SERVO_MIN_ABS[SERVOS_ATTACHED] = {0, 0};
const servo_position_f SERVO_MIN_DEG[SERVOS_ATTACHED] = {0, 0};
const servo_position_f SERVO_MIN_RAD[SERVOS_ATTACHED] = {0, 0};

/**
 * Servo output structure.
 * To save memory the length of the array is defined through the
 * actual number of servos attached SERVOS_ATTACHED.
 * The first servo must be attached to port SERVO0, the second to
 * port SERVO1 and so on.
 *
 */
struct servo_control_values {
	/** desired pulse widths for each of the supported channels */
	servo_position_f	values[SERVOS_ATTACHED];
	uint8_t				set_value[SERVOS_ATTACHED];
};

/*
 * ORB tag for servo control.
 */
ORB_DECLARE(servo12c_control);


/*
 * ioctl() definitions
 *
 * Note that ioctls and ORB updates should not be mixed, as the
 * behaviour of the system in this case is not defined.
 */
#define _SERVO_BASE		0xac00

/** set the input type: */
#define SERVO_INPUT		_IOC(_SERVO_BASE, 0)

	/** input type: absolute 0 .. 255 */
	#define SERVO_INPUT_ABS		_IOC(_SERVO_BASE, 1)

	/** input type: degree 0° .. 180° */
	#define SERVO_INPUT_DEG		_IOC(_SERVO_BASE, 2)

	/** input type: radian 0*pi .. 2*pi */
	#define SERVO_INPUT_RAD 	_IOC(_SERVO_BASE, 3)



__END_DECLS
