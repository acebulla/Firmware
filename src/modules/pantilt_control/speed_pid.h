/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file speed_pid.h
 *
 * Definition of speed control PID interface.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef SPEED_PID_H_
#define SPEED_PID_H_

#include <stdint.h>

__BEGIN_DECLS

/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error */
#define SPEED_PID_MODE_DERIVATIV_CALC	0
/* PID_MODE_DERIVATIV_CALC_NO_SP calculates discrete derivative from previous value, setpoint derivative is ignored */
#define SPEED_PID_MODE_DERIVATIV_CALC_NO_SP	1

typedef struct {
	float kp;
	float ki;
	float kd;
	float sp;
	float integral;
	float error_previous;
	float last_output;
	float limit_min;
	float limit_max;
	float dt_min;
	uint8_t mode;
} speed_pid_t;

__EXPORT void speed_pid_init(speed_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max, uint8_t mode, float dt_min);
__EXPORT int speed_pid_set_parameters(speed_pid_t *pid, float kp, float ki, float kd, float limit_min, float limit_max);
__EXPORT float speed_pid_calculate(speed_pid_t *pid, float sp, float val, float dt);
__EXPORT void speed_pid_set_integral(speed_pid_t *pid, float i);

__END_DECLS

#endif /* SPEED_PID_H_ */
