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

/*
 * @file multirotor_pos_control_params.c
 *
 * Parameters for multirotor_pos_control
 */

#include "pantilt_params.h"

/* controller parameters */
PARAM_DEFINE_FLOAT(PT_p_pos_KP, 300.0f);
PARAM_DEFINE_FLOAT(PT_p_pos_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_p_pos_KD, 0.0f);
PARAM_DEFINE_FLOAT(PT_t_pos_KP, 300.0f);
PARAM_DEFINE_FLOAT(PT_t_pos_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_t_pos_KD, 0.0f);

PARAM_DEFINE_FLOAT(PT_p_vel_KP, 3.5f);
PARAM_DEFINE_FLOAT(PT_p_vel_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_p_vel_KD, 0.0f);
PARAM_DEFINE_FLOAT(PT_t_vel_KP, 3.5f);
PARAM_DEFINE_FLOAT(PT_t_vel_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_t_vel_KD, 0.0f);


int parameters_init(struct pantilt_param_handles *h)
{
	h->pan_pos_KP = param_find("PT_p_pos_KP");
	h->pan_pos_KI = param_find("PT_p_pos_KI");
	h->pan_pos_KD 	=	param_find("PT_p_pos_KD");
	h->tilt_pos_KP = param_find("PT_t_pos_KP");
	h->tilt_pos_KI = param_find("PT_t_pos_KI");
	h->tilt_pos_KD 	=	param_find("PT_t_pos_KD");

	h->pan_vel_KP = param_find("PT_p_vel_KP");
	h->pan_vel_KI = param_find("PT_p_vel_KI");
	h->pan_vel_KD 	=	param_find("PT_p_vel_KD");
	h->tilt_vel_KP = param_find("PT_t_vel_KP");
	h->tilt_vel_KI = param_find("PT_t_vel_KI");
	h->tilt_vel_KD 	=	param_find("PT_t_vel_KD");

	return OK;
}

int parameters_update(const struct pantilt_param_handles *h, struct pantilt_params *p)
{
	param_get(h->pan_pos_KP, &(p->pan_pos_KP));
	param_get(h->pan_pos_KI, &(p->pan_pos_KI));
	param_get(h->pan_pos_KD, &(p->pan_pos_KD));
	param_get(h->tilt_pos_KP, &(p->tilt_pos_KP));
	param_get(h->tilt_pos_KI, &(p->tilt_pos_KI));
	param_get(h->tilt_pos_KD, &(p->tilt_pos_KD));

	param_get(h->pan_vel_KP, &(p->pan_vel_KP));
	param_get(h->pan_vel_KI, &(p->pan_vel_KI));
	param_get(h->pan_vel_KD, &(p->pan_vel_KD));
	param_get(h->tilt_vel_KP, &(p->tilt_vel_KP));
	param_get(h->tilt_vel_KI, &(p->tilt_vel_KI));
	param_get(h->tilt_vel_KD, &(p->tilt_vel_KD));

	return OK;
}
