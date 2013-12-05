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
PARAM_DEFINE_FLOAT(PT_pan_KP, 1000.0f);
PARAM_DEFINE_FLOAT(PT_pan_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_pan_KD, 0.0f);
PARAM_DEFINE_FLOAT(PT_tilt_KP, 1000.0f);
PARAM_DEFINE_FLOAT(PT_tilt_KI, 0.0f);
PARAM_DEFINE_FLOAT(PT_tilt_KD, 0.0f);


int parameters_init(struct pantilt_param_handles *h)
{
	h->pan_KP = param_find("PT_pan_KP");
	h->pan_KI = param_find("PT_pan_KI");
	h->pan_KD 	=	param_find("PT_pan_KD");
	h->tilt_KP = param_find("PT_tilt_KP");
	h->tilt_KI = param_find("PT_tilt_KI");
	h->tilt_KD 	=	param_find("PT_tilt_KD");

	return OK;
}

int parameters_update(const struct pantilt_param_handles *h, struct pantilt_params *p)
{
	param_get(h->pan_KP, &(p->pan_KP));
	param_get(h->pan_KI, &(p->pan_KI));
	param_get(h->pan_KD, &(p->pan_KD));
	param_get(h->tilt_KP, &(p->tilt_KP));
	param_get(h->tilt_KI, &(p->tilt_KI));
	param_get(h->tilt_KD, &(p->tilt_KD));

	return OK;
}
