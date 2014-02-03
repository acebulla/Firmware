/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file Wii IR sensor driver interface.
 */

#ifndef _DRV_WIIIRSENSOR_H
#define _DRV_WIIIRSENSOR_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_orb_dev.h"

#define WII_IR_SENSOR_DEVICE_PATH	"/dev/wiiIRsensor"

/**
 * The IR sensor can detect up to 4 IR sources. Their position is
 * reported with this structure.
 */

struct wii_IR_report {
	uint64_t timestamp;
	int Ix1, Iy1, Ix2, Iy2;		/** in pixels, if no source was detected the value will be 1023 */
	int Ix3, Iy3, Ix4, Iy4;
};

/*
 * ObjDev tag for Wii IR sensor data.
 */
ORB_DECLARE(wii_IR_sensor);

/*
 * ioctl() definitions
 *
 */

#define _WIIIRSENSORIOCBASE			(0x5300)
#define __WIIIRSENSORIOC(_n)		(_IOC(_WIIIRSENSORIOCBASE, _n))

/** set the maximum blob size */
#define WIIIRSENSORIOCMAXSIZE	__WIIIRSENSORIOC(1)

/** set the gain (smaller number = higher gain) */
#define WIIIRSENSORIOCGAIN	__WIIIRSENSORIOC(2)

/** set the gain limit (must be smaller than the gain) */
#define WIIIRSENSORIOCGAINLIMIT	__WIIIRSENSORIOC(3)

/** set the minimum blob size */
#define WIIIRSENSORIOCMINSIZE	__WIIIRSENSORIOC(4)

#endif /* _DRV_WIIIRSENSOR_H */
