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
 * @file Rangefinder driver interface for multiple sensors.
 */

#ifndef _DRV_RANGEFINDER_MULTSENS_H
#define _DRV_RANGEFINDER_MULTSENS_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define RANGE_FINDER_DEVICE_PATH	"/dev/range_finder"

#define MAX_SENSOR_COUNT 6	/** Maximum number of sensor */
#define GROUPEND 0x80	/** Bitmask indicating the end of an address group */
#define ADDRPART 0x7F	/** Bitmask for extracting the address part */

/**
 * range finder report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct range_finder_multsens_report {
	uint64_t timestamp;
	float distance [MAX_SENSOR_COUNT];	/** in meters */
	uint8_t valid [MAX_SENSOR_COUNT];	/** 1 == within sensor range, 0 = outside sensor range */
	uint8_t sensor_start, sensor_end;	/** Index of the first and the last sensor in the group */
};

/*
 * ObjDev tag for raw range finder data.
 */
ORB_DECLARE(multsens_range_finder);

/*
 * ioctl() definitions
 *
 * Rangefinder drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */

#define _RANGEFINDERIOCBASE			(0x7900)
#define __RANGEFINDERIOC(_n)		(_IOC(_RANGEFINDERIOCBASE, _n))

/** set the minimum effective distance of the device */
#define RANGEFINDERIOCSETMINIUMDISTANCE	__RANGEFINDERIOC(1)

/** set the maximum effective distance of the device */
#define RANGEFINDERIOCSETMAXIUMDISTANCE	__RANGEFINDERIOC(2)


#endif /* _DRV_RANGEFINDER_MULTSENS_H */
