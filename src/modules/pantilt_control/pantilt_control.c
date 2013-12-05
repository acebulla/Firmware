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
 * @file pantilt_control.c
 *
 * Multirotor position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/marker_location.h>
#include <systemlib/systemlib.h>
#include <systemlib/pid/pid.h>
#include <mavlink/mavlink_log.h>

#include <drivers/drv_servo12c.h>

#include "pantilt_params.h"
#include "speed_pid.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int pantilt_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int pantilt_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static float scale_control(float ctl, float end, float dz);

static float norm(float x, float y);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: pantilt_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int pantilt_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		warnx("start");
		thread_should_exit = false;
		deamon_task = task_spawn_cmd("pantilt_control",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_MAX - 60,
					     4096,
					     pantilt_control_thread_main,
					     (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		warnx("stop");
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("app is running");

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static float scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

static float norm(float x, float y)
{
	return sqrtf(x * x + y * y);
}

static int pantilt_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	warnx("started");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[pantilt] started");

	/* structures */
	struct marker_location_s marker_loc;
	memset(&marker_loc, 0, sizeof(marker_loc));
	struct servo_control_values servo_control;

	servo_control.set_value[0] = 1;
	servo_control.speed[0] = 0.0f;
	servo_control.values[0] = 127.0f;
	servo_control.set_value[1] = 1;
	servo_control.speed[1] = 0.0f;
	servo_control.values[1] = 127.0f;

	/* subscribe to marker location and system state */
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int marker_location_sub = orb_subscribe(ORB_ID(marker_location));
	int servo_control_sub = orb_subscribe(ORB_ID(servo12c_control));

	/* publish setpoint XXX */
//	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	orb_advert_t servo_control_pub = orb_advertise(ORB_ID(servo12c_control), &servo_control_sub);

	hrt_abstime t_prev = 0;
	const float alt_ctl_dz = 0.2f;
	const float pos_ctl_dz = 0.05f;

	float ref_alt = 0.0f;
	hrt_abstime ref_alt_t = 0;
	uint64_t local_ref_timestamp = 0;

	speed_pid_t pan_vel_pid;
	speed_pid_t tilt_vel_pid;

	thread_running = true;

	struct pantilt_params params;
	struct pantilt_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);



	speed_pid_init(&pan_vel_pid, params.pan_KP, params.pan_KI, params.pan_KD, -2.0f, 2.0f, SPEED_PID_MODE_DERIVATIV_CALC, 0.02f);
	speed_pid_init(&tilt_vel_pid, params.tilt_KP, params.tilt_KI, params.tilt_KD, -2.0f, 2.0f, SPEED_PID_MODE_DERIVATIV_CALC, 0.02f);


	while (!thread_should_exit) {

		bool param_updated;
		orb_check(param_sub, &param_updated);

		if (param_updated) {
			/* clear updated flag */
			struct parameter_update_s ps;
			orb_copy(ORB_ID(parameter_update), param_sub, &ps);
			/* update params */
			parameters_update(&params_h, &params);


			speed_pid_set_parameters(&pan_vel_pid,  params.pan_KP, params.pan_KI, params.pan_KD, -2.0f, 2.0f);
			speed_pid_set_parameters(&tilt_vel_pid,  params.tilt_KP, params.tilt_KI, params.tilt_KD, -2.0f, 2.0f);
		}

		bool new_marker_loc;
		orb_check(marker_location_sub, &new_marker_loc);

		hrt_abstime t = hrt_absolute_time();
		float dt;

		if (t_prev != 0) {
			dt = (t - t_prev) * 0.000001f;

		} else {
			dt = 0.0f;
		}

		if (new_marker_loc) {

			float pan_s, tilt_s;
			/* clear updated flag */
			orb_copy(ORB_ID(marker_location), marker_location_sub, &marker_loc);

			/* calculate speed */
			pan_s = speed_pid_calculate(&pan_vel_pid, 0.0, marker_loc.pan, dt);
			tilt_s = speed_pid_calculate(&tilt_vel_pid, 0.0, marker_loc.tilt, dt);

			if (pan_s < 0) {
				servo_control.set_value[0] = 1;
				servo_control.speed[0] = -pan_s;
				servo_control.values[0] = 0.0f;
			} else {
				servo_control.set_value[0] = 1;
				servo_control.speed[0] = pan_s;
				servo_control.values[0] = 255.0f;
			}

			if (tilt_s < 0) {
				servo_control.set_value[1] = 1;
				servo_control.speed[1] = -tilt_s;
				servo_control.values[1] = 0.0f;
			} else {
				servo_control.set_value[1] = 1;
				servo_control.speed[1] = tilt_s;
				servo_control.values[1] = 255.0f;
			}


			orb_publish(ORB_ID(servo12c_control), servo_control_pub, &servo_control);


		}


		t_prev = t;


		/* run at approximately 30 Hz */
		usleep(34000);
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[pantilt] stopped");

	thread_running = false;

	fflush(stdout);
	return 0;
}

