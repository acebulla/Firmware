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
#include <uORB/topics/vehicle_attitude.h>
#include <systemlib/systemlib.h>
#include <systemlib/pid/pid.h>
#include <mavlink/mavlink_log.h>

#include <drivers/drv_servo12c.h>

#include "pantilt_params.h"
#include "pos_pid.h"

#include "speed_pid.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

static float start_pos = 1.5808f;
static float error_band = 0.035f; /** 2° */
static float max_speed; //513f; /** RAD in 30 ms = 2.6 DEG*/

__EXPORT int pantilt_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int pantilt_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);



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
					     SCHED_PRIORITY_MAX - 50,
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



static int pantilt_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	warnx("started");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[pantilt] started");

	/* set the position input to RAD */
	int fd = open(SERVO12C_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		//printf("fd: %d", fd);
		errx(1, "Could not set input type");;
	}

	if (ioctl(fd, SERVO_INPUT, SERVO_INPUT_RAD) < 0) {
		close(fd);
		errx(1, "Could not set input type");;
	}

	close(fd);

	/* structures */
	struct marker_location_s marker_loc;
	memset(&marker_loc, 0, sizeof(marker_loc));
	struct servo_control_values servo_control;
	struct vehicle_attitude_s attitude_s;

	uint8_t i;

	for (i = 0; i < SERVOS_ATTACHED; i++) {
		servo_control.set_value[i] = 1;
		servo_control.speed[i] = 7.8f;
		servo_control.values[i] = start_pos;
	}

	/* subscribe to marker location, parameter update and attitude */
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int marker_location_sub = orb_subscribe(ORB_ID(marker_location));

	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* publish servo positions */
	orb_advert_t servo_control_pub = orb_advertise(ORB_ID(servo12c_control), &servo_control);


	thread_running = true;

	struct pantilt_params params;
	struct pantilt_param_handles params_h;
	parameters_init(&params_h);
	parameters_update(&params_h, &params);


	servo_position_f current_pos[2] = {start_pos, start_pos};
	servo_position_f target[2] = {start_pos, start_pos};
	servo_position_f tmp = 0.0f;


	bool target_reached[2] = {true, true};

	float error =  0.0f;

	max_speed = params.pan_pos_KD;

	while (!thread_should_exit) {

		bool param_updated;
		orb_check(param_sub, &param_updated);

		if (param_updated) {
			/* clear updated flag */
			struct parameter_update_s ps;
			orb_copy(ORB_ID(parameter_update), param_sub, &ps);
			/* update params */
			parameters_update(&params_h, &params);
			max_speed = params.pan_pos_KD;
		}


		bool new_marker_loc;
		orb_check(marker_location_sub, &new_marker_loc);

		if (new_marker_loc) {

			/* clear updated flag */
			orb_copy(ORB_ID(marker_location), marker_location_sub, &marker_loc);




			// Calculated current error value
			error =  0.0f - marker_loc.pan;

			if ( !((-error_band < error) && (error < error_band)) ) {
				/* calculate position */
				tmp = params.pan_pos_KP*error;

				/* Limit it to the maximum distance, which the servo can travle in 30 ms */
				if (((tmp < 0) ? -tmp : tmp) > max_speed) {
					// (target[0] > 0) - (target[0] < 0) gives the sign.
					current_pos[0] = current_pos[0] + (float)( (tmp > 0) - (tmp < 0) ) * max_speed;
				} else {
					current_pos[0] = current_pos[0] + tmp;
				}

				target_reached[0] = false;
			}  else {
				target_reached[0] = true;
			}


			// Calculated current error value
			error =  0.0f - marker_loc.tilt;

			if ( !((-error_band < error) && (error < error_band)) ) {
				/* calculate position */
				tmp = params.tilt_pos_KP*error;

				/* Limit it to the maximum distance, which the servo can travle in 30 ms */
				if (((tmp < 0) ? -tmp : tmp) > max_speed) {
					// (target[0] > 0) - (target[0] < 0) gives the sign.
					current_pos[1] = current_pos[1] + (float)( (tmp > 0) - (tmp < 0) ) * max_speed;
				} else {
					current_pos[1] = current_pos[1] + tmp;
				}

				target_reached[1] = false;
			} else {
				target_reached[1] = true;
			}

		} else {
			target_reached[0] = true;
			target_reached[1] = true;
		}


		/* Correct for change of quadrotor's attitude */
//		orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &attitude_s); // Assume we always get data
//
//		if( ((attitude_s.yawspeed < 0) ? -attitude_s.yawspeed : attitude_s.yawspeed) > 0.010f) {
//			current_pos[0] = current_pos[0] - (attitude_s.yawspeed / 300.0f);
//			target_reached[0] = false;
//		}
//
//		if ( ((attitude_s.pitchspeed < 0) ? -attitude_s.pitchspeed : attitude_s.pitchspeed) > 0.008f) {
//			current_pos[1] = current_pos[1] - (attitude_s.pitchspeed / 300.0f);
//			target_reached[1] = false;
//		}

		/* Check bounds */
		for (i = 0; i < SERVOS_ATTACHED; i++) {
			if (target_reached[i]) {
				servo_control.set_value[i] = 0;
				continue;
			}

			if (current_pos[i] > SERVO_MAX_RAD[i]) {
				current_pos[i] = SERVO_MAX_RAD[i];
			} else if (current_pos[i] < SERVO_MIN_RAD[i]) {
				current_pos[i] = SERVO_MIN_RAD[i];
			}

			servo_control.values[i] = current_pos[i];
			servo_control.set_value[i] = 1;
		}


		if (!(target_reached[0] && target_reached[1]) ) {
			orb_publish(ORB_ID(servo12c_control), servo_control_pub, &servo_control);
		}


		/* Synchronize using camera messages */
		usleep(1000);
		for (i = 0; i < SERVOS_ATTACHED; i++) {
			target_reached[i] = true;
			servo_control.set_value[i] = 0;
		}

	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[pantilt] stopped");

	thread_running = false;

	fflush(stdout);
	return 0;
}

