/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot.
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_servo12c.h>


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;

/* create topic metadate */
ORB_DEFINE(servo12c_control, struct servo_control_values);


/**
 * daemon management function.
 */
__EXPORT int servo12c_test_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int servo12c_test_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int servo12c_test_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("servo12c_test",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2048,
					 servo12c_test_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int servo12c_test_thread_main(int argc, char *argv[]) {

	bool left;
	int topic_handle;
	short i = 0;

	warnx("[servo12c_test] starting\n");

	thread_running = true;

	/* generate the initial data for first publication */
	struct servo_control_values servcon;
	servcon.values[0] = 10.0f; servcon.set_value[0] = 1;
	left = false;

	/* advertise the topic and make the initial publication */
	topic_handle = orb_advertise(ORB_ID(servo12c_control), &servcon);

	while (!thread_should_exit) {

		servcon.values[0] = (left) ? 245.0f : 10.0f;
		left = !left;

		orb_publish(ORB_ID(servo12c_control), topic_handle, &servcon);
		usleep(500000);

	}

	warnx("[servo12c_test] exiting.\n");

	thread_running = false;

	return 0;
}
