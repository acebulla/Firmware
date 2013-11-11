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
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_servo12c.h>
#include <drivers/drv_hrt.h>





/**
 * daemon management function.
 */
extern "C" { __EXPORT int servo12c_test_main(int argc, char *argv[]); }


class SERVO12C_TEST
{


public:
	~SERVO12C_TEST();

	void start(char manual);
	void stop();
	void set_values(float values[], char * inputType);

private:
	enum Input_type {
				ABS,
				DEG,
		        RAD
			};

	/**
	 * Mainloop of daemon.
	 */
	int servo12c_test_thread_main();

	/** work trampoline */
	static void		_test_trampoline(void *arg);


	struct hrt_call		_call;

	Input_type	_input_type;
	float _value[SERVOS_ATTACHED]; bool _new_val;
	bool _manual;
	bool _left;
	bool thread_should_run;
	int topic_handle;
	struct servo_control_values servcon;

};

SERVO12C_TEST::~SERVO12C_TEST()
{
	hrt_cancel(&_call);
	::close(topic_handle);
}

void
SERVO12C_TEST::start(char manual)
{
	thread_should_run = true;
	printf("char manual: %d", manual);
	_manual = (bool) manual;
	printf("bool manual: %d", _manual);
	uint8_t i;

	/* start calling the thread at the specified rate */
	hrt_call_after(&_call, 1000, (hrt_callout)&SERVO12C_TEST::_test_trampoline, this);

	/* generate the initial data for first publication */
	for (i = 0; i < SERVOS_ATTACHED; i++)
	{
		servcon.values[i] = 10.0f; servcon.set_value[i] = 1;
	}
	_left = false;

	/* advertise the topic and make the initial publication */
	topic_handle = orb_advertise(ORB_ID(servo12c_control), &servcon);

	warnx("[servo12c_test] starting\n");


}

void
SERVO12C_TEST::stop()
{
	warnx("[servo12c_test] exiting.\n");
	thread_should_run = false;
}

void
SERVO12C_TEST::_test_trampoline(void *arg)
{
	(reinterpret_cast<SERVO12C_TEST *>(arg))->servo12c_test_thread_main();
}

void
SERVO12C_TEST::set_values(float values[], char * inputType)
{
	uint8_t i;
	int fd;
	if (!strcmp(inputType, "ABS") && _input_type != ABS)
	{
		/* set the poll rate to default, starts automatic data collection */
		fd = open(SERVO12C_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			//printf("fd: %d", fd);
			goto fail;
		}

		if (ioctl(fd, SERVO_INPUT, SERVO_INPUT_ABS) < 0) {
			close(fd);
			goto fail;
		}

		close(fd);

		_input_type = ABS;
	} else if (!strcmp(inputType, "DEG") && _input_type != DEG)
	{
		/* set the poll rate to default, starts automatic data collection */
		fd = open(SERVO12C_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			//printf("fd: %d", fd);
			goto fail;
		}

		if (ioctl(fd, SERVO_INPUT, SERVO_INPUT_DEG) < 0) {
			close(fd);
			goto fail;
		}

		close(fd);

		_input_type = DEG;
	} else if  (!strcmp(inputType, "RAD") && _input_type != RAD)
	{
		/* set the poll rate to default, starts automatic data collection */
		fd = open(SERVO12C_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			//printf("fd: %d", fd);
			goto fail;
		}

		if (ioctl(fd, SERVO_INPUT, SERVO_INPUT_RAD) < 0) {
			close(fd);
			goto fail;
		}

		close(fd);

		_input_type = RAD;
	}

	for (i = 0; i < SERVOS_ATTACHED; i++)
	{
		_value[i] = values[i];
	}

	_new_val = true;
	_manual = true;

	return;

fail:

	errx(1, "Could not set input type");
}

int
SERVO12C_TEST::servo12c_test_thread_main() {
	uint8_t i;
	if (_manual)
	{
		if (_new_val)
		{
			for (i = 0; i < SERVOS_ATTACHED; i++)
			{
				servcon.values[i] = _value[i];
				servcon.set_value[i] = 1;
			}
			_new_val = false;
		}
	}
	else
	{
		for (i = 0; i < SERVOS_ATTACHED; i++)
		{
			servcon.values[i] = (_left) ? 245.0f : 5.0f;
		}
		_left = !_left;
	}

	orb_publish(ORB_ID(servo12c_control), topic_handle, &servcon);

	if (thread_should_run) {
		/* start calling the thread at the specified rate */
		hrt_call_after(&_call, 200000, (hrt_callout)&SERVO12C_TEST::_test_trampoline, this);
	}

	return 0;
}

/**
 * Local functions in support of the shell command.
 */
namespace servo12c_test
{

SERVO12C_TEST	*g_servo12c_test;

void	start(char manual);
void	stop();
void	set_values(float values[], char * inputType);
void	usage(const char *reason);


/**
 * Start the driver.
 */
void
start(char manual)
{
	if (g_servo12c_test != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_servo12c_test = new SERVO12C_TEST();

	if (g_servo12c_test == nullptr)
		goto fail;

	g_servo12c_test->start(manual);

	exit(0);
fail:

	if (g_servo12c_test != nullptr) {
		delete g_servo12c_test;
		g_servo12c_test = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop()
{
	g_servo12c_test->stop();
	delete g_servo12c_test;
	g_servo12c_test = nullptr;
	exit(0);
}

void
set_values(float values[], char * inputType)
{
	g_servo12c_test->set_values(values, inputType);

	exit(0);
}

void
usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

} // namespace

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int
servo12c_test_main(int argc, char *argv[])
{
	if (argc < 1)
		servo12c_test::usage("missing command");

	if (!strcmp(argv[1], "start")) {

		servo12c_test::start(0);
	}

	if (!strcmp(argv[1], "startmanual")) {
		servo12c_test::start(1);
	}

	if (!strcmp(argv[1], "ABS") || !strcmp(argv[1], "DEG") || !strcmp(argv[1], "RAD"))
	{
		uint8_t i;
		float values[SERVOS_ATTACHED];

		for (i = 2; i < SERVOS_ATTACHED + 2; i++)
		{
			values[i-2] = (float) atof(argv[i]);
		}

		servo12c_test::set_values(values, argv[1]);
	}

	if (!strcmp(argv[1], "stop")) {
		servo12c_test::stop();
	}


	errx(1, "Unrecognized command, try 'start', 'test', 'reset' or 'info'");

}


