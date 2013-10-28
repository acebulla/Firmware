/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file servo12c.cpp
 * @author Alexander Cebulla
 *
 * Driver for the SERVO12C: A servo control chip with an I2C interface
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_servo12c.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define SERVO12C_BUS 			PX4_I2C_BUS_EXPANSION
#define SERVO12C_BASEADDR 40 /* 7-bit address. 40 decimal */

/* SERVO12C I2C commands */

#define SERVO12C_MODE_REG		0x14		/* Mode Register */
#define SERVO12C_STANDARD_MODE	0x00
#define SERVO12C_EXTENDED_MODE	0x01

/* XXX Should we move servo limits from the drv_servo12c header */



/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


class SERVO12C : public device::I2C
{
public:
	SERVO12C(int bus=SERVO12C_BUS, uint8_t address=SERVO12C_BASEADDR);
	virtual ~SERVO12C();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:
	enum Input_type {
				ABS,
				DEG,
		        RAD
			};

	Input_type			_input_type;
	int					_task;
	volatile bool		_task_exit;

	bool				_sensor_ok;

	uint8_t				_current_values[SERVOS_ATTACHED];
	float				_conversion_values[SERVOS_ATTACHED];

	int					_servo_control_topic;
	servo_control_values	_controls;




	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	// int					probe_address(uint8_t address);

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	//float				get_minimum_distance();
	//float				get_maximum_distance();


	int					set_servo_values();

	/**
	 * Convert degree or radian to absolute value.
	 * Cast absolute values from float to uint8_t.
	 * Enforces boundaries.
	 *
	 * @param conv		The to be converted value.
	 */

	uint8_t				convert(float conv, uint8_t servo);


	static void	task_cycle_trampoline(int argc, char *argv[]);
	void		task_cycle() __attribute__((noreturn));


};

namespace
{

SERVO12C	*g_servo12c;

}

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int servo12c_main(int argc, char *argv[]);

SERVO12C::SERVO12C(int bus, uint8_t address) :
	I2C("SERVO12C", SERVO12C_DEVICE_PATH, bus, address, 100000),
	_input_type(ABS),
	_task(-1),
	_task_exit(false),
	_sensor_ok(true),
	_servo_control_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "servo12c_write")),
	_comms_errors(perf_alloc(PC_COUNT, "servo12c_comms_errors"))
{

	// enable debug() calls
	_debug_enabled = true;



	memset(_current_values, 127, sizeof(_controls));
}

SERVO12C::~SERVO12C()
{
	if (_task != -1) {
			/* tell the task we want it to go away */
			_task_exit = true;

			unsigned i = 10;
			do {
				/* wait 50ms - it should wake every 100ms or so worst-case */
				usleep(50000);

				/* if we have given up, kill it */
				if (--i == 0) {
					task_delete(_task);
					break;
				}

			} while (_task != -1);
		}

		g_servo12c = nullptr;
}



int
SERVO12C::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		//printf("I2C init gone wrong");
		ret = -errno;
	}

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	/* start the HIL interface task */
	_task = task_spawn_cmd("servo12c",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2048,
			   (main_t)&SERVO12C::task_cycle_trampoline,
			   nullptr);

	//printf("In init(): _task = %d \n", _task);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		ret = -errno;
	}

	return ret;
}


int
SERVO12C::probe()
{

	int ret,i ;
	const uint8_t msg[2] = {(uint8_t) 0, (uint8_t) 10};



	//printf("sizeof(msg) = %d\n", sizeof(msg));

	ret = transfer(msg, sizeof(msg), nullptr, 0);



	return ret;
}


int
SERVO12C::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	uint8_t i;
	switch (cmd) {

		case SERVO_INPUT: {
				switch (arg) {

					/* switching to absolute input values */
				case SERVO_INPUT_ABS:
					_input_type = ABS;
					return OK;

					/* switching to degree input values */
				case SERVO_INPUT_DEG:
					_input_type = DEG;
					for (i = 0; i < SERVOS_ATTACHED; i++) {
						_conversion_values[i] = (SERVO_MAX_ABS[i] - SERVO_MIN_ABS[i]) / (SERVO_MAX_DEG[i] - SERVO_MIN_DEG[i]);
					}
					return OK;

					/* switching to radian input values */
				case SERVO_INPUT_RAD:
					_input_type = RAD;
					for (i = 0; i < SERVOS_ATTACHED; i++) {
						_conversion_values[i] = (SERVO_MAX_ABS[i] - SERVO_MIN_ABS[i]) / (SERVO_MAX_RAD[i] - SERVO_MIN_RAD[i]);
					}
					return OK;

					/* other input types are not supported */
				default:
					return -EINVAL;

				}
			break;
			}

		/* Other commands are not supported */
		default:
			return -EINVAL;
	}


}

ssize_t
SERVO12C::read(struct file *filp, char *buffer, size_t buflen)
{

//	unsigned count = buflen / sizeof(struct range_finder_multsens_report);
//	struct range_finder_multsens_report *rbuf = reinterpret_cast<struct range_finder_multsens_report *>(buffer);
//	int ret = 0;
//
//	/* buffer must be large enough */
//	if (count < 1)
//		return -ENOSPC;
//
//	/* if automatic measurement is enabled */
//	if (_measure_ticks > 0) {
//
//		/*
//		 * While there is space in the caller's buffer, and reports, copy them.
//		 * Note that we may be pre-empted by the workq thread while we are doing this;
//		 * we are careful to avoid racing with them.
//		 */
//		while (count--) {
//			if (_reports->get(rbuf)) {
//				ret += sizeof(*rbuf);
//				rbuf++;
//			}
//		}
//
//		/* if there was no data, warn the caller */
//		return ret ? ret : -EAGAIN;
//	}
//
//	/* manual measurement - run one conversion */
//	do {
//		_reports->flush();
//
//		/* trigger a measurement */
//		if (OK != measure()) {
//			ret = -EIO;
//			break;
//		}
//
//		/* wait for it to complete */
//		usleep(MB12XX_CONVERSION_INTERVAL);
//
//		/* run the collection phase */
//		if (OK != collect()) {
//			ret = -EIO;
//			break;
//		}
//
//		/* state machine will have generated a report, copy it out */
//		if (_reports->get(rbuf)) {
//			ret = sizeof(*rbuf);
//		}
//
//	} while (0);
//
//	return ret;

	return OK;

}

int
SERVO12C::set_servo_values()
{
	uint8_t msg[SERVOS_ATTACHED+1] = {0, 10, 127};
	uint8_t * val = &msg[1];
	int ret;
	uint8_t i;

	/* Start at the first servo (port SERVO0) */
	msg[0] = 0;

	/* For every servo, if the value must be adjusted then convert it and store it in the msg array,
	 * otherwise use the old value.
	 */
	for (i = 0; i < SERVOS_ATTACHED; i++) {
		//printf("i = %d, Set value = %d, value = %f \n", i, _controls.set_value[i], _controls.values[i]);
		val[i] = _controls.set_value[i] ? convert(_controls.values[i], i) : _current_values[i];
		//printf("val = %d \n", val[i]);
	}

//	for (i=0; i<SERVOS_ATTACHED+1; i++){
//		printf("msg[%d] = %d \n", i, msg[i]);
//	}

	/*
	 * Send the command to adjust the servo positions.
	 */
	//printf("sizeof(msg) = %d\n", sizeof(msg));
	ret = transfer(msg, sizeof(msg), nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
	}
	else {
		for (i = 0; i < SERVOS_ATTACHED; i++) {
			_current_values[i] = val[i];
		}
	}

	return ret;
}

uint8_t
SERVO12C::convert(float conv, uint8_t servo)
{
	uint8_t ret;

	switch (_input_type) {


		case ABS:
			ret = (uint8_t) conv;
			//printf("[SERVO12C] conv: %f \n", conv);
			//printf("[SERVO12C] ret: %d \n", ret);
			return ret;


		case DEG:
			if (conv < SERVO_MIN_DEG[servo]) {
				return (uint8_t) SERVO_MIN_ABS[servo];
			}
			else if (conv > SERVO_MAX_DEG[servo]) {
				return (uint8_t) SERVO_MAX_ABS[servo];
			}

			ret = SERVO_MIN_ABS[servo] + _conversion_values[servo] * (conv - SERVO_MIN_DEG[servo]);
			ret = (uint8_t) roundf(ret);
			//printf("[SERVO12C] conv: %f \n", conv);
			//printf("[SERVO12C] ret: %d \n", ret);
			return ret;


		case RAD:
			if (conv < SERVO_MIN_DEG[servo]) {
				return (uint8_t) SERVO_MIN_ABS[servo];
			}
			else if (conv > SERVO_MAX_DEG[servo]) {
				return (uint8_t) SERVO_MAX_ABS[servo];
			}

			ret = SERVO_MIN_ABS[servo] + _conversion_values[servo] * (conv - SERVO_MIN_RAD[servo]);
			ret = (uint8_t) roundf(ret);
			//printf("[SERVO12C] conv: %f \n", conv);
			//printf("[SERVO12C] ret: %d \n", ret);
			return ret;

			/* other input types are not supported */
		default:
			return 127;


		}

}


void
SERVO12C::task_cycle_trampoline(int argc, char *argv[])
{
	g_servo12c->task_cycle();
}

void
SERVO12C::task_cycle()
{
	/* Subscribe to the servo12c_control topic */
	_servo_control_topic = orb_subscribe(ORB_ID(servo12c_control));

	pollfd fds[1];
	fds[0].fd = _servo_control_topic;
	fds[0].events = POLLIN;

	log("starting");
	//printf("test _task_exit: %d \n", _task_exit);

	/* loop until killed */
	while (!_task_exit) {

		//printf("test");


		/* sleep waiting for data, but no more than a second */
		int ret = ::poll(&fds[0], 1, 1000);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			continue;
		}

		/* do we have a control update? */
		if (fds[0].revents & POLLIN) {

			/* get controls - must always do this to avoid spinning */
			orb_copy(ORB_ID(servo12c_control), _servo_control_topic, &_controls);

			set_servo_values();

		}

	}

	::close(_servo_control_topic);


	log("stopping");


	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);

}

void
SERVO12C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

}

/**
 * Local functions in support of the shell command.
 */
namespace servo12c
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;



void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_servo12c != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_servo12c = new SERVO12C(SERVO12C_BUS, SERVO12C_BASEADDR);

	//printf("Created driver.\n");

	if (g_servo12c == nullptr) {
		goto fail;
	}

	if (OK != g_servo12c->init()) {
		//printf("Init was not successful");
		goto fail;
	}

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

	exit(0);

fail:

	//printf("In start: we have failed!");

	if (g_servo12c != nullptr)
	{
		delete g_servo12c;
		g_servo12c = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_servo12c != nullptr)
	{
		delete g_servo12c;
		g_servo12c = nullptr;
	}
	else
	{
		errx(1, "driver not running");
	}
	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

//	struct range_finder_multsens_report report;
//	ssize_t sz;
//	int ret;
//
//	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);
//
//	if (fd < 0)
//		err(1, "%s open failed (try 'mb12xx start' if the driver is not running", RANGE_FINDER_DEVICE_PATH);
//
//	/* do a simple demand read */
//	sz = read(fd, &report, sizeof(report));
//
//	if (sz != sizeof(report))
//		err(1, "immediate read failed");
//
//	warnx("single read");
//	// XXX Change such that the results of the whole group are printed:
//	warnx("measurement: %0.2f m", (double)report.distance[0]);
//	warnx("time:        %lld", report.timestamp);
//
//	/* start the sensor polling at 2Hz */
//	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
//		errx(1, "failed to set 2Hz poll rate");
//
//	/* reset the sensor poll rate */
//	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT))
//		errx(1, "failed to reset poll rate");
//
//	/* read the sensor 5x and report each value */
//	for (unsigned i = 0; i < 5; i++) {
//		struct pollfd fds;
//
//		/* wait for data to be ready */
//		fds.fd = fd;
//		fds.events = POLLIN;
//		ret = poll(&fds, 1, 2000);
//
//		if (ret != 1)
//			errx(1, "timed out waiting for sensor data");
//
//		/* now go get it */
//		sz = read(fd, &report, sizeof(report));
//
//		if (sz != sizeof(report))
//			err(1, "periodic read failed");
//
//		warnx("periodic read %u", i);
//		// XXX Change such that the results of the whole group are printed:
//		warnx("measurement: %0.3f", (double)report.distance[0]);
//		warnx("time:        %lld", report.timestamp);
//	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
//	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);
//
//	if (fd < 0)
//		err(1, "failed ");
//
//	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
//		err(1, "driver reset failed");
//
//	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
//		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_servo12c == nullptr) {
		errx(1, "driver not running");
	}

	//printf("state @ %p\n", g_servo12c);
	//g_servo12c->print_info();

	exit(0);
}

} // namespace

int
servo12c_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
			servo12c::start();
	}

	 /*
	  * Stop the driver
	  */
	 if (!strcmp(argv[1], "stop"))
		 servo12c::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		servo12c::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		servo12c::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		servo12c::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
