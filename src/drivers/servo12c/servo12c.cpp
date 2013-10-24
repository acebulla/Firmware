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
#define SERVO12C_BASEADDR 	0x28 /* 7-bit address. 40 decimal */

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
	bool				_task_exit;
	RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;

	uint8_t				_current_values[SERVOS_ATTACHED];




	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
	* and MB12XX_MAX_DISTANCE
	*/
	float				get_minimum_distance();
	float				get_maximum_distance();


	int					set_servo_values();

	/**
	 * Convert degree or radian to absolute value.
	 * Cast absolute values from float to uint8_t.
	 * Enforces boundaries.
	 *
	 * @param conv		The to be converted value.
	 */

	uint8_t				convert(float conv);


	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main() __attribute__((noreturn));


};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int servo12c_main(int argc, char *argv[]);

SERVO12C::SERVO12C(int bus, uint8_t address) :
	I2C("SERVO12C", SERVO12C_DEVICE_PATH, bus, address, 100000),
	_input_type(ABS),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_task(-1),
	_task_exit(false),
	_sample_perf(perf_alloc(PC_ELAPSED, "servo12c_write")),
	_comms_errors(perf_alloc(PC_COUNT, "servo12c_comms_errors"))
{
	// enable debug() calls
	_debug_enabled = true;
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

namespace
{

SERVO12C	*g_servo12c;

}

int
SERVO12C::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		ret = -errno;

	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	/* start the HIL interface task */
	_task = task_spawn_cmd("servo12c",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   2048,
			   (main_t)&SERVO12C::task_main_trampoline,
			   nullptr);

	if (_task < 0) {
		debug("task start failed: %d", errno);
		ret = -errno;
	}

	return ret;
}

/*
int
SERVO12C::probe()
{
	return measure();
}

void
SERVO12C::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
SERVO12C::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
SERVO12C::get_minimum_distance()
{
	return _min_distance;
}

float
SERVO12C::get_maximum_distance()
{
	return _max_distance;
}
*/

int
SERVO12C::ioctl(struct file *filp, int cmd, unsigned long arg)
{
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
					return OK;

					/* switching to radian input values */
				case SERVO_INPUT_RAD:
					_input_type = RAD;
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
	int ret;
	uint8_t i;

	for (i = _sensor_start; i < _sensor_count; i++) {
		/*
		 * Adjust the address to the current sensor.
		 */
		set_address(_addresses[i] & ADDRPART);

		/*
		 * Send the command to begin a measurement.
		 */
		uint8_t cmd = MB12XX_TAKE_RANGE_REG;
		ret = transfer(&cmd, 1, nullptr, 0);

		if (OK != ret)
		{
			perf_count(_comms_errors);
			log("i2c::transfer returned %d for sensor with address %X", ret, _addresses[i] & ADDRPART);
			// XXX Maybe we want to continue if there is a problem with only one sensor
			_sensor_end = i;
			return ret;
		}

		if (_addresses[i] & GROUPEND) {
			_sensor_end = i;
			ret = OK;
			break;
		}

	}

	if (!(_addresses[i] & GROUPEND))
	{
		perf_count(_comms_errors);
		log("mb12xx::End of the final sensor group is not marked");
		ret = -EPERM;
	}

	return ret;
}

int
SERVO12C::collect()
{
	int	ret = -EIO;
	uint8_t i;
	struct range_finder_multsens_report report;

	perf_begin(_sample_perf);

	/* read from the sensor */
	uint8_t val[2] = {0, 0};
	report.sensor_start = _sensor_start;
	i = _sensor_start;

	while (i <= _sensor_end) {
		set_address(_addresses[i] & ADDRPART);

		ret = transfer(nullptr, 0, &val[0], 2);

		if (ret < 0)
		{
			log("error reading from sensor with address %X: %d", _addresses[i] & ADDRPART, ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		uint16_t distance = val[0] << 8 | val[1];
		float si_units = (distance * 1.0f)/ 100.0f; /* cm to m */

		report.distance[i] = si_units;
		report.valid[i] = si_units > get_minimum_distance() && si_units < get_maximum_distance() ? 1 : 0;

		i++;
	}

	report.sensor_end = i-1;
	_sensor_start = (i < _sensor_count) ? i : 0;

	/* This should be fairly close to the end of the measurement, so the best approximation of the time.
	 * We record the time after all sensors in the group have been measured.
	 * */
	report.timestamp = hrt_absolute_time();

	/* publish it */
	orb_publish(ORB_ID(multsens_range_finder), _range_finder_topic, &report);

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
SERVO12C::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();
	_sensor_start = 0;
	_sensor_end = 0;

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&SERVO12C::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_RANGEFINDER};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
SERVO12C::stop()
{
	work_cancel(HPWORK, &_work);
}

void
SERVO12C::cycle_trampoline(void *arg)
{
	SERVO12C *dev = (SERVO12C *)arg;

	dev->cycle();
}

void
SERVO12C::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(MB12XX_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&SERVO12C::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(MB12XX_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&SERVO12C::cycle_trampoline,
		   this,
		   USEC2TICK(MB12XX_CONVERSION_INTERVAL));
}

void
SERVO12C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace mb12xx
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

MB12XX	*g_dev;

void	start(uint8_t addresses[], uint8_t sensor_count);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(uint8_t *addresses, uint8_t sensor_count)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new MB12XX(MB12XX_BUS, addresses, sensor_count);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

	exit(0);

fail:

	if (g_dev != nullptr)
	{
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr)
	{
		delete g_dev;
		g_dev = nullptr;
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
	struct range_finder_multsens_report report;
	ssize_t sz;
	int ret;

	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'mb12xx start' if the driver is not running", RANGE_FINDER_DEVICE_PATH);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	// XXX Change such that the results of the whole group are printed:
	warnx("measurement: %0.2f m", (double)report.distance[0]);
	warnx("time:        %lld", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* reset the sensor poll rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT))
		errx(1, "failed to reset poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		// XXX Change such that the results of the whole group are printed:
		warnx("measurement: %0.3f", (double)report.distance[0]);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(RANGE_FINDER_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace

int
mb12xx_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		int i;
		uint8_t addri = 0, addrcount = 0;
		uint8_t * addr;

		if (argc > 3 && (strcmp(argv[2], "-a") == 0 || strcmp(argv[2], "--addrgroups") == 0)) {
			addrcount = atoi(argv[3]);
			// printf("addrcount: %d\n", addrcount);
			if (addrcount <= MAX_SENSOR_COUNT) {
				addr = new uint8_t[addrcount];
				for (i = 4; i < argc; i++) {
					if (strcmp(argv[i], ",") == 0) {
						addr[addri-1] |= GROUPEND;
						// printf("%X \n", addr[addri-1]);
						continue;
					}

					addr[addri] = (uint8_t) atoi(argv[i]);
					// printf("addri: %d\t addr: %d \n", addri, addr[addri]);
					addri++;
				}

				/* Last address must be the end of a group in any case. */
				addr[addri-1] |= GROUPEND;
			}

			mb12xx::start(addr, addrcount);
			return OK;
		}

		addrcount = 1;
		addr = new uint8_t[addrcount];
		addr[0] = MB12XX_BASEADDR | GROUPEND;

		mb12xx::start(addr, addrcount);
	}

	 /*
	  * Stop the driver
	  */
	 if (!strcmp(argv[1], "stop"))
		 mb12xx::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		mb12xx::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		mb12xx::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		mb12xx::info();

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
