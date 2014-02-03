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
 * @file wiiIRsensor.cpp
 * @author Alexander Cebulla
 *
 * Driver for the Wii IR sensor connected via I2C.
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

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_wiiIRsensor.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

/* Configuration Constants */
#define WIIIRSENSOR_BUS 			PX4_I2C_BUS_EXPANSION
#define WIIIRSENSOR_BASEADDR 	0x58 /* 7-bit address. 8-bit address is 0xB0 */

/* MB12xx Registers addresses */

#define WIIIRSENSOR_TAKE_READING_SHORT	0x36		/* Take a reading. Output format is short */
#define WIIIRSENSOR_TAKE_READING_MEDIUM	0x33		/* Take a reading. Output format is medium */
#define WIIIRSENSOR_TAKE_READING_LONG	0x3E		/* Take a reading. Output format is long */

	 
#define WIIIRSENSOR_CONVERSION_INTERVAL 5000 /* 5ms */

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class WIIIRSENSOR : public device::I2C
{
public:
	WIIIRSENSOR(int bus = WIIIRSENSOR_BUS, int address = WIIIRSENSOR_BASEADDR);
	virtual ~WIIIRSENSOR();
	
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
	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	
	// http://makezine.com/2008/11/22/hacking-the-wiimote-ir-ca/
	uint8_t				_p0;	// MAXSIZE: Maximum blob size. Wii uses values from 0×62 to 0xc8
	uint8_t				_p1;	// GAIN: Sensor Gain. Smaller values = higher gain
	uint8_t				_p2;	// GAINLIMIT: Sensor Gain Limit. Must be less than GAIN for camera to function.
	uint8_t				_p3;	// MINSIZE: Minimum blob size. Wii uses values from 3 to 5

	orb_advert_t		_wii_IR_sensor_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return		True if the device is present.
	*/
	int					probe_address(uint8_t address);
	
	/**
	* Sends two bytes via I2C
	*
	* @param d1, d2	The first and the second byte.
	* @return		The result of the transfer function.
	*/
	int					write_2bytes(uint8_t d1, uint8_t d2);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();
	
	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();
	
	
	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void		cycle_trampoline(void *arg);
	
	
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int wiiIRsensor_main(int argc, char *argv[]);

WIIIRSENSOR::WIIIRSENSOR(int bus, int address) :
	I2C("wiiIRsensor", WII_IR_SENSOR_DEVICE_PATH, bus, address, 100000),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_p0(0x72), _p1(0x20), _p2(0x1F), _p3(0x03), // Level 2
	_wii_IR_sensor_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "mb12xx_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mb12xx_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "mb12xx_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;
	
	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

WIIIRSENSOR::~WIIIRSENSOR()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;
}

int
WIIIRSENSOR::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(wii_IR_report));

	if (_reports == nullptr)
		goto out;

	/* get a publish handle on the Wii IR topic */
	struct wii_IR_report zero_report;
	memset(&zero_report, 0, sizeof(zero_report));
	_wii_IR_sensor_topic = orb_advertise(ORB_ID(wii_IR_sensor), &zero_report);

	if (_wii_IR_sensor_topic < 0)
		debug("failed to create sensor_range_finder object. Did you start uOrb?");

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	/* start measurements */
	start();

out:
	return ret;
}

int
WIIIRSENSOR::probe()
{
	// Use probe to initialize the IR sensor:
	int ret;

	// Use simple initialization
    write_2bytes(0x30,0x01); usleep(10000);
    write_2bytes(0x30,0x08); usleep(10000);
    write_2bytes(0x06,0x90); usleep(10000);
    write_2bytes(0x08,0xC0); usleep(10000);
    write_2bytes(0x1A,0x40); usleep(10000);
    ret = write_2bytes(0x33,0x33); usleep(10000);
    usleep(100000);

	return ret;
}


int
WIIIRSENSOR::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case WIIIRSENSORIOCMAXSIZE:
		_p0 = arg;
		break;

	case WIIIRSENSORIOCGAIN:
		_p1 = arg;
		break;

	case WIIIRSENSORIOCGAINLIMIT:
		_p2 = arg;
		break;
		
	case WIIIRSENSORIOCMINSIZE:
		_p3 = arg;
		break;
	
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
WIIIRSENSOR::read(struct file *filp, char *buffer, size_t buflen)
{
//	unsigned count = buflen / sizeof(struct range_finder_report);
//	struct range_finder_report *rbuf = reinterpret_cast<struct range_finder_report *>(buffer);
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

int WIIIRSENSOR::write_2bytes(uint8_t d1, uint8_t d2)
{
	uint8_t msg[2];
	int ret = 0;

	msg[0] = d1; msg[1] = d2;

	ret = transfer(msg, sizeof(msg), nullptr, 0);

	return ret;
}

int
WIIIRSENSOR::measure()
{
	int ret;
	int Ix1, Iy1, Ix2, Iy2, Ix3, Iy3, Ix4, Iy4, s;

	/*
	 * Send the command to begin a measurement.
	 */
	uint8_t cmd = WIIIRSENSOR_TAKE_READING_SHORT;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret)
	{
		perf_count(_comms_errors);
		log("i2c::transfer returned %d", ret);
		return ret;
	}
	
	/* read from the sensor */
	uint8_t val[16];
	
	perf_begin(_sample_perf);
	
	ret = transfer(nullptr, 0, &val[0], 16);
	
	if (ret < 0)
	{
		log("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}
	
	/* Conversion */
	struct wii_IR_report report;

	Ix1 = val[1];
	Iy1 = val[2];
    s   = val[3];
    Ix1 += (s & 0x30) <<4;
    Iy1 += (s & 0xC0) <<2;

    Ix2 = val[4];
    Iy2 = val[5];
    s   = val[6];
    Ix2 += (s & 0x30) <<4;
    Iy2 += (s & 0xC0) <<2;

    Ix3 = val[7];
    Iy3 = val[8];
    s   = val[9];
    Ix3 += (s & 0x30) <<4;
    Iy3 += (s & 0xC0) <<2;

    Ix4 = val[10];
    Iy4 = val[11];
    s   = val[12];
    Ix4 += (s & 0x30) <<4;
    Iy4 += (s & 0xC0) <<2;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
    report.Ix1 = Ix1; report.Iy1 = Iy1;
    report.Ix2 = Ix2; report.Iy2 = Iy2;
    report.Ix3 = Ix3; report.Iy3 = Iy3;
    report.Ix4 = Ix4; report.Iy4 = Iy4;
	report.timestamp = hrt_absolute_time();
	
	/* publish it */
	orb_publish(ORB_ID(wii_IR_sensor), _wii_IR_sensor_topic, &report);

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
WIIIRSENSOR::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&WIIIRSENSOR::cycle_trampoline, this, 1);
	
//	/* notify about state change */
//	struct subsystem_info_s info = {
//		true,
//		true,
//		true,
//		SUBSYSTEM_TYPE_RANGEFINDER};
//	static orb_advert_t pub = -1;
//
//	if (pub > 0) {
//		orb_publish(ORB_ID(subsystem_info), pub, &info);
//	} else {
//		pub = orb_advertise(ORB_ID(subsystem_info), &info);
//	}
}

void
WIIIRSENSOR::stop()
{
	work_cancel(HPWORK, &_work);
}

void
WIIIRSENSOR::cycle_trampoline(void *arg)
{
	WIIIRSENSOR *dev = (WIIIRSENSOR *)arg;

	dev->cycle();
}

void
WIIIRSENSOR::cycle()
{
	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&WIIIRSENSOR::cycle_trampoline,
		   this,
		   USEC2TICK(WIIIRSENSOR_CONVERSION_INTERVAL));
}

void
WIIIRSENSOR::print_info()
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
namespace wiiIRsensor
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

WIIIRSENSOR	*g_dev;

void	start();
void	stop();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new WIIIRSENSOR(WIIIRSENSOR_BUS);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
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
wiiIRsensor_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		wiiIRsensor::start();
	
	 /*
	  * Stop the driver
	  */
	 if (!strcmp(argv[1], "stop"))
		 wiiIRsensor::stop();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status"))
		wiiIRsensor::info();

	errx(1, "unrecognized command, try 'start', 'stop' or 'info'");
}
