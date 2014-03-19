/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Alexander Cebulla <acebulla@student.ethz.ch>
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
 * @file servo_calibration.c
 * An application for calibrating the pan/tilt platform, such that the position of the servos
 * 	correspond to the correct angles.
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>

#include <uORB/uORB.h>
#include <uORB/topics/marker_location.h>

#include <systemlib/param/param.h>

#include <drivers/drv_servo12c.h>


__EXPORT int servo_calibration_main(int argc, char *argv[]);

/**
 * Sets the position of the servo and measures the corresponding angle.
 */
void get_measurements(uint8_t servo, uint8_t pos_abs[], float pos_angle[]);


/**
 *  Sends the servo position of servo to the servo12c driver.
 */
void send_pos(uint8_t servo, uint8_t servo_position);

/**
 * Reads out the measured angle 5 times and returns the average.
 */
float average_pos(uint8_t servo);

/**
 * Finds a and b, such that y = a*x + b, where x in RAD and y in ABS
 */
void lin_regression(uint8_t pos_abs[], float pos_angle[], float *a, float *b);

/* structures */
struct marker_location_s marker_loc;
struct servo_control_values servo_control;

/* variables */
int marker_location_sub;
orb_advert_t servo_control_pub;

uint8_t nPoints = 10;
uint8_t start_pos = 120;

struct pollfd fds[1];

int servo_calibration_main(int argc, char *argv[])
{
	/* set the position input to ABS */
	int fd = open(SERVO12C_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not set input type");;
	}

	if (ioctl(fd, SERVO_INPUT, SERVO_INPUT_ABS) < 0) {
		close(fd);
		errx(1, "Could not set input type");;
	}

	close(fd);

	/* Subscription to camera message */
	memset(&marker_loc, 0, sizeof(marker_loc));
	marker_location_sub = orb_subscribe(ORB_ID(marker_location));

	fds[0].fd = marker_location_sub;
	fds[0].events = POLLIN;

	memset(&servo_control, 0, sizeof(servo_control));

	uint8_t i;

	for (i = 0; i < SERVOS_ATTACHED; i++) {
		servo_control.set_value[i] = 1;
		servo_control.values[i] = start_pos;
	}

	/* publish inital servo positions */
	servo_control_pub = orb_advertise(ORB_ID(servo12c_control), &servo_control);
	usleep(100000);

	uint8_t pos_abs[nPoints];
	float pos_angle[nPoints];

	param_t param_slope;
	param_t param_yintercept;

	float a = 0.0f, b = 0.0f;

	for (i = 0; i < 2; i++) {
		send_pos(0, start_pos);
		send_pos(1, start_pos);
		usleep(1000000);
		get_measurements(i, pos_abs, pos_angle);
		lin_regression(pos_abs, pos_angle, &a, &b);
		printf("y = %.3f*x + %.3f \n", a, b);
		if (i == 0) {
			param_slope = param_find("SERVO_PAN_SLOPE");
			param_yintercept = param_find("SERVO_PAN_YINT");
		} else {
			param_slope = param_find("SERVO_TILT_SLOPE");
			param_yintercept = param_find("SERVO_TILT_YINT");
		}
		param_set(param_slope, &a);
		param_set(param_yintercept, &b);
	}

	param_save_default();


	return OK;
}


void get_measurements(uint8_t servo, uint8_t pos_abs[], float pos_angle[])
{
	int i, poll_ret;
	uint8_t servo_position = start_pos;
	uint8_t step;
	bool dir = true;
	for (i = 0; i < 2; i++)
	{
		/* Find extreme points */
		/* wait for sensor update of 1 file descriptor for 100 ms */
		while ( (poll_ret = poll(fds, 1, 100)) != 0)
		{
			orb_copy(ORB_ID(marker_location), marker_location_sub, &marker_loc);
//			printf("%.2f \n", marker_loc.pan);
			servo_position += dir ? 5 : -5;
			send_pos(servo, servo_position);
			usleep(90000);
		}
		while ( (poll_ret = poll(fds, 1, 100)) == 0)
		{
			servo_position += dir ? -1 : 1;
			send_pos(servo, servo_position);
		}
		usleep(1000000);
		pos_abs[i] = servo_position;
		pos_angle[i] = average_pos(servo);
		dir = !dir;
	}

	printf("pos_abs[0]: %d , pos_angle[0]: %.6f \n", pos_abs[0], pos_angle[0]);
	printf("pos_abs[1]: %d , pos_angle[1]: %.6f \n", pos_abs[1], pos_angle[1]);
	step = (uint8_t) roundf((float) (pos_abs[0] - pos_abs[1]) / (float) nPoints);
	while ((servo_position < pos_abs[0]) && (i < nPoints)) {
		servo_position += step;
		send_pos(servo, servo_position);
		usleep(1000000);
		pos_abs[i] = servo_position;
		pos_angle[i] = average_pos(servo);
		printf("pos_abs[%d]: %d , pos_angle[%d]: %.6f \n", i, pos_abs[i], i, pos_angle[i]);
		i++;
	}

}

void send_pos(uint8_t servo, uint8_t servo_position)
{
	servo_control.set_value[0] = 0; servo_control.set_value[1] = 0;
	servo_control.set_value[servo] = 1;
	servo_control.values[servo] = (float) servo_position;
	orb_publish(ORB_ID(servo12c_control), servo_control_pub, &servo_control);
}

float average_pos(uint8_t servo)
{
	float angle = 0.0f;
	int error_counter = 0;
	uint8_t i;

	for (i = 0; i<5; i++)
	{
		/* wait for sensor update of 1 file descriptor for 100 ms */
		int poll_ret = poll(fds, 1, 100);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* This means the camera does not detect the marker. Not good */
			printf("[servo_calibration] In average_pos(): Did not receive data. \n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[servo_calibration] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(marker_location), marker_location_sub, &marker_loc);
				angle += (servo == 0) ? marker_loc.pan : marker_loc.tilt;
			}

		}
	}

	return angle / 5.0f;
}

void lin_regression(uint8_t pos_abs[], float pos_angle[], float *a, float *b)
{
	/* Finds a and b, such that y = a*x + b, where x in RAD and y in ABS.
	 * 	x = pos_angle; y = pos_abs */
	float x_avg = 0.0f, y_avg = 0.0f, a1 = 0.0f, a2 = 0.0f;
	uint8_t i;
	for (i = 0; i < nPoints; i++)
	{
		x_avg += pos_angle[i];
		y_avg += (float) pos_abs[i];
	}
	x_avg = x_avg / nPoints;
	y_avg = y_avg / nPoints;

	for (i = 0; i < nPoints; i++)
	{
		a1 += (pos_angle[i] - x_avg)*(pos_abs[i] - y_avg);
		a2 += (pos_angle[i] - x_avg)*(pos_angle[i] - x_avg);
	}
	*a = a1 / a2;
	*b = y_avg - (*a)*x_avg;

}
