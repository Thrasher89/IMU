/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file streamer.c
 * 
 * @author Example User <mail@example.com>
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include "serial.h"


#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>

#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_gps_heading.h>


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */


void streamLoop(Serial out);

/**
 * daemon management function.
 */
extern "C" { __EXPORT int streamer_main(int argc, char *argv[]); }

/**
 * Mainloop of daemon.
 */
int streamer_thread_main(int argc, char *argv[]);

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
int streamer_main(int argc, char *argv[])
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
		daemon_task = task_spawn_cmd("streamer",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2000,
					 streamer_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
	printf("2_\n");
	fflush(stdout);
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

int streamer_thread_main(int argc, char *argv[]) {
	warnx("[streamer] starting\n");

	thread_running = true;
	Serial out;
	out.init("/dev/ttyACM0",921600);
	out.serialOpen(true);
	streamLoop(out);
	out.serialClose();

	warnx("[streamer] exiting.\n");

	thread_running = false;

	return 0;
}

void streamLoop(Serial out){
	bool gps_updated;
	int  gps_heading_sub = orb_subscribe(ORB_ID(vehicle_gps_heading));
	int  gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	uint8_t outBuffer[64];
	int i;


	printf("in loop");
	fflush(stdout);

	while (!thread_should_exit) {
		orb_check(gps_heading_sub, &gps_updated);
		if (gps_updated) {
			struct vehicle_gps_heading_s gps;
			orb_copy(ORB_ID(vehicle_gps_heading), gps_heading_sub, &gps);
			//printf("gps updated heading%f \n",(double)gps.hdg);
			fflush(stdout);
			i=0;
			outBuffer[i++]='g';
			outBuffer[i++]='p';
			outBuffer[i++]='s';
			outBuffer[i++]='h';

			memcpy(&outBuffer[i],&(gps.timestamp_time),sizeof(gps.timestamp_time));
			i=i+sizeof(gps.timestamp_time); //i=8
			i=i+sprintf((char*)&outBuffer[i], " %08.4f, %07.3f, %08.4f, %06.3f ",(double)gps.hdg,(double)gps.hdg_deviation, (double)gps.ptch,(double)gps.ptch_deviation);

			/**
			  *Fill up to 48
			  */
			while(i<=53)
				outBuffer[i++]=' ';

			outBuffer[i++]='\r';
			outBuffer[i++]='\n';
			out.putData(outBuffer,i);

		}
		orb_check(gps_position_sub, &gps_updated);
		if (gps_updated) {
			struct vehicle_gps_position_s gps_pos;
			orb_copy(ORB_ID(vehicle_gps_position), gps_position_sub, &gps_pos);
			i=0;
			outBuffer[i++]='g';
			outBuffer[i++]='p';
			outBuffer[i++]='s';
			outBuffer[i++]='p';
			//i=i+sprintf((char*)&outBuffer[i], " %08.4f, +-%07.3f, %08.4f, +-%06.3f ",(double)gps.hdg,(double)gps.hdg_deviation, (double)gps.ptch,(double)gps.ptch_deviation);

			memcpy(&outBuffer[i],&(gps_pos.timestamp_position),sizeof(gps_pos.timestamp_position));
			i=i+sizeof(gps_pos.timestamp_position); //i=8
			i=i+sprintf((char*)&outBuffer[i], " %d;%d;%d",gps_pos.lat,gps_pos.lon,gps_pos.alt);

			while(i<=53)
				outBuffer[i++]=' ';
			outBuffer[i++]='\r';
			outBuffer[i++]='\n';
			out.putData(outBuffer,i);

		}

	}
}
