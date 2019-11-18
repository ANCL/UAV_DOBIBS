/****************************************************************************
 *
 * Copyright (c) 2014 ANCL Development Team. All rights reserved.
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
 * @file mc_dobibs_control_main.cpp
 * @author zhijun <zxue2@ualberta.ca>
 * Multicopter Backstepping controller.
 *
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>

#include "BlockDOBIBSController.hpp"

int INFO5();
namespace DOBIBS
{
	static bool thread_should_exit = false;
	static bool thread_running = false;
	static int deamon_task;
	static BlockDOBIBSController *control;
}

/**
 * Deamon management function.
 */
extern "C" __EXPORT int mc_dobibs_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int mc_dobibs_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: mc_dobibs_control {start|stop|status} [-p <additional params>]\n\n");
    return 0;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */


int INFO5()
{
	int sub = -1;
        sub = orb_subscribe(ORB_ID(vehicle_secondary_control_setpoint));

	if (sub>0) {
                struct vehicle_secondary_control_setpoint_s data;
		        memset(&data,0,sizeof(data));
                orb_copy(ORB_ID(vehicle_secondary_control_setpoint), sub, &data);
                PX4_INFO("mc_dobibs_control OUTPUT");
                PX4_INFO("Timestamp: (%" PRIu64 ")",data.timestamp);
                PX4_INFO("Roll: (%.5f)",(double)data.control[0] );
                PX4_INFO("Pitch: (%.5f)",(double)data.control[1] );
                PX4_INFO("Yaw: (%.5f)",(double)data.control[2] );
                PX4_INFO("Thrust: (%.5f)",(double)data.control[3] );
	} else {
				PX4_INFO("Could not subscribe to vehicle_attitude_setpoint topic");
				return 1;
        }
        sub = orb_unsubscribe(sub);
return OK;
}

int mc_dobibs_control_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {

		if (DOBIBS::thread_running) {
            /* this is not an error */
			warnx("already running");
			/*have to use return instead of exit*/
			//exit(0);
            return 0;
		}

		DOBIBS::thread_should_exit = false;
                PX4_INFO("mc_dobibs_control START");
		DOBIBS::deamon_task = px4_task_spawn_cmd("mc_dobibs_control",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_ATTITUDE_CONTROL,
						 4048,
						 mc_dobibs_control_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
        return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		DOBIBS::thread_should_exit = true;
        return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (DOBIBS::thread_running) {
			warnx("is running");
			INFO5();
		} else {
			warnx("not started");
		}

        return 0;
	}

	usage("unrecognized command");
    return 1;
}

int mc_dobibs_control_thread_main(int argc, char *argv[])
{

	warnx("starting");

	DOBIBS::control = new BlockDOBIBSController;

	if (DOBIBS::control == nullptr) {
		warnx("alloc failed");
		return 1;
	}

	DOBIBS::thread_running = true;

	while (!DOBIBS::thread_should_exit) {
		DOBIBS::control->update();
	}

	warnx("exiting.");
	delete DOBIBS::control;
	DOBIBS::control = nullptr;
	DOBIBS::thread_running = false;

	return 0;
}



