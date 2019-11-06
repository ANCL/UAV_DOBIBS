/****************************************************************************
 *
 *   Copyright (c) 2013 - 2015 PX4 Development Team. All rights reserved.
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
 * @file vicon.cpp
 * Vicon Status information.
 *
 * @author Geoff <gfink@ualbert.ca>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <poll.h>
#include <uORB/topics/vicon.h>


/**
 *
 *
 * @ingroup apps
 */
extern "C" __EXPORT int vicon_main(int argc, char *argv[]);

int print_vicon_status();
int print_vicon_hz();

int vicon_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: vicon {status|hz}");
		return 1;
	}
	if (!strcmp(argv[1], "status")) return print_vicon_status();
	if (!strcmp(argv[1], "hz")) return print_vicon_hz();
	
	warnx("unrecognized command");
	return 1;
}

int print_vicon_status() {
	int sub = -1;
	sub = orb_subscribe(ORB_ID(vicon));
	if (sub>0) {
		struct vicon_s data;
		memset(&data,0,sizeof(data));
		orb_copy(ORB_ID(vicon), sub, &data);
		PX4_INFO("Vicon:");
		PX4_INFO("pos: (%2.3f,%2.3f,%2.3f)",(double)data.p[0],(double)data.p[1],(double)data.p[2]);
		PX4_INFO("vel: (%2.3f,%2.3f,%2.3f)",(double)data.v[0],(double)data.v[2],(double)data.v[2]);
		PX4_INFO("q: (%2.3f,%2.3f,%2.3f,%2.3f)",(double)data.q[0],(double)data.q[1],(double)data.q[2],(double)data.q[3]);
		PX4_INFO("received @ %" PRIu64 " / %" PRIu64 "\n      sent @ %" PRIu64, data.t_local, hrt_absolute_time(), data.t_remote);
		sub = orb_unsubscribe(sub);
	} else {
		PX4_INFO("Could not subscribe to Vicon topic");
		return 1;
	}
	return 0;
}

int print_vicon_hz() {
	int sub = -1;
	sub = orb_subscribe(ORB_ID(vicon));
	if (sub>0) {
		px4_pollfd_struct_t fds;
		fds.fd = sub;
		fds.events = POLLIN;
		uint64_t t0 = hrt_absolute_time();
		uint64_t t = 0;
		uint64_t count = 0;
		while (t<5000000 && count < 250) { //5 s or 250 msgs
			px4_poll(&fds, 1, 1000);	//1 s
			t=hrt_absolute_time()-t0;
			if (fds.revents & POLLIN) {
				orb_copy(ORB_ID(vicon), sub, NULL);
				count++;
			}
		}
		PX4_INFO("Vicon:");
		PX4_INFO("count = %" PRIu64, count); 
		PX4_INFO("time = %" PRIu64 " ms", t/1000); 
		PX4_INFO( "%3.1f Hz", double(count)/((double)t/(double)1000000));
	} else {
		PX4_INFO("Could not subscribe to Vicon topic");
		return 1;
	}
	return 0;		

}
