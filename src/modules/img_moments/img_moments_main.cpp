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
 * @file img_moments.cpp
 * Image Moments information.
 *
 * @author Kenny <kenny3@ualberta.ca>
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
#include <errno.h>
#include <uORB/topics/img_moments.h>
#include <uORB/topics/vehicle_secondary_attitude_setpoint.h>
#include <math.h>
#include <systemlib/scheduling_priorities.h>

/**
 *
 *
 * @ingroup apps
 */
extern "C" __EXPORT int img_moments_main(int argc, char *argv[]);

class FakeImgMoments {
public:
	FakeImgMoments();
	~FakeImgMoments();
	int start();
private:
	struct img_moments_s img_moments;
	orb_advert_t pub;
	int task;
	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();
	bool task_should_exit;
};

namespace fake_img_moments {
	FakeImgMoments *fake;
}

FakeImgMoments::FakeImgMoments() : pub(nullptr), task(-1), task_should_exit(false) {
	memset(&img_moments,0,sizeof(img_moments));
}

FakeImgMoments::~FakeImgMoments() {

	if (task != -1) {
		task_should_exit = true;

		unsigned i =0;

		do {
			usleep(20000);
			if (++i>50) {
				px4_task_delete(task);
				break;
			}
		} while (task != -1);
	}

	fake_img_moments::fake = nullptr;

	if (!pub)
		orb_unadvertise(pub);
}

int
FakeImgMoments::start() {
	ASSERT(task == -1);
	task = px4_task_spawn_cmd("fake_img_moments",
				SCHED_DEFAULT,
				SCHED_PRIORITY_SLOW_DRIVER-1,
				1296,
				(px4_main_t)&FakeImgMoments::task_main_trampoline,
				nullptr);
	if (task < 0) {
		warnx("Task start failed");
		return -errno;
	}

	return OK;
}

void
FakeImgMoments::task_main_trampoline(int argc, char *argv[]) {
	fake_img_moments::fake->task_main();
}

void
FakeImgMoments::task_main() {

	while (!task_should_exit) {
		usleep(10000);
		
		img_moments.timestamp = hrt_absolute_time();

		img_moments.s[0]=sin(img_moments.timestamp);
		img_moments.s[1]=cos(img_moments.timestamp);;
		img_moments.s[2]=1+sin(img_moments.timestamp);;
		img_moments.s[3]=sin(img_moments.timestamp);;
		
		img_moments.usec = img_moments.timestamp-1000;

		if (pub == nullptr) {
			pub = orb_advertise(ORB_ID(img_moments),&img_moments);
		} else {
			orb_publish(ORB_ID(img_moments),pub,&img_moments);
		}

	}

	task = -1;
}
	


int img_moments_main(int argc, char *argv[])
{
        int sub=-1;
        struct img_moments_s data;
        struct vehicle_secondary_attitude_setpoint_s img_sp;
        memset(&data,0,sizeof(data));

        if (argc < 2) {
                warnx("usage: img_moments {status|fake}");
                return 1;
        }

        if (!strcmp(argv[1], "status")) {
                sub = orb_subscribe(ORB_ID(img_moments));
                if (sub>0) {
                        PX4_INFO("Image moments:");
			if (fake_img_moments::fake !=nullptr)
			PX4_INFO("FAKE DATA!!!");
                        orb_copy(ORB_ID(img_moments), sub, &data);
                        PX4_INFO("Timestamp: (%u)",data.usec);
                        PX4_INFO("s1 of Objects: (%.5f)",(double)data.s[0]);
                        PX4_INFO("s2 of Objects: (%.5f)",(double)data.s[1]);
                        PX4_INFO("s3 of Objects: (%.5f)",(double)data.s[2]);
                        PX4_INFO("s4 of Objects: (%.5f)",(double)data.s[3]);
                        PX4_INFO("Valid: %u",data.valid);
                } else {
                        PX4_INFO("Could not subscribe to img_moments topic");
                }
                sub = orb_unsubscribe(sub);
                return 0;
        }

	if (!strcmp(argv[1], "fake")) {

		if (!strcmp(argv[2], "start")) {
			if (fake_img_moments::fake != nullptr) {
				warnx("already running");
				return 1;
			}

			fake_img_moments::fake = new FakeImgMoments;

			if (OK != fake_img_moments::fake->start()) {
				delete fake_img_moments::fake;
				fake_img_moments::fake = nullptr;
				warnx("start failed");
				return 1;
			}

			return 0;
		}

		if (!strcmp(argv[2], "stop")) {
			if (fake_img_moments::fake == nullptr) {
				warnx("not running");
				return 1;
			}
			delete fake_img_moments::fake;
			fake_img_moments::fake = nullptr;
			return 0;
		}

	}

        if (!strcmp(argv[1], "ibvs")) {
                sub = orb_subscribe(ORB_ID(vehicle_secondary_attitude_setpoint));
                if (sub>0) {
                        PX4_INFO("Image moments:");
                        orb_copy(ORB_ID(vehicle_secondary_attitude_setpoint), sub, &img_sp);
                        PX4_INFO("Timestamp: (%" PRIu64 ")",img_sp.timestamp);
                        PX4_INFO("Roll: (%.5f)",(double)img_sp.roll);
                        PX4_INFO("Pitch: (%.5f)",(double)img_sp.pitch);
                        PX4_INFO("Yaw: (%.5f)",(double)img_sp.yaw);
                        PX4_INFO("Thrust: (%.5f)",(double)img_sp.thrust);
                        if(img_sp.valid){
                            PX4_INFO("Valid: TRUE");
                        } else {
                            PX4_INFO("Valid: FALSE");
                        }

                } else {
                        PX4_INFO("Could not subscribe to vehicle_secondary_attitude_setpoint topic");
                }
                sub = orb_unsubscribe(sub);
                return 0;
        }




	
        warnx("unrecognized command");
        return 1;
}
