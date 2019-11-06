/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file LandDetector.h
Land detector interface for multicopter, fixedwing and VTOL implementations.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <px4_workqueue.h>
#include <systemlib/hysteresis/hysteresis.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_land_detected.h>

namespace land_detector
{


class LandDetector
{
public:
	enum class LandDetectionState {
		FLYING = 0,
		LANDED = 1,
		FREEFALL = 2
	};

	LandDetector();
	virtual ~LandDetector();

	/*
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _taskIsRunning;
	}


	/*
	 * @return current state.
	 */
	LandDetectionState get_state() const
	{
		return _state;
	}

	/*
	 * Tells the task that it should exit.
	 */
	void stop();

	/*
	 * Get the work queue going.
	 */
	int start();

protected:
	/*
	 * Called once to initialize uORB topics.
	 */
	virtual void _initialize_topics() = 0;

	/*
	 * Update uORB topics.
	 */
	virtual void _update_topics() = 0;


	/*
	 * Update parameters.
	 */
	virtual void _update_params() = 0;

	/*
	 * @return true if UAV is in a landed state.
	 */
	virtual bool _get_landed_state() = 0;

	/*
	 * @return true if UAV is in free-fall state.
	 */
	virtual bool _get_freefall_state() = 0;

	/*
	 * Convenience function for polling uORB subscriptions.
	 *
	 * @return true if there was new data and it was successfully copied
	 */
	static bool _orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	/* Run main land detector loop at this rate in Hz. */
	static constexpr uint32_t LAND_DETECTOR_UPDATE_RATE_HZ = 50;

	/* Time in us that landing conditions have to hold before triggering a land. */
	static constexpr uint64_t LAND_DETECTOR_TRIGGER_TIME_US = 2000000;

	/* Time interval in us in which wider acceptance thresholds are used after arming. */
	static constexpr uint64_t LAND_DETECTOR_ARM_PHASE_TIME_US = 2000000;

	orb_advert_t _landDetectedPub;
	struct vehicle_land_detected_s _landDetected;

	int _parameterSub;

	LandDetectionState _state;

	systemlib::Hysteresis _freefall_hysteresis;
	systemlib::Hysteresis _landed_hysteresis;

private:
	static void _cycle_trampoline(void *arg);

	void _cycle();

	void _check_params(const bool force);

	void _update_state();

	bool _taskShouldExit;
	bool _taskIsRunning;

	struct work_s	_work;
};


} // namespace land_detector
