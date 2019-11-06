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

/**
 * @file MulticopterLandDetector.h
 * Land detection implementation for multicopters.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Morten Lysgaard <morten@lysgaard.no>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include <systemlib/param/param.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>

#include "LandDetector.h"

namespace land_detector
{

class MulticopterLandDetector : public LandDetector
{
public:
	MulticopterLandDetector();

protected:
	virtual void _initialize_topics() override;

	virtual void _update_params() override;

	virtual void _update_topics() override;

	virtual bool _get_landed_state() override;

	virtual bool _get_freefall_state() override;
private:

	/**
	* @brief Handles for interesting parameters
	**/
	struct {
		param_t maxClimbRate;
		param_t maxVelocity;
		param_t maxRotation;
		param_t maxThrottle;
		param_t minManThrottle;
		param_t acc_threshold_m_s2;
		param_t ff_trigger_time;
	} _paramHandle;

	struct {
		float maxClimbRate;
		float maxVelocity;
		float maxRotation_rad_s;
		float maxThrottle;
		float minManThrottle;
		float acc_threshold_m_s2;
		float ff_trigger_time;
	} _params;

	int _vehicleLocalPositionSub;
	int _actuatorsSub;
	int _armingSub;
	int _attitudeSub;
	int _manualSub;
	int _ctrl_state_sub;
	int _vehicle_control_mode_sub;

	struct vehicle_local_position_s		_vehicleLocalPosition;
	struct actuator_controls_s		_actuators;
	struct actuator_armed_s			_arming;
	struct vehicle_attitude_s		_vehicleAttitude;
	struct manual_control_setpoint_s	_manual;
	struct control_state_s			_ctrl_state;
	struct vehicle_control_mode_s		_ctrl_mode;

	uint64_t _min_trust_start;		///< timestamp when minimum trust was applied first
	uint64_t _arming_time;
};


} // namespace land_detector
