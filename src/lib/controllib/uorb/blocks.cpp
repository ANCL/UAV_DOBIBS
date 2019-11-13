/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file uorb_blocks.cpp
 *
 * uorb block library code
 */

#include "blocks.hpp"
#include <geo/geo.h>

namespace control
{

BlockWaypointGuidance::BlockWaypointGuidance(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	_xtYawLimit(this, "XT2YAW"),
	_xt2Yaw(this, "XT2YAW"),
	_psiCmd(0)
{
}

BlockWaypointGuidance::~BlockWaypointGuidance() {};

void BlockWaypointGuidance::update(
	const vehicle_global_position_s &pos,
	const vehicle_attitude_s &att,
	const position_setpoint_s &missionCmd,
	const position_setpoint_s &lastMissionCmd)
{

	// heading to waypoint
	float psiTrack = get_bearing_to_next_waypoint(
				 (double)pos.lat / (double)1e7,
				 (double)pos.lon / (double)1e7,
				 missionCmd.lat,
				 missionCmd.lon);

	// cross track
	struct crosstrack_error_s xtrackError;
	get_distance_to_line(&xtrackError,
			     (double)pos.lat / (double)1e7,
			     (double)pos.lon / (double)1e7,
			     lastMissionCmd.lat,
			     lastMissionCmd.lon,
			     missionCmd.lat,
			     missionCmd.lon);

	_psiCmd = _wrap_2pi(psiTrack -
			    _xtYawLimit.update(_xt2Yaw.update(xtrackError.distance)));
}

BlockUorbEnabledAutopilot::BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// subscriptions
	_att(ORB_ID(vehicle_attitude), 20, 0, &getSubscriptions()),
	_attCmd(ORB_ID(vehicle_attitude_setpoint), 20, 0, &getSubscriptions()),
	_ratesCmd(ORB_ID(vehicle_rates_setpoint), 20, 0, &getSubscriptions()),
	_pos(ORB_ID(vehicle_global_position), 20, 0, &getSubscriptions()),
	_missionCmd(ORB_ID(position_setpoint_triplet), 20, 0, &getSubscriptions()),
	_manual(ORB_ID(manual_control_setpoint), 20, 0, &getSubscriptions()),
	_status(ORB_ID(vehicle_status), 20, 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
	// publications
	_actuators(ORB_ID(actuator_controls_0), -1, &getPublications())
{
}

BlockUorbEnabledAutopilot::~BlockUorbEnabledAutopilot() {};

BlockANCLOuterLoop::BlockANCLOuterLoop(SuperBlock *parent, const char *name):SuperBlock(parent,name),
	// subscriptions
	//_att(ORB_ID(vehicle_attitude),20,0,&getSubscriptions()),
	_img_moments(ORB_ID(img_moments),20,0,&getSubscriptions()),
	_img_point(ORB_ID(img_point),20,0,&getSubscriptions()),
	_img_line(ORB_ID(img_line),20,0,&getSubscriptions()),
	_status(ORB_ID(vehicle_status), 20, 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
	_pos(ORB_ID(vehicle_local_position), 20, 0, &getSubscriptions()),
	//publications
	_att_sp(ORB_ID(vehicle_secondary_attitude_setpoint),-1,&getPublications())
{}

BlockANCLOuterLoop::~BlockANCLOuterLoop() {};

BlockANCLInnerLoop::BlockANCLInnerLoop(SuperBlock *parent, const char *name):SuperBlock(parent,name),
	// subscriptions
	_att(ORB_ID(vehicle_attitude),20.0,0,&getSubscriptions()),
	_attCmd(ORB_ID(vehicle_attitude_setpoint),20.0,0,&getSubscriptions()),
	_att2Cmd(ORB_ID(vehicle_secondary_attitude_setpoint),20.0,0,&getSubscriptions()),
	_status(ORB_ID(vehicle_status),20.0,0,&getSubscriptions()),
	_param_update(ORB_ID(parameter_update),20.0,0,&getSubscriptions()),
	//publications
	_control_sp(ORB_ID(vehicle_secondary_control_setpoint),-1,&getPublications())
{}

BlockANCLInnerLoop::~BlockANCLInnerLoop() {};

BlockANCLLoop::BlockANCLLoop(SuperBlock *parent, const char *name):SuperBlock(parent,name),
	// subscriptions
	//_att(ORB_ID(vehicle_attitude),20,0,&getSubscriptions()),
	_img_moments(ORB_ID(img_moments),20,0,&getSubscriptions()),
	_img_point(ORB_ID(img_point),20,0,&getSubscriptions()),
	_img_line(ORB_ID(img_line),20,0,&getSubscriptions()),
	_status(ORB_ID(vehicle_status), 20, 0, &getSubscriptions()),
	_param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
	_pos(ORB_ID(vehicle_local_position), 20, 0, &getSubscriptions()),
	_att(ORB_ID(vehicle_attitude),20.0,0,&getSubscriptions()),
	//publications
	_att_sp(ORB_ID(vehicle_secondary_attitude_setpoint),-1,&getPublications()),
	_control_sp(ORB_ID(vehicle_secondary_control_setpoint),-1,&getPublications())
{}

BlockANCLLoop::~BlockANCLLoop() {};





BlockDOBIBSLoop::BlockDOBIBSLoop(SuperBlock *parent, const char *name):SuperBlock(parent,name),
        // subscriptions
		
		//_vel_sp_LIN(ORB_ID(vehicle_global_velocity_setpoint), 20, 0, &getSubscriptions()),
		_vicon(ORB_ID(vicon), 20, 0, &getSubscriptions()),
		_ctrl_state(ORB_ID(control_state), 20, 0, &getSubscriptions()),
        _status(ORB_ID(vehicle_status), 20, 0, &getSubscriptions()),
        _param_update(ORB_ID(parameter_update), 1000, 0, &getSubscriptions()), // limit to 1 Hz
        _pos(ORB_ID(vehicle_local_position), 20, 0, &getSubscriptions()),
        _att(ORB_ID(vehicle_attitude),20.0,0,&getSubscriptions()),
        //publications
        _att_sp(ORB_ID(vehicle_secondary_attitude_setpoint),-1,&getPublications()),
        _control_sp(ORB_ID(vehicle_secondary_control_setpoint),-1,&getPublications())
        //_actuators(ORB_ID(actuator_controls),-1, &getPublications())
{}

BlockDOBIBSLoop::~BlockDOBIBSLoop() {};
}// namespace control
