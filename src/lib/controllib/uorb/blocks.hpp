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
 * @file uorb_blocks.h
 *
 * uorb block library code
 */

#pragma once

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/img_moments.h>
#include <uORB/topics/img_point.h>
#include <uORB/topics/img_line.h>
#include <uORB/topics/vehicle_secondary_attitude_setpoint.h>
#include <uORB/topics/vehicle_secondary_control_setpoint.h>

//#include <uORB/topics/vehicle_image_attitude_setpoint.h>
#include <uORB/topics/vicon.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <poll.h>

#include "../blocks.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

namespace control
{

/**
 * Waypoint Guidance block
 */
class __EXPORT BlockWaypointGuidance : public SuperBlock
{
private:
	BlockLimitSym _xtYawLimit;
	BlockP _xt2Yaw;
	float _psiCmd;
public:
	BlockWaypointGuidance(SuperBlock *parent, const char *name);
	virtual ~BlockWaypointGuidance();
	void update(const vehicle_global_position_s &pos,
		    const vehicle_attitude_s &att,
		    const position_setpoint_s &missionCmd,
		    const position_setpoint_s &lastMissionCmd);
	float getPsiCmd() { return _psiCmd; }
};

/**
 * UorbEnabledAutopilot
 */
class __EXPORT BlockUorbEnabledAutopilot : public SuperBlock
{
protected:
	// subscriptions
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _attCmd;
	uORB::Subscription<vehicle_rates_setpoint_s> _ratesCmd;
	uORB::Subscription<vehicle_global_position_s> _pos;
	uORB::Subscription<position_setpoint_triplet_s> _missionCmd;
	uORB::Subscription<manual_control_setpoint_s> _manual;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	// publications
	uORB::Publication<actuator_controls_s> _actuators;
public:
	BlockUorbEnabledAutopilot(SuperBlock *parent, const char *name);
	virtual ~BlockUorbEnabledAutopilot();
};

/**
 * ANCL Outer Loop with uORB
 */

class __EXPORT BlockANCLOuterLoop : public SuperBlock
{
protected:
	//subscriptions
	//uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<img_moments_s> _img_moments;
	uORB::Subscription<img_point_s> _img_point;
	uORB::Subscription<img_line_s> _img_line;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Subscription<vehicle_local_position_s> _pos;
	//publications
	uORB::Publication<vehicle_secondary_attitude_setpoint_s> _att_sp;
public:
	BlockANCLOuterLoop(SuperBlock *parent, const char *name);
	virtual ~BlockANCLOuterLoop();
};

/**
 * ANCL Inner Loop with uORB
 */

class __EXPORT BlockANCLInnerLoop : public SuperBlock
{
protected:
	//subscriptions
	uORB::Subscription<vehicle_attitude_s> _att;
	uORB::Subscription<vehicle_attitude_setpoint_s> _attCmd;
	uORB::Subscription<vehicle_secondary_attitude_setpoint_s> _att2Cmd;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	//publications
	uORB::Publication<vehicle_secondary_control_setpoint_s> _control_sp;
public:
	BlockANCLInnerLoop(SuperBlock *parent, const char *name);
	virtual ~BlockANCLInnerLoop();
};

/**
 * ANCL Single Loop with uORB
 */

class __EXPORT BlockANCLLoop : public SuperBlock
{
protected:
	//subscriptions
	uORB::Subscription<img_moments_s> _img_moments;
	uORB::Subscription<img_point_s> _img_point;
	uORB::Subscription<img_line_s> _img_line;
	uORB::Subscription<vehicle_status_s> _status;
	uORB::Subscription<parameter_update_s> _param_update;
	uORB::Subscription<vehicle_local_position_s> _pos;
	uORB::Subscription<vehicle_attitude_s> _att;
	//publications
	uORB::Publication<vehicle_secondary_attitude_setpoint_s> _att_sp;
	uORB::Publication<vehicle_secondary_control_setpoint_s> _control_sp;
public:
	BlockANCLLoop(SuperBlock *parent, const char *name);
	virtual ~BlockANCLLoop();

};
/**
 * yunzhi NewPID Outer Loop with uORB
 */


class __EXPORT BlockNewPIDOuterLoop : public SuperBlock
{
protected:
        //subscriptions
        //uORB::Subscription<vehicle_attitude_s> _att;

        //uORB::Subscription<img_moments_s> _img_moments;
        //uORB::Subscription<img_point_s> _img_point;
        //uORB::Subscription<img_line_s> _img_line;
		

		//uORB::Subscription<vehicle_global_velocity_setpoint_s> _vel_sp_LIN;
		uORB::Subscription<vicon_s> _vicon;
        uORB::Subscription<vehicle_status_s> _status;
		uORB::Subscription<parameter_update_s> _param_update;
		
		

        uORB::Subscription<vehicle_local_position_s> _pos;
        //publications
        uORB::Publication<vehicle_secondary_attitude_setpoint_s> _att_sp;
public:
        BlockNewPIDOuterLoop(SuperBlock *parent, const char *name);
        virtual ~BlockNewPIDOuterLoop();
};

class __EXPORT BlockINLoop : public SuperBlock
{
protected:

		uORB::Subscription<vicon_s> _vicon;
		uORB::Subscription<vehicle_attitude_setpoint_s> _v_att_sp;
		uORB::Subscription<control_state_s>_ctrl_state;

        uORB::Subscription<vehicle_status_s> _status;
		uORB::Subscription<parameter_update_s> _param_update;

		//publications
		uORB::Publication<actuator_controls_s> _actuators;
        uORB::Publication<vehicle_secondary_attitude_setpoint_s> _att_sp;
public:
        BlockINLoop(SuperBlock *parent, const char *name);
        virtual ~BlockINLoop();
};
	
	
class __EXPORT BlockTASKLoop : public SuperBlock
{
protected:

		uORB::Subscription<vicon_s> _vicon;
		uORB::Subscription<control_state_s>_ctrl_state;
        uORB::Subscription<vehicle_status_s> _status;
		uORB::Subscription<parameter_update_s> _param_update;
        uORB::Subscription<vehicle_local_position_s> _pos; //LOCAL_POSITION_NED???? only x,y,z,vx,vy,vz are sent
        uORB::Subscription<vehicle_attitude_s> _att; //ATTITUDE. r p y speed
                                                     //ATTITUDE .q -> euler
        //publications
        uORB::Publication<vehicle_secondary_attitude_setpoint_s> _att_sp;//ATTITUDE_TARGET2
        uORB::Publication<vehicle_secondary_control_setpoint_s> _control_sp; //ACTUATOR_CONTROL_TARGET2
        //uORB::Publication<actuator_controls_s> _actuators;
public:
        BlockTASKLoop(SuperBlock *parent, const char *name);
        virtual ~BlockTASKLoop();

};


} // namespace control

