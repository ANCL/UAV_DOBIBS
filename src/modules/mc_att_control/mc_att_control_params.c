/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll time constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @unit s
 * @min 0.15
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_TC, 0.2f);

/**
 * Pitch time constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @unit s
 * @min 0.15
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_TC, 0.2f);

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 8
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.05f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.05f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw feed forward
 *
 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_FF, 0.5f);

/**
 * Max roll rate
 *
 * Limit for roll rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * A value of significantly over 120 degrees per second can already lead to mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Max yaw rate in auto mode
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode,
 * to avoid large control output and mixer saturation. A value of significantly
 * over 60 degrees per second can already lead to mixer saturation.
 * A value of 30 degrees / second is recommended to avoid very audible twitches.
 *
 * @unit deg/s
 * @min 0.0
 * @max 120.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRAUTO_MAX, 45.0f);

/**
 * Max acro roll rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX, 360.0f);

/**
 * Max acro pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX, 360.0f);

/**
 * Max acro yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX, 360.0f);

/**
 * Threshold for Rattitude mode
 *
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_RATT_TH, 1.0f);

/**
 * Threshold for Throttle PID Attenuation (TPA)
 *
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch P gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_TPA_BREAK, 1.0f);

/**
 * Slope for Throttle PID Attenuation (TPA)
 *
 * Rate at which to attenuate roll/pitch P gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - slope*(abs(throttle)-breakpoint))
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_TPA_SLOPE, 1.0f);


/**
 * Whether to scale outputs by battery power level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);
