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
 * @file mc_TASK_params.c
 * Multicopter TASK controller parameters.
 *
 * @author yunzhi <yunzhi2@ualberta.ca>
 */


/**
 * Gain for image around image x axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_X_P, 0.35f);

/**
 * Gain for image around image y axis in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Y_P, 0.35f);

/**
 * Gain for image around image z axis  in Body frame (Camera Frame)
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.001
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Z_P, 0.25f);


/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_X_I, 0.14f);

/**
 * Integral gain for horizontal velocity error
 *
 * Non-zero value allows to resist wind.
 *
 * @min 0.0
 * @max 0.2
 * @decimal 3
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Y_I, 0.14f);

/**
 * Integral gain for vertical velocity error
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Z_I, 0.02f);

/**
 * Gain for yaw
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_YAW_P,0.6f);


/**
 * x integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_X_I_MAX, 0.2f);

/**
 * y integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Y_I_MAX, 0.2f);

/**
 * z integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_PID_Z_I_MAX, 0.035f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_X_D, 0.007f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.005
 * @max 0.1
 * @decimal 3  
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_Y_D, 0.01f);

/**
 * Differential gain for vertical velocity error
 *
 * @min 0.0
 * @max 0.1
 * @decimal 3
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_Z_D, 0.0f);

/**
 * Lowpass for x error derivative calculation
 *
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_X_D_LP, 0.1f);

/**
 * Lowpass for y error derivative calculation
 *
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_Y_D_LP, 0.1f);

/**
 * Lowpass for y error derivative calculation
 *
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_PID_Z_D_LP, 0.1f);





/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_POSX_P, 1.3f);

/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_POSY_P, 1.3f);

/**
 * Proportional gain for vertical position error
 *
 * @unit norm
 * @min 0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter TASK Control
 */

PARAM_DEFINE_FLOAT(TASK_POSZ_P, 1.0f);

/**
 * thrust required for gravity
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter  TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_G, 0.05f);

/**
 * yaw setpoint
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @group Multicopter  TASK Control
 */
PARAM_DEFINE_FLOAT(TASK_YAW_CONST, 0.0f);





//TODO: IN

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_R_P,0.15f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_P_P, 0.15f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_Y_P, 0.2f);


/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_R_I, 0.05f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_P_I, 0.05f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_Y_I, 0.1f);


/**
 * x integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_R_I_MAX, 0.035f);

/**
 * y integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_P_I_MAX, 0.035f);

/**
 * z integral Saturation
 *
 * @unit norm
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_Y_I_MAX, 0.035f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_A_R_D, 0.003f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_P_D, 0.003f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_A_Y_D, 0.0f);

/**
 * Lowpass for x error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_A_R_D_LP, 0.1f);

/**
 * Lowpass for y error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_A_P_D_LP, 0.1f);

/**
 * Lowpass for z error derivative calculation
 *
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_A_Y_D_LP, 0.1f);





/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 8
 * @decimal 2
 * @increment 0.1
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_RATE_ROLL_P, 6.5f);

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
 * @group Multicopter IN Control
 */

PARAM_DEFINE_FLOAT(TASK_RATE_PIT_P, 6.5f);

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
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_RATE_YAW_P, 2.8f);

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
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_RRATE_MAX, 220.0f);

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
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_PRATE_MAX, 220.0f);

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
 * @group Multicopter IN Control
 */
PARAM_DEFINE_FLOAT(TASK_YRATE_MAX, 200.0f);


/**
*  Derivative parameter
*/
PARAM_DEFINE_FLOAT(TASK_D_PARAM,0.01f);

/**
*  Integration parameter
*/
PARAM_DEFINE_FLOAT(TASK_I_PARAM,1.0f);

PARAM_DEFINE_FLOAT(TASK_I_PARAM_MAX,2.0f);

PARAM_DEFINE_FLOAT(TASK_D_PARAM_LP,0.1f);









