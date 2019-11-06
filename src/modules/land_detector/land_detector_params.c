/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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
 * @file land_detector.c
 * Land detector algorithm parameters.
 *
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

/**
 * Multicopter max climb rate
 *
 * Maximum vertical velocity allowed in the landed state (m/s up and down)
 *
 * @unit m/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_Z_VEL_MAX, 0.70f);

/**
 * Multicopter max horizontal velocity
 *
 * Maximum horizontal velocity allowed in the landed state (m/s)
 *
 * @unit m/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_XY_VEL_MAX, 1.50f);

/**
 * Multicopter max rotation
 *
 * Maximum allowed angular velocity around each axis allowed in the landed state.
 *
 * @unit deg/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_ROT_MAX, 20.0f);

/**
 * Multicopter specific force threshold
 *
 * Multicopter threshold on the specific force measured by accelerometers in m/s^2 for free-fall detection
 *
 * @unit m/s^2
 * @min 0.1
 * @max 10
 * @decimal 2
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_FFALL_THR, 2.0f);

/**
 * Multicopter free-fall trigger time
 *
 * Seconds (decimal) that freefall conditions have to met before triggering a freefall.
 * Minimal value is limited by LAND_DETECTOR_UPDATE_RATE=50Hz in landDetector.h
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_FFALL_TTRI, 0.3);

/**
 * Fixedwing max horizontal velocity
 *
 * Maximum horizontal velocity allowed in the landed state (m/s)
 *
 * @unit m/s
 * @min 0.5
 * @max 10
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VEL_XY_MAX, 5.0f);

/**
 * Fixedwing max climb rate
 *
 * Maximum vertical velocity allowed in the landed state (m/s up and down)
 *
 * @unit m/s
 * @min 5
 * @max 20
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VEL_Z_MAX, 10.0f);

/**
 * Fixedwing max short-term velocity
 *
 * Maximum velocity integral in flight direction allowed in the landed state (m/s)
 *
 * @unit m/s
 * @min 2
 * @max 10
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VELI_MAX, 4.0f);

/**
 * Airspeed max
 *
 * Maximum airspeed allowed in the landed state (m/s)
 *
 * @unit m/s
 * @min 4
 * @max 20
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_AIRSPD_MAX, 8.00f);
