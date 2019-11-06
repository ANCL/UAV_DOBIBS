include(nuttx/px4_impl_nuttx)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-none-eabi.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	drivers/stm32
	drivers/stm32/adc
	drivers/stm32/tone_alarm
	drivers/led
	drivers/px4fmu
	drivers/px4io
	drivers/boards/px4fmu-v1
	drivers/ardrone_interface
	drivers/rgbled
	drivers/mpu6000
	drivers/lsm303d
	drivers/l3gd20
	drivers/hmc5883
	drivers/ms5611
	drivers/mb12xx
	drivers/sf0x
	drivers/ll40ls
	drivers/trone
	#drivers/gps
	#drivers/pwm_out_sim
	#drivers/hott
	#drivers/hott/hott_telemetry
	#drivers/hott/hott_sensors
	drivers/blinkm
	drivers/airspeed
	drivers/ets_airspeed
	drivers/meas_airspeed
	drivers/frsky_telemetry
	modules/sensors
	#drivers/mkblctrl
	drivers/px4flow
	drivers/camera_trigger

	#
	# System commands
	#
	systemcmds/bl_update
	systemcmds/mixer
	systemcmds/param
	systemcmds/perf
	systemcmds/pwm
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/top
	systemcmds/config
	systemcmds/nshterm
	systemcmds/mtd
	systemcmds/dumpfile
	systemcmds/ver

	#
	# General system control
	#
	modules/commander
	#modules/navigator
	modules/mavlink
	#modules/gpio_led
	#modules/land_detector
	modules/vicon

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	# Too high RAM usage due to static allocations
	# modules/attitude_estimator_ekf
	modules/attitude_estimator_q
	modules/ekf_att_pos_estimator
	modules/position_estimator_inav
	#modules/ekf2


	#
	# Logging
	#
	modules/sdlog2

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/controllib
	modules/uORB
	modules/dataman

	#
	# Libraries
	#
	#lib/mathlib/CMSIS
	lib/mathlib
	lib/mathlib/math/filter
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	platforms/nuttx

	# had to add for cmake, not sure why wasn't in original config
	platforms/common 
	platforms/nuttx/px4_layer


)

set(config_extra_builtin_cmds
	serdis
	sercon
	)

set(config_io_board
	px4io-v1
	)

set(config_extra_libs
	${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM4lf_math.a
	)

set(config_io_extra_libs
	#${CMAKE_SOURCE_DIR}/src/lib/mathlib/CMSIS/libarm_cortexM3l_math.a
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
