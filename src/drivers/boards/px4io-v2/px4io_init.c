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
 * @file px4iov2_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>

#include <stm32.h>
#include "board_config.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{

	/* configure GPIOs */

	/* LEDS - default to off */
	px4_arch_configgpio(GPIO_LED1);
	px4_arch_configgpio(GPIO_LED2);
	px4_arch_configgpio(GPIO_LED3);
	px4_arch_configgpio(GPIO_LED4);

	px4_arch_configgpio(GPIO_BTN_SAFETY);

	/* spektrum power enable is active high - enable it by default */
	px4_arch_configgpio(GPIO_SPEKTRUM_PWR_EN);

	px4_arch_configgpio(GPIO_SERVO_FAULT_DETECT);

	/* RSSI inputs */
	px4_arch_configgpio(GPIO_TIM_RSSI); /* xxx alternate function */
	px4_arch_configgpio(GPIO_ADC_RSSI);

	/* servo rail voltage */
	px4_arch_configgpio(GPIO_ADC_VSERVO);

	px4_arch_configgpio(GPIO_SBUS_INPUT); /* xxx alternate function */
	px4_arch_configgpio(GPIO_SBUS_OUTPUT);

	/* sbus output enable is active low - disable it by default */
	px4_arch_gpiowrite(GPIO_SBUS_OENABLE, true);
	px4_arch_configgpio(GPIO_SBUS_OENABLE);

	px4_arch_configgpio(GPIO_PPM); /* xxx alternate function */

	px4_arch_gpiowrite(GPIO_PWM1, true);
	px4_arch_configgpio(GPIO_PWM1);

	px4_arch_gpiowrite(GPIO_PWM2, true);
	px4_arch_configgpio(GPIO_PWM2);

	px4_arch_gpiowrite(GPIO_PWM3, true);
	px4_arch_configgpio(GPIO_PWM3);

	px4_arch_gpiowrite(GPIO_PWM4, true);
	px4_arch_configgpio(GPIO_PWM4);

	px4_arch_gpiowrite(GPIO_PWM5, true);
	px4_arch_configgpio(GPIO_PWM5);

	px4_arch_gpiowrite(GPIO_PWM6, true);
	px4_arch_configgpio(GPIO_PWM6);

	px4_arch_gpiowrite(GPIO_PWM7, true);
	px4_arch_configgpio(GPIO_PWM7);

	px4_arch_gpiowrite(GPIO_PWM8, true);
	px4_arch_configgpio(GPIO_PWM8);
}
