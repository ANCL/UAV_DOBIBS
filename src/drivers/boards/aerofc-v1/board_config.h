/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * @author David Sidrane <david_s5@nscdg.com>
 * @author Lorenz Meier <lorenz@px4.io>
 *
 * AEROFC_V1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

#define UDID_START		0x1FFF7A10

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs
 *
 * PC4     BLUE_LED                  D4 Blue LED cathode
 * PC5     RED_LED                   D5 Red LED cathode
*/
#define GPIO_LED1              (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED2              (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN10)
#define GPIO_BLUE_LED GPIO_LED1
#define GPIO_RED_LED  GPIO_LED2

#define GPIO_SENSORS_POWER		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)

/*
 * I2C busses
 *
 * Peripheral   Port     Signal Name               CONN
 * I2C1_SDA     PB9     I2C1_SDA                  J2-4,9,16,21 mpu6050, U4 MS6507
 * I2C1_SDL     PB8     I2C1_SCL                  J2-3,10,15,22 mpu6050, U4 MS6507
 *
 * I2C2_SDA     PB11    Sonar Echo/I2C_SDA        JP2-31,32
 * I2C2_SDL     PB10    Sonar Trig/I2C_SCL        JP2-29,30
 *
 * I2C3_SDA     PC9     COMPASS_I2C3_SDA          JP1-27,28
 * I2C3_SDL     PA8     COMPASS_I2C3_SCL          JP1-25,26
 *
 */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_ONBOARD		3

#define PX4_I2C_OBDEV_HMC5883	0x1E

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 * PC0     VOLTAGE                   JP2-13,14          - 1.84 @16.66  1.67 @15.12 Scale 0.1105
 *
 */
#define ADC_CHANNELS 0

/*
 * ADC defines just to not break sensors.cpp build, battery voltage and current
 * will be read in another way in future.
 */
#define ADC_BATTERY_VOLTAGE_CHANNEL    0
#define ADC_BATTERY_CURRENT_CHANNEL    ((uint8_t)(-1))

#define DIRECT_PWM_OUTPUT_CHANNELS	1
#define BOARD_HAS_PWM	0

#define BOARD_FMU_GPIO_TAB {{0, 0, 0}}

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

#define GPS_DEFAULT_UART_PORT	"/dev/ttyS2"

/* RC Serial port
 */
#define RC_SERIAL_PORT		"/dev/ttyS3"
#define INVERT_RC_INPUT(_s)		while(0)

/* High-resolution timer
 */
#define HRT_TIMER		3	/* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL	4	/* use capture/compare channel */

#define	BOARD_NAME "AEROFC_V1"

#define GPIO_SPI_CS_MPU6500		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPIDEV_MPU			1

#define  FLASH_BASED_PARAMS

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

#define board_spi_reset(ms)
#define board_peripheral_reset(ms)

extern void stm32_usbinitialize(void);

/************************************************************************************
 * Name: board_sdio_initialize
 *
 * Description:
 *   Called to configure SDIO.
 *
 ************************************************************************************/

extern int board_sdio_initialize(void);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif


/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the tap-v1 board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage);

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the power button
 ****************************************************************************/

bool board_pwr_button_down(void);

/****************************************************************************
 * Name: board_pwr
 *
 * Description:
 *   Called to turn on or off the TAP
 *
 ****************************************************************************/

void board_pwr(bool on_not_off);

#endif /* __ASSEMBLY__ */

__END_DECLS
