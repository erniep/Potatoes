/*
 * light_sensor.h
 *
 *  Created on: Nov 16, 2014
 *      Author: Wicho
 */

#ifndef LIGHT_SENSOR_H_
#define LIGHT_SENSOR_H_

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Board Header files */
#include "inc.h"
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <inc/hw_gpio.h>
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 * timer2A_ISR					39
 * timer2B_ISR					40
 * timer3A_ISR					51
 * timer4A_ISR					86
 * timer4B_ISR					87
 *
 * TASK
 * TASK_NAME					Priority
 * Robot_lightsnsr_task			1
 *
 * CLOCK (main clock 10ms)
 * Clock func name				ticks
 * lightsense_CLK				1
 *
 * SEMAPHORES
 * Sema_lightsense_f
 * Sema_lightsense
 * Sema_lightsense_calibrate
 *
 */
//
//			  Robot
//-----------------------------
//	  		LIGHTSNSR1
//LIGHTSNSR3 		LIGHTSNSR4
//	   		LIGHTSNSR2
//-----------------------------
//

/* Light Sensor Pin Values */
#define LIGHTSNSR1_BASE				GPIO_PORTA_BASE
#define LIGHTSNSR2_BASE				GPIO_PORTC_BASE
#define LIGHTSNSR3A_BASE			GPIO_PORTD_BASE
#define LIGHTSNSR3B_BASE			GPIO_PORTF_BASE
#define LIGHTSNSR4A_BASE			GPIO_PORTB_BASE
#define LIGHTSNSR4B_BASE			GPIO_PORTC_BASE
#define LIGHTSNSR1					GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
#define LIGHTSNSR1_1				GPIO_PIN_2
#define LIGHTSNSR1_2				GPIO_PIN_3
#define LIGHTSNSR1_3				GPIO_PIN_4
#define LIGHTSNSR2					GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
#define LIGHTSNSR2_1				GPIO_PIN_5
#define LIGHTSNSR2_2				GPIO_PIN_6
#define LIGHTSNSR2_3				GPIO_PIN_7
#define LIGHTSNSR3A					GPIO_PIN_6 | GPIO_PIN_7
#define LIGHTSNSR3B					GPIO_PIN_4
#define LIGHTSNSR3A_1				GPIO_PIN_6
#define LIGHTSNSR3A_2				GPIO_PIN_7
#define LIGHTSNSR3B_3				GPIO_PIN_4
#define LIGHTSNSR4A					GPIO_PIN_2 | GPIO_PIN_3 // CHANGE THIS - ACCELEROMETER STUFF
#define LIGHTSNSR4B					GPIO_PIN_4
#define LIGHTSNSR4A_1				GPIO_PIN_2
#define LIGHTSNSR4A_2				GPIO_PIN_3
#define LIGHTSNSR4B_3				GPIO_PIN_4

/* Threshold values for light sensor */
#define fixedpoint_microsec_coeff 	1000000

/* Timer periods */
#define timer2A_prd 				10
/*#define thresh3						(135 + (390-135)/2)/2//min 135, max 390
#define thresh4                     (215 + (900-165)/2)/2//min 164, max 900
#define thresh1 					(160 + (820-160)/2)/2//min - 160, max 820
#define thresh2						(220 + (912-220)/2)/2// min 220, max 912 */
#define thresh3						(135 + (390-135)/2)/2//min 135, max 390
#define thresh4                     (215 + (900-165)/2)/2//min 164, max 900
#define thresh1 					(110 + (820-160)/2)/2//min - 160, max 820
#define thresh2						(220 + (912-220)/2)/2// min 220, max 912

/* function prototypes */
void sample_lightsense(void);

#endif
