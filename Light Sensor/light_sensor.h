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
#include "motors.h"
#include <inc/hw_ints.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
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
#define LIGHTSNSR4A_BASE			GPIO_PORTF_BASE
#define LIGHTSNSR4B_BASE			GPIO_PORTB_BASE
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
#define LIGHTSNSR4A					GPIO_PIN_2 | GPIO_PIN_3
#define LIGHTSNSR4B					GPIO_PIN_3
#define LIGHTSNSR4A_1				GPIO_PIN_2
#define LIGHTSNSR4A_2				GPIO_PIN_3
#define LIGHTSNSR4B_3				GPIO_PIN_3

/* Threshold values for light sensor */
#define fixedpoint_microsec_coeff 	1000000

/* Timer periods */
#define timer2B_prd 				100
#define timer2A_prd 				10


/* function prototypes */
