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
#include <inc/hw_types.h>
#include <inc/hw_timer.h>
#include <driverlib/gpio.h>
#include <inc/hw_gpio.h>

#include "drive.h"
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 * timer2A_ISR					39
 * gpio stuff
 *
 * TASK
 * TASK_NAME					Priority
 * Robot_lightsnsr_task			1
 * median_filter_task			3
 *
 * CLOCK (main clock 10ms)
 * Clock func name				ticks
 * lightsense_CLK				5
 *
 * SEMAPHORES
 * Sema_lightsense_f
 * Sema_lightsense
 * Sema_lightsense_filter
 *
 */
//
//			  Robot
//-----------------------------
//			LIGHTSNSR5 (PD6)
//	  		LIGHTSNSR1
//LIGHTSNSR3 (PD7)		LIGHTSNSR4 (PB3)
//	   		LIGHTSNSR2
//			LIGHTSNSR6 (PB2)
//-----------------------------
//
/* Light Sensor Pin Values */
#define LIGHTSNSR1_BASE				GPIO_PORTA_BASE
#define LIGHTSNSR2_BASE				GPIO_PORTC_BASE
#define LIGHTSNSR3_BASE				GPIO_PORTD_BASE
#define LIGHTSNSR4_BASE				GPIO_PORTB_BASE
#define LIGHTSNSR5_BASE				GPIO_PORTD_BASE
#define LIGHTSNSR6_BASE				GPIO_PORTB_BASE
#define LIGHTSNSR1					GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 			// Array 0
#define LIGHTSNSR1_1				GPIO_PIN_2
#define LIGHTSNSR1_2				GPIO_PIN_3
#define LIGHTSNSR1_3				GPIO_PIN_4
#define LIGHTSNSR2					GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7			// Array 1
#define LIGHTSNSR2_1				GPIO_PIN_5
#define LIGHTSNSR2_2				GPIO_PIN_6
#define LIGHTSNSR2_3				GPIO_PIN_7
#define LIGHTSNSR3					GPIO_PIN_7
#define LIGHTSNSR4					GPIO_PIN_3
#define LIGHTSNSR5					GPIO_PIN_6
#define LIGHTSNSR6					GPIO_PIN_2

/* GPIO interrupt masks */
#define LIGHTSNSR1_INT				GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4
#define LIGHTSNSR2_INT				GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7
#define LIGHTSNSR3_INT				GPIO_INT_PIN_7
#define LIGHTSNSR4_INT				GPIO_INT_PIN_3
#define LIGHTSNSR5_INT				GPIO_INT_PIN_6
#define LIGHTSNSR6_INT				GPIO_INT_PIN_2

#define LIGHTSNSR1_1_INT			GPIO_INT_PIN_2
#define LIGHTSNSR1_2_INT			GPIO_INT_PIN_3
#define LIGHTSNSR1_3_INT			GPIO_INT_PIN_4
#define LIGHTSNSR2_1_INT			GPIO_INT_PIN_5
#define LIGHTSNSR2_2_INT			GPIO_INT_PIN_6
#define LIGHTSNSR2_3_INT			GPIO_INT_PIN_7

/* Bit defines */
#define BIT0						(1<<0)
#define BIT1						(1<<1)
#define BIT2						(1<<2)
#define BIT3						(1<<3)
#define BIT4						(1<<4)
#define BIT5						(1<<5)
#define BIT6						(1<<6)
#define BIT7						(1<<7)
#define BIT8						(1<<8)
#define BIT9						(1<<9)
#define BIT10						(1<<10)
#define BIT11						(1<<11)


/* Threshold values for light sensor */
#define fixedpoint_microsec_coeff 	1000000
#define fixedpoint_count_to_time	80
#define MAX_PERIOD					1500               // Saturates the max reading for each sensor

/* Clock Values */
#define Clock 						80000000
#define CHARGE_PRD					10				//us

/* function prototypes */
void sort_arr(uint32_t * arr);
int partition(uint32_t a[], int l, int r);
void quicksort(uint32_t a[], int l, int r);
/*
void quickSort(uint32_t a[], uint8_t l, uint8_t r);
uint8_t partition(uint32_t a[], uint8_t l, uint8_t r);
*/
#endif
