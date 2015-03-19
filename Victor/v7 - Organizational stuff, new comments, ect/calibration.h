/*
 * calibration.h
 *
 *  Created on: Feb 25, 2015
 *      Author: Wicho
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include "light_sensor.h"
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 * gpio_A_ISR					16
 * gpio_B_ISR					17
 * gpio_C_ISR					18
 * gpio_D_ISR					19
 * gpio_F_ISR					46
 * Wtimer0A_ISR					110
 *
 * SEMAPHORES
 * Sema_lightsense_calibrate
 * Sema_lightsense
 *
 * CLOCKS
 *
 */
//
//			  Robot
//-----------------------------
//	  		LIGHTSNSR1
//LIGHTSNSR3 		LIGHTSNSR4
//	   		LIGHTSNSR2
//-----------------------------
#define LIGHTSNSR1_INT				GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4
#define LIGHTSNSR2_INT				GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7
#define LIGHTSNSR3A_INT				GPIO_INT_PIN_6 | GPIO_INT_PIN_7
#define LIGHTSNSR3B_INT				GPIO_INT_PIN_4
#define LIGHTSNSR4A_INT				GPIO_INT_PIN_2 | GPIO_INT_PIN_3
#define LIGHTSNSR4B_INT				GPIO_INT_PIN_4
#define LIGHTSNSR1_1_INT			GPIO_INT_PIN_2
#define LIGHTSNSR1_2_INT			GPIO_INT_PIN_3
#define LIGHTSNSR1_3_INT			GPIO_INT_PIN_4
#define LIGHTSNSR2_1_INT			GPIO_INT_PIN_5
#define LIGHTSNSR2_2_INT			GPIO_INT_PIN_6
#define LIGHTSNSR2_3_INT			GPIO_INT_PIN_7
#define LIGHTSNSR3A_1_INT			GPIO_INT_PIN_6
#define LIGHTSNSR3A_2_INT			GPIO_INT_PIN_7
#define LIGHTSNSR3B_3_INT			GPIO_INT_PIN_4
#define LIGHTSNSR4A_1_INT			GPIO_INT_PIN_2
#define LIGHTSNSR4A_2_INT			GPIO_INT_PIN_3
#define LIGHTSNSR4B_3_INT			GPIO_INT_PIN_4

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
#define LIGHTSNSR4A					GPIO_PIN_2 | GPIO_PIN_3
#define LIGHTSNSR4B					GPIO_PIN_4
#define LIGHTSNSR4A_1				GPIO_PIN_2
#define LIGHTSNSR4A_2				GPIO_PIN_3
#define LIGHTSNSR4B_3				GPIO_PIN_4

/* Fixed point values for light sensor */
#define fixedpoint_microsec_coeff 	1000000
#define fixedpoint_millisec_coeff	1000

/* Timer periods */
#define timer5A_prd 				10//us
#define Wtimer0_prd					10//ms
#define thresh3						(135 + (390-135)/2)/2//min 135, max 390
#define thresh4                     (165 + (900-165)/2)/2//min 164, max 900
#define thresh1 					(160 + (820-160)/2)/2 //min 160, max 820
#define thresh2						(220 + (912-220)/2)/2// min 220, max 912

#define min							0
#define max							1

void calibrate_lightsensor(void);
void lightsensorcharge(void);
void min_and_max(uint8_t index_sensor, uint8_t index_pin, uint32_t delta_val, uint32_t (*delta)[3][2]);

#endif /* CALIBRATION_H_ */
