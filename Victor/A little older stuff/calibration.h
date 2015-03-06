/*
 * calibration.h
 *
 *  Created on: Feb 25, 2015
 *      Author: Wicho
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
/* Standard Libraries */
/* XDCtools Header files */
/* BIOS Header files */
#include "light_sensor.h"
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 * timer5A_ISR					108
 * timer5B_ISR					109
 *
 * SEMAPHORES
 * Sema_lightsense_calibrate
 * Sema_lightsense
 *
 */
/* TivaC Header files */


//
//			  Robot
//-----------------------------
//	  		LIGHTSNSR1
//LIGHTSNSR3 		LIGHTSNSR4
//	   		LIGHTSNSR2
//-----------------------------

#define timer5A_prd 10
#define	timer5B_prd 1
#define k_num		5
#define k_den		3
void calibrate_lightsensor(void);
void calibrate_lightsensor_init(void);
void timer5A_ISR(void);

#endif /* CALIBRATION_H_ */
