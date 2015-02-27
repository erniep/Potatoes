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
 * timer5A_ISR					92
 *
 * SEMAPHORES
 * Sema_lightsense_calibrate
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
void calibrate_lightsensor(void);

#endif /* CALIBRATION_H_ */
