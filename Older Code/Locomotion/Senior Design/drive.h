/*
 * drive.h
 *
 *  Created on: Feb 3, 2015
 *      Author: Wicho
 */

#ifndef DRIVE_H_
#define DRIVE_H_

/* Standard Libraries */
#include <stdint.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Board Header Files */
#include "motors.h"
#include "light_sensor.h"

/* Robot locomotion states */
#define DRIVESTATE_IDLE				0x0
#define DRIVESTATE_FWD				0x1
#define DRIVESTATE_REV				0x2
#define DRIVESTATE_TURNCW			0x3
#define DRIVESTATE_TURNCCW			0x4
#define DRIVESTATE_TURNAROUND		0x5
#define DRIVESTATE_STRAFELEFT		0x6
#define DRIVESTATE_STRAFERIGHT		0x7

/* PID Controller Values */
#define KP							1
#define KI							0
#define KD							0
#define F_SAMP						100

/* Black Line consecutive reads */
#define BLK_TAPE_READ				5

/* PID U limits */
#define U_MAX						50

/* Fixed Point Coefficient Values */
#define fxdpnt_coeff				1000				// To be adjusted to be tuned for optimal resolution on fixed-point math

//*****************************************************************************
void pid_terms_calc(uint8_t kp_term, uint8_t ki_term, uint8_t kd_term, uint8_t sampling_frq, int32_t * pid_vals);
//*****************************************************************************
//
//! pid_terms_calc - calculates the PID terms at start, to be used for the controller
//! This is useful as it decreses the amount of instruction cycles per caclulation,
//! thus we could increase the sampling frequency as we see fit. The coefficients a,b,c
//! are multiplied by the coefficient fxdpnt_coeff, as defined in the header file, This is to
//! allow extra resolution in the fixed point math as a controller effort could be very small
//!
//! \param kp_term is the proportional gain
//!
//! \param ki_term is the integral gain
//!
//! \param kd_term is the derivative gain
//!
//! \param sampling_frq is the sampling frequency of the light sensor
//!
//! \param pid_vals is the a,b,c coefficients to the PID controller.
//!
//! \return none
//
//*****************************************************************************
void error_calc(uint32_t lightsense, int8_t * error);
//*****************************************************************************
//
//! error_calc - calculates error for the light sensor
//!
//! \param lightsense - The lightsensor 3 bit value
//!
//! \param error - is the result storage location
//!
//! \return none
//
//*****************************************************************************
int abs(int32_t val);
//*****************************************************************************
//
//! abs - Takes absolute value
//!
//! \param val is the value to be converted
//!
//! \return absolute value of parameter val
//
//*****************************************************************************
#endif /* DRIVE_H_ */
