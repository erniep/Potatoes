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
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 *
 * TASK
 * TASK_NAME					Priority
 * Robot_drive_task				2
 *
 * CLOCK (main clock 10ms)
 * Clock func name				ticks
 * lightsense_CLK				1
 *
 * SEMAPHORES
 * Sema_lightsense
 *
 */
/* Robot locomotion states */
#define DRIVESTATE_IDLE				0x0
#define DRIVESTATE_FWD				0x1
#define DRIVESTATE_REV				0x2
#define DRIVESTATE_TURNCW			0x3
#define DRIVESTATE_TURNCCW			0x4
#define DRIVESTATE_TURNAROUND		0x5
#define DRIVESTATE_STRAFELEFT		0x6
#define DRIVESTATE_STRAFERIGHT		0x7
#define LINEDETECT_NULLS_0			0x8
#define LINEDETECT_BLACK_0			0x9
#define LINEDETECT_NULLS_1			0xA

/*
 * PID Controller Values
 *
 * note:
 * The gains are actually inversed,
 * when relating to error, due to the
 * abnormally large error values.
 */
#define KP							20
#define KI							10000
#define KD							10000

/* Lightsensor PID Reference */
#define LIGHTSENSEREF				1000

/*	Safety Factor Gains	*/
#define LINEDETECT_K_DEN			12
#define LINEDETECT_K_NUM			1
#define LINEDETECT_K_DEN_2			10//15
#define LINEDETECT_K_NUM_2			2
/* Black Line consecutive reads */
#define BLK_TAPE_READ				10

/* PID U limits */
#define U_MAX						100

void PIDError(uint32_t * periods, int16_t * error);
//*****************************************************************************
//
//! PIDError - calculates error for the light sensor
//!
//! \param periods - The measured periods
//!
//! \param error - The error array for the result
//!
//! \return none
//
//*****************************************************************************
uint32_t lightsense_thresh_check(uint32_t * raw_data, uint32_t * thresh, uint8_t lightsense_index);
//*****************************************************************************
//
//! lightsense_thresh_check - checks if periods of lightsensors are greater
//! than their respective thresholds, and if so, assigns a value to reflect this
//!
//! \param raw_data - The measured periods
//!
//! \param thresh - An array representing thresholds for each sensor
//!
//! \param lightsense_index - An index for which lightsensor
//!
//! \return uint8_t result - an 8 bit packed representing the sensors
//! current reading
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
int32_t PIDController(int16_t * e, int32_t u);
//*****************************************************************************
//
//! PIDController - This function finds u givent the PID coefficients and the errors
//!
//! \param int16_t * e - The pointer to the error array that is used in calculation
//!
//! \param int32_t u - The previous control effort value
//!
//! \param int32_t * pid_coeff - The PID coefficients of the controller
//!
//! \return new control effort
//
//*****************************************************************************
void linedetection(uint32_t val_1, uint32_t val_2, uint16_t * reads, uint8_t * iterations);
//*****************************************************************************
//
//! linedetection - This function finds horizontal lines and increments a counter
//!
//! \param int32_t val_1 - threshold compare value of 1st sensor
//!
//! \param uint32_t val_2 - threshold compare value of 2nd sensor
//!
//! \param uint16_t * reads - pointer to array for blk, and white reads
//!
//! \param uint8_t * iterations - A pointer to the number of iterations done
//!
//*****************************************************************************
void intersectiondetect(uint32_t * lightsense_vals, uint16_t * reads, uint8_t * iterations);
//*****************************************************************************
//
//! linedetection - This function finds intersections and increments a counter
//!
//! \param int32_t * lightsense_vals - Array of the lightsensor values
//!
//! \param int8_t cnt_vals - The count values stored in an array
//!
//! \param uint8_t * iterations - A pointer to the number of iterations done
//!
//*****************************************************************************
void linecheck(int32_t * u_fwd, int32_t * u_back, uint8_t * iterations, uint8_t moves);
//*****************************************************************************
//
//! linecheck - This function finds horizontal lines and increments a counter
//!
//! \param int32_t * u_fwd - The control effort of the forward sensor
//!
//! \param int32_t * u_back - The control effort of the reverse sensor
//!
//! \param uint8_t moves - The number of moves intended
//!
//! \param uint8_t * iterations - A pointer to the number of iterations done
//!
//*****************************************************************************
void clearReadCNT(uint16_t * reads);
//*****************************************************************************
//
//! clearReadCNT - This function clears the read counters
//!
//! uint16_t * reads - This is a pointer to the read counters
//!
//*****************************************************************************
#endif /* DRIVE_H_ */
