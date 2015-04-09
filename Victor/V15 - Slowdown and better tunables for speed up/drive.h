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
#include "light_sensor.h"
#include "motors.h"
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
 * lightsense_CLK				5
 *
 * SEMAPHORES
 * Sema_lightsense
 *
 */
/* Robot locomotion states */
#define NUM_ARRAYS					6
#define NUM_SENSORS					3

#define DRIVESTATE_IDLE				0x0
#define DRIVESTATE_FWD				0x1
#define DRIVESTATE_REV				0x2
#define DRIVESTATE_TURNCW			0x3
#define DRIVESTATE_TURNCCW			0x4
#define DRIVESTATE_TURNAROUND		0x5
#define DRIVESTATE_STRAFELEFT		0x6
#define DRIVESTATE_STRAFERIGHT		0x7
#define DRIVESTATE_CALIBRATE		0x8
#define LINEDETECT_NULLS			0x9
#define LINEDETECT_BLACK			0xA
#define FRONTLINEDETECT_NULLS		0xB
#define FRONTLINEDETECT_BLACK		0xC
#define FRONTLINEDETECT_SLOWDOWN	0xD

/*
 * PID Controller Values
 *
 * note:
 * The gains are actually inversed,
 * when relating to error, due to the
 * abnormally large error values.
 */
#define KP							2//50
#define KI							100//100
#define KD							1000

/*	Filter for line sensors */
#define NUM_SAMPLES					3//3 is good too

/* Lightsensor PID Reference */
#define LIGHTSENSEREF				100			// Reference to the light sensor following
#define	CALIBRATIONSF_NUM			3			// Safety factor for lightsense compare - Tells if on white surface - Numerator coefficient
#define	CALIBRATIONSF_DEN			2			// Denominator Coefficient
#define CALIBRATION_SCALING_FACTOR	2			// Scaling factor for calibration thresholds
#define FRONTLINEDETECT_CUTFACTOR	2			// Factor that we cut down DC if we see an intersection coming up

/*	Safety Factor Gains	*/
#define LINEDETECT_K_DEN			28// 12 + 8sf inch squares
#define LINEDETECT_K_NUM			1// 1 inch black line
#define LINEDETECT_K_DEN_2			12//15
#define LINEDETECT_K_NUM_2			1// 1 inch black line
#define LINEDETECT_K_DEN_3			100//
#define LINEDETECT_K_NUM_3			1// TBD
#define SAMP_PRD					2 //ms

/* Ramp down/up constants and tunables */
#define DELTA_T						100			// The target change in time for the slowdown period
#define TARGET_DC					25			// Ending duty cycle after slowdown
#define FIXPNTCF					1000		// Fixed point math coffactor
#define RAMPUPCF					2			// A higher number represents a slower rampup speed

/* min and max indexes and constants */
#define i_MIN						0
#define i_MAX						1
#define NULLS						0
#define BLACKS						1
#define CALIBRATION_NUM_SAMP		400

/* PID U limits */
#define U_MAX						100

void PIDError(uint32_t periods[NUM_ARRAYS][NUM_SENSORS], int16_t error[NUM_ARRAYS][NUM_SENSORS], uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2]);
//*****************************************************************************
//
//! PIDError - calculates error for the light sensor
//!
//! \param periods - The measured periods
//!
//! \param error - The error array for the result
//!
//! \param minmax - The array that holds the minimum and max values
//!
//! \return none
//
//*****************************************************************************
void ThreshCompare(uint32_t raw_data[NUM_ARRAYS][NUM_SENSORS], uint32_t thresh[NUM_ARRAYS][NUM_SENSORS], uint8_t * result);
//*****************************************************************************
//
//! ThreshCompare - checks if periods of lightsensors are greater
//! than their respective thresholds, and if so, assigns a value to reflect this
//!
//! \param raw_data - The measured periods
//!
//! \param thresh - A pointer to the threshold arrays
//!
//! \return uint8_t result - an 8 bit packed representing the sensors
//! current reading
//
//*****************************************************************************
void Calibration_Get(uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2], uint32_t sample[NUM_ARRAYS][NUM_SENSORS]);
//*****************************************************************************
//
//! calibration_minmax - checks if sample is greater than max of samples in
//! argument minmax_hold, or less than, and stores the value
//!
//! \param minmax_hold - The measured holder for respective mins/maxes
//!
//! \param sample - values sampled via line sensors
//
//*****************************************************************************
void Calibrate_Sensors(void);
//*****************************************************************************
//
//! Calibrate_Sensors - calibrates the lightsensor by changing states
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
int32_t PIDController(int16_t * err, int32_t u);
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
void Detect_Adjacent(uint8_t * readings, uint16_t reads[2][2], uint8_t index0, uint8_t index1, uint8_t * iterations);
//*****************************************************************************
//
//! Detect_Adjacent - This function finds horizontal lines and increments a counter
//!
//! \param uint32_t * readings - The readings from the light sensor
//!
//! \param uint16_t * reads - arrays to hold the amount of reads
//!
//! \param uint8_t index0 - the index for an adjacent sensor
//!
//! \param uint8_t index1 - index for another adjacent sensor
//!
//! \param uint8_t * iterations - A pointer to the number of iterations done
//!
//*****************************************************************************
void Detect_Intersection(uint8_t * val, uint16_t reads[2][2], uint8_t * iterations, uint8_t index_0, uint8_t index_1, uint8_t * flags);
//*****************************************************************************
//
//! Detect_Intersection - This function detects intersections by using the front
//! rear linesensors
//!
//! \param int32_t val - values of the light sensors after threshold compare
//!
//! \param int8_t reads - The array for holding the number of reads
//!
//! \param uint8_t * iterations - A pointer to the number of iterations done
//!
//! \param uint8_t index_0 - An index to the sensor 0
//!
//! \param uint8_t index_1 - And index to the sensor 1
//!
//! \param uint8_t flags - A pointer to flags to check if we read past both sensors
//!
//*****************************************************************************
void Check_Iterations(int32_t * u_fwd, int32_t * u_back, uint8_t * iterations, uint8_t moves);
//*****************************************************************************
//
//! Check_Iterations - This function finds horizontal lines and increments a counter
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
void ThresholdApproximation(uint32_t thresh[NUM_ARRAYS][NUM_SENSORS], uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2]);
//*****************************************************************************
//
//! ThresholdApproximation - This function finds the calibrated threshold
//!
//! uint16_t thresh - This is an array with the thresholds for compares
//!
//!	uint32_t minmax - This is an array holding the mins and maxes of each sensor
//!
//*****************************************************************************
void Detect_Front(uint8_t val, uint16_t * reads, uint8_t iterations, uint8_t moves);
//*****************************************************************************
//
//! Detect_Intersection - This function detects intersections by using the front sensor
//!
//! \param int32_t val - The value of the light sensor in the direction you are moving
//!
//! \param int8_t reads - The array for holding the number of reads
//!
//! \param uint8_t iterations - The number of iterations done
//!
//! \param uint8_t moves - The number of moves we need to do
//!
//*****************************************************************************
#endif /* DRIVE_H_ */
