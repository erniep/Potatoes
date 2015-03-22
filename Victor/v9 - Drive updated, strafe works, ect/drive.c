/*
 * drive.c
 *
 *  Created on: Feb 3, 2015
 *      Author: Wicho
 */
#include "drive.h"
/* Global Variables */
uint8_t dutycycle = 0;											// Initial Duty Cycle
uint8_t drivestate = 0;											// Drive Task State
uint8_t linedetect_state = LINEDETECT_NULLS_0;					// Line detection state counter
/* Externs */
extern uint8_t num_moves;										// Num moves for current state
extern uint32_t lightsnsr_thresh[4];							// Threshold compare values
extern uint32_t lightsnsr_prd_1[3];								// Holds values for periods measured
extern uint32_t lightsnsr_prd_2[3];								// Holds values for periods measured
extern uint32_t lightsnsr_prd_3[3];								// Holds values for periods measured
extern uint32_t lightsnsr_prd_4[3];								// Holds values for periods measured
/* Functions */
void Robot_drive_task(void)
{
	Robot_PWM_init();											// Initialize motors
	uint16_t side_reads[2] = {0,0};								// 0th element is nulls read, and 1st element is non-zero reads
	uint8_t num_iterations = 0;									// Iterations counter
	int16_t e0[3];												// Error arrays
	int16_t e1[3];
	int16_t e2[3];
	int16_t e3[3];
	int32_t u;													// Control efforts
	int32_t u2;
	uint32_t lightsnsr_val_1;									// Values read after threshold compares
	uint32_t lightsnsr_val_2;
	uint32_t lightsnsr_val_3;
	uint32_t lightsnsr_val_4;

	//rv_motors(40,1);
	tr_motors(50, 1); //Test :)
	//servo_tangent();

	while(TRUE)
	{
		// Pend on semaphore - Light Sensor values updated
		Semaphore_pend(Sema_lightsense, BIOS_WAIT_FOREVER);

		// compare threshold values, update errors, and threshold values
		lightsnsr_val_1 = lightsense_thresh_check(lightsnsr_prd_1, lightsnsr_thresh, 1);
		lightsnsr_val_2 = lightsense_thresh_check(lightsnsr_prd_2, lightsnsr_thresh, 2);
		lightsnsr_val_3 = lightsense_thresh_check(lightsnsr_prd_3, lightsnsr_thresh, 3);
		lightsnsr_val_4 = lightsense_thresh_check(lightsnsr_prd_4, lightsnsr_thresh, 4);
		PIDError(lightsnsr_prd_1, e0);
		PIDError(lightsnsr_prd_2, e1);
		PIDError(lightsnsr_prd_3, e2);
		PIDError(lightsnsr_prd_4, e3);
		switch(drivestate)									// State machine
		{
		case DRIVESTATE_IDLE:
			//Park state needs to be done
			// Do nothing :)
			break;
		case DRIVESTATE_FWD:
			linedetection(lightsnsr_val_3, lightsnsr_val_4, side_reads, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			u = PIDController(e0, u);
			u2 = PIDController(e1, u2);
			fwd_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_REV:
			linedetection(lightsnsr_val_3, lightsnsr_val_4, side_reads, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			u = PIDController(e0, u);
			u2 = PIDController(e1, u2);
			rev_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_STRAFELEFT:
			linedetection(lightsnsr_val_1, lightsnsr_val_2, side_reads, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			u = PIDController(e2, u);
			u2 = PIDController(e3, u2);
			tl_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_STRAFERIGHT:
			linedetection(lightsnsr_val_1, lightsnsr_val_2, side_reads, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			u = PIDController(e2, u);
			u2 = PIDController(e3, u2);
			tr_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_TURNCW:
		/*	intersectiondetect(lightsnsr_val, side_reads, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);*/
			break;
		case DRIVESTATE_TURNCCW:
			//intersectiondetect(lightsnsr_val, side_reads, &num_iterations);
			//linecheck(&u, &u2, &num_iterations, num_moves);
			break;
		}
	}
}
/*
 * PID CONTROLLER HELPER FUNCTIONS
 */
void PIDError(uint32_t * periods, int16_t * error)
{
	error[2] = error[1];
	error[1] = error[0];
	// Get new error value by taking weighted average
	int32_t new_error = ((0 * periods[0] + 1000 * periods[1] + 2000 * periods[2]) / (periods[0] + periods[1] + periods[2]));
	error[0] = (int16_t)new_error - LIGHTSENSEREF;								// Subtract the reference for the error value
}
int32_t PIDController(int16_t * err, int32_t u)
{
	int32_t result = u + (err[0] - err[1])/KP + ((err[0] - err[1])/2)/KI + 2*(err[0] - 2*err[1] + err[2])/KD;
	// Limit control effort
	uint32_t mag_u = abs(u);
	if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
	return result;
}
/*
 * LINE DETECTION HELPER FUNCTIONS
 */
void linedetection(uint32_t val_1, uint32_t val_2, uint16_t * reads, uint8_t * iterations)
{
	switch(linedetect_state)
	{
	case LINEDETECT_NULLS_0:
		if((val_1==0) & (val_2==0)) reads[0]++;
		else linedetect_state = LINEDETECT_BLACK_0;
		break;
	case LINEDETECT_BLACK_0:
		if(val_1 | val_2)
		{
			reads[1]++;
			if(reads[0])
			{
				if(reads[1] > ((reads[0] * LINEDETECT_K_NUM) / LINEDETECT_K_DEN))
				{
				(*iterations)++;
				clearReadCNT(reads);
				}
			}
			else
			{
				// If the sensor did not read a null first, go back to first state
				// This is a control for if the robot begins on a black line
				linedetect_state = LINEDETECT_NULLS_0;
				clearReadCNT(reads);
			}
		}
		else
		{
			linedetect_state = LINEDETECT_NULLS_0;
			clearReadCNT(reads);
		}
		break;
	}
}
void intersectiondetect(uint32_t * lightsense_vals, uint16_t * reads, uint8_t * iterations)
{
	switch(linedetect_state)
	{
	case LINEDETECT_NULLS_0:
		//if((lightsense_vals[0]==0) & (lightsense_vals[1]==0) & (lightsense_vals[2]==0) & (lightsense_vals[3]==0)) reads[0]++;
		if((lightsense_vals[0]==0)) reads[0]++;
		else linedetect_state = LINEDETECT_BLACK_0;
		break;
	case LINEDETECT_BLACK_0:
		if(lightsense_vals[0])
		//if(lightsense_vals[0] | (lightsense_vals[1] | lightsense_vals[2]) | lightsense_vals[3])
		{
			reads[1]++;
			if(reads[0])
			{
				if(reads[1] > ((reads[0] * LINEDETECT_K_NUM) / LINEDETECT_K_DEN))
				{
				(*iterations)++;
				clearReadCNT(reads);
				}
			}
			else
			{
				// If the sensor did not read a null first, go back to first state
				// This is a control for if the robot begins on a black line
				linedetect_state = LINEDETECT_NULLS_0;
				clearReadCNT(reads);
			}
		}
		else
		{
			linedetect_state = LINEDETECT_NULLS_0;
			clearReadCNT(reads);
		}
		break;
	}
}
void linecheck(int32_t * u_fwd, int32_t * u_back, uint8_t * iterations, uint8_t moves)
{
	if((*iterations) >= moves)
	{

		stop_motors(0);										// Change state and stop motors
		(*u_fwd) = 0;
		(*u_back) = 0;
		(*iterations) = 0;

	}
}
void clearReadCNT(uint16_t * reads)
{
	// Reset read counters
	reads[0] = 0;
	reads[1] = 0;
}
uint32_t lightsense_thresh_check(uint32_t * raw_data, uint32_t * thresh, uint8_t lightsense_index)
{
	uint8_t n;												// For loop index
	uint8_t result = 0;										// Result placeholder
	for(n=0; n<3; n++)
	{
		if(raw_data[n]>thresh[lightsense_index-1]) result |= (1<<n);
	}
	return result;
}
/*
 * MISC HELPER FUNCTIONS
 */
int32_t abs(int32_t val)
{
	return val > 0 ? val : -val;
}
