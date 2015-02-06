/*
 * drive.c
 *
 *  Created on: Feb 3, 2015
 *      Author: Wicho
 */
#include "drive.h"
/* Global Variables */
uint8_t dutycycle = 90;											// Initial Duty Cycle
uint8_t drivestate = 0;											// Drive Task State
/* Externs */
extern uint32_t lightsnsr_val1;
extern uint32_t lightsnsr_val2;
extern uint32_t lightsnsr_val3;
extern uint32_t lightsnsr_val4;
extern uint8_t num_moves;
/* Functions */
void Robot_drive_task(void)
{
	Robot_PWM_init();											// Initialize motors
	int32_t pid_coeff[3];										// PID Coefficients
	int8_t e[3];
	int32_t u;
	int32_t mag_u;
	pid_terms_calc(KP, KI, KD, F_SAMP, pid_coeff);
	uint8_t blk_cnt1 = 0;										// Consecutive BLK read
	uint8_t blk_cnt2 = 0;
	uint8_t blk_cnt3 = 0;
	uint8_t blk_cnt4 = 0;
	uint8_t num_iterations = 0;
	while(TRUE)
	{
		// Pend on semaphore - Light Sensor values updated
		Semaphore_pend(Sema_lightsense, BIOS_WAIT_FOREVER);
		switch(drivestate)
		{
		case DRIVESTATE_IDLE:
			// Do nothing :)
			break;
		case DRIVESTATE_FWD:
			// First Check for horizontal lines
			if((lightsnsr_val4 == 0x7) || (lightsnsr_val4 == 0x2))
				{
					blk_cnt4++;									// Increment BLK tape read counter
				}
			else blk_cnt4 = 0;
			if((lightsnsr_val3 == 0x7) || (lightsnsr_val3 == 0x2))
				{
					blk_cnt3++;									// Increment BLK tape read counter
				}
			else blk_cnt3 = 0;
			if((blk_cnt3 >= BLK_TAPE_READ) && (blk_cnt4 >= BLK_TAPE_READ))
			{
				blk_cnt3 = 0;
				blk_cnt4 = 0;
				if(++num_iterations >= num_moves)
				{
					u = 0;
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future parking algorithm for CW turns
					// Future support for accelerometer/dist check :)
				}
			}
			// PID Algo
			error_calc(lightsnsr_val1, e);
			u = u + (pid_coeff[0] * e[0] + pid_coeff[1] * e[1] + pid_coeff[2] * e[2]) / fxdpnt_coeff;
			mag_u = abs(u);
			if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
			fwd_pid(u,dutycycle);
			break;
		case DRIVESTATE_REV:
			// First Check for horizontal lines
			if((lightsnsr_val4 == 0x7) || (lightsnsr_val4 == 0x2))
				{
					blk_cnt4++;									// Increment BLK tape read counter
				}
			else blk_cnt4 = 0;
			if((lightsnsr_val3 == 0x7) || (lightsnsr_val3 == 0x2))
				{
					blk_cnt3++;									// Increment BLK tape read counter
				}
			else blk_cnt3 = 0;
			if((blk_cnt3 >= BLK_TAPE_READ) && (blk_cnt4 >= BLK_TAPE_READ))
			{
				blk_cnt3 = 0;
				blk_cnt4 = 0;
				if(++num_iterations >= num_moves)
				{
					u = 0;
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future parking algorithm for CW turns
					// Future support for accelerometer/dist check :)
				}
			}
			// PID
			error_calc(lightsnsr_val2, e);
			u = u + (pid_coeff[0] * e[0] + pid_coeff[1] * e[1] + pid_coeff[2] * e[2]) / fxdpnt_coeff;
			mag_u = abs(u);
			if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
			rev_pid(u,dutycycle);
			break;
		case DRIVESTATE_STRAFELEFT:
			// First Check for horizontal lines
			if((lightsnsr_val1 == 0x7) || (lightsnsr_val1 == 0x2))
				{
					blk_cnt1++;									// Increment BLK tape read counter
				}
			else blk_cnt1 = 0;
			if((lightsnsr_val2 == 0x7) || (lightsnsr_val2 == 0x2))
				{
					blk_cnt2++;									// Increment BLK tape read counter
				}
			else blk_cnt2 = 0;
			if((blk_cnt1 >= BLK_TAPE_READ) && (blk_cnt2 >= BLK_TAPE_READ))
			{
				blk_cnt1 = 0;
				blk_cnt2 = 0;
				if(++num_iterations >= num_moves)
				{
					u = 0;
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future parking algorithm for CW turns
					// Future support for accelerometer/dist check :)
				}
			}
			// PID
			error_calc(lightsnsr_val3, e);
			u = u + (pid_coeff[0] * e[0] + pid_coeff[1] * e[1] + pid_coeff[2] * e[2]) / fxdpnt_coeff;
			mag_u = abs(u);
			if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
			tl_pid(u,dutycycle);
			break;
		case DRIVESTATE_STRAFERIGHT:
			// First Check for horizontal lines
			if((lightsnsr_val1 == 0x7) || (lightsnsr_val1 == 0x2))
				{
					blk_cnt1++;									// Increment BLK tape read counter
				}
			else blk_cnt1 = 0;
			if((lightsnsr_val2 == 0x7) || (lightsnsr_val2 == 0x2))
				{
					blk_cnt2++;									// Increment BLK tape read counter
				}
			else blk_cnt2 = 0;
			if((blk_cnt1 >= BLK_TAPE_READ) && (blk_cnt2 >= BLK_TAPE_READ))
			{
				blk_cnt1 = 0;
				blk_cnt2 = 0;
				if(++num_iterations >= num_moves)
				{
					u = 0;
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future support for accelerometer/dist check :)
				}
			}
			// PID
			error_calc(lightsnsr_val4, e);
			u = u + (pid_coeff[0] * e[0] + pid_coeff[1] * e[1] + pid_coeff[2] * e[2]) / fxdpnt_coeff;
			mag_u = abs(u);
			if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
			tr_pid(u,dutycycle);
			break;
		case DRIVESTATE_TURNCW:
			if((lightsnsr_val4 == 0x7) || (lightsnsr_val4 == 0x2))
				{
					blk_cnt4++;									// Increment BLK tape read counter
				}
			else blk_cnt4 = 0;
			if((lightsnsr_val3 == 0x7) || (lightsnsr_val3 == 0x2))
				{
					blk_cnt3++;									// Increment BLK tape read counter
				}
			else blk_cnt3 = 0;
			if((lightsnsr_val2 == 0x7) || (lightsnsr_val2 == 0x2))
				{
					blk_cnt2++;									// Increment BLK tape read counter
				}
			else blk_cnt2 = 0;
			if((lightsnsr_val1 == 0x7) || (lightsnsr_val1 == 0x2))
				{
					blk_cnt1++;									// Increment BLK tape read counter
				}
			if(((blk_cnt3 >= BLK_TAPE_READ) && (blk_cnt4 >= BLK_TAPE_READ)) || ((blk_cnt1 >= BLK_TAPE_READ) && (blk_cnt2 >= BLK_TAPE_READ)))
			{
				blk_cnt1 = 0;
				blk_cnt2 = 0;
				blk_cnt3 = 0;
				blk_cnt4 = 0;
				if(++num_iterations >= num_moves)
				{
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future parking algorithm for CW turns
					// Future support for accelerometer check :)
				}
			}
			break;
		case DRIVESTATE_TURNCCW:
			if((lightsnsr_val4 == 0x7) || (lightsnsr_val4 == 0x2))
				{
					blk_cnt4++;									// Increment BLK tape read counter
				}
			else blk_cnt4 = 0;
			if((lightsnsr_val3 == 0x7) || (lightsnsr_val3 == 0x2))
				{
					blk_cnt3++;									// Increment BLK tape read counter
				}
			else blk_cnt3 = 0;
			if((lightsnsr_val2 == 0x7) || (lightsnsr_val2 == 0x2))
				{
					blk_cnt2++;									// Increment BLK tape read counter
				}
			else blk_cnt2 = 0;
			if((lightsnsr_val1 == 0x7) || (lightsnsr_val1 == 0x2))
				{
					blk_cnt1++;									// Increment BLK tape read counter
				}
			if(((blk_cnt3 >= BLK_TAPE_READ) && (blk_cnt4 >= BLK_TAPE_READ)) || ((blk_cnt1 >= BLK_TAPE_READ) && (blk_cnt2 >= BLK_TAPE_READ)))
			{
				blk_cnt1 = 0;
				blk_cnt2 = 0;
				blk_cnt3 = 0;
				blk_cnt4 = 0;
				if(++num_iterations >= num_moves)
				{
					num_iterations = 0;
					stop_motors(dutycycle);						// Change state and stop motors
					// Future parking algorithm for CW turns
					// Future check for accelerometer :)
				}
			}
			break;
		}
	}
}
void pid_terms_calc(uint8_t kp_term, uint8_t ki_term, uint8_t kd_term, uint8_t sampling_frq, int32_t * pid_vals)
{
	pid_vals[0] = ((fxdpnt_coeff * kp_term) + ((fxdpnt_coeff * ki_term) / (2 * sampling_frq)) + (fxdpnt_coeff * (kd_term * sampling_frq)));
	pid_vals[1] = (((fxdpnt_coeff * ki_term) / (2 * sampling_frq)) - (2 * (kd_term * fxdpnt_coeff) * sampling_frq) - (fxdpnt_coeff * kp_term));
	pid_vals[2] = ((fxdpnt_coeff * kd_term) * sampling_frq);
}
void error_calc(uint32_t lightsnsr, int8_t * error)
{
	error[2] = error[1];
	error[1] = error[0];
	switch(lightsnsr)
	{
	case 0x7:
		error[0] = 0;
		break;
		case 0x6:
		error[0] = 1;
		break;
	case 0x5:
		error[0] = 0;
		break;
	case 0x4:
		error[0] = 3;
		break;
	case 0x3:
		error[0] = -1;
		break;
	case 0x2:
		error[0] =  0;
		break;
	case 0x1:
		error[0] = -3;
		break;
	case 0x0:
		if(error[1] > 0) error[0] = 5;
		else if(error[1] < 0) error[0] = -5;
		else if(error[2] > 0) error[0] = 5;
		else if(error[2] < 0) error[0] = -5;
		break;
	}
}
int32_t abs(int32_t val)
{
	return val > 0 ? val : -val;
}
