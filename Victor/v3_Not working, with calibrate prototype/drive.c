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
/* Externs */
extern uint32_t lightsnsr_val[4];
extern uint8_t num_moves;
/* Functions */
void Robot_drive_task(void)
{
	Robot_PWM_init();											// Initialize motors
	int32_t pid_coeff[3];										// PID Coefficients
	int8_t e[3];
	int8_t e2[3];
	int32_t u;
	int32_t u2;
	pid_terms_calc(KP, KI, KD, F_SAMP, pid_coeff);
	uint8_t blk_cnt[4];											// Consecutive BLK read counters
	uint8_t num_iterations = 0;

	fw_motors(80, 3); //Test :)
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
			linedetection(lightsnsr_val, blk_cnt, 0, 1, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			error_calc(lightsnsr_val[0], e);
			error_calc(lightsnsr_val[1], e2);
			u = findu(e, u, pid_coeff);
			u2 = findu(e2, u2, pid_coeff);
			fwd_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_REV:
			linedetection(lightsnsr_val, blk_cnt, 1, 0, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			error_calc(lightsnsr_val[1], e);
			error_calc(lightsnsr_val[0], e2);
			u = findu(e, u, pid_coeff);
			u2 = findu(e2, u2, pid_coeff);
			rev_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_STRAFELEFT:
			linedetection(lightsnsr_val, blk_cnt, 2, 3, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			error_calc(lightsnsr_val[2], e);
			error_calc(lightsnsr_val[3], e2);
			u = findu(e, u, pid_coeff);
			u2 = findu(e2, u2, pid_coeff);
			tl_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_STRAFERIGHT:
			linedetection(lightsnsr_val, blk_cnt, 3, 2, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			error_calc(lightsnsr_val[3], e);
			error_calc(lightsnsr_val[2], e2);
			u = findu(e, u, pid_coeff);
			u2 = findu(e2, u2, pid_coeff);
			tl_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_TURNCW:
			intersectiondetect(lightsnsr_val, blk_cnt, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
			break;
		case DRIVESTATE_TURNCCW:
			intersectiondetect(lightsnsr_val, blk_cnt, &num_iterations);
			linecheck(&u, &u2, &num_iterations, num_moves);
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
int32_t findu(int8_t * e, int32_t u, int32_t * pid_coeff)
{
	u = u + (pid_coeff[0] * e[0] + pid_coeff[1] * e[1] + pid_coeff[2] * e[2]) / fxdpnt_coeff;
	uint32_t mag_u = abs(u);
	if(mag_u >= U_MAX) u = (u * U_MAX) / mag_u;
	return u;
}
void linedetection(uint32_t * lightsense_vals, uint8_t * cnt_vals, uint8_t fwd_index, uint8_t rev_index, uint8_t * iterations)
{
	if((lightsense_vals[fwd_index] == 0x7) | (lightsense_vals[fwd_index] == 0x2))
		{
			cnt_vals[fwd_index]++;										// Increment BLK tape read counter
		}
	else cnt_vals[fwd_index] = 0;
	if((lightsense_vals[rev_index] == 0x7) | (lightsense_vals[rev_index] == 0x2))
		{
		cnt_vals[rev_index]++;										// Increment BLK tape read counter
		}
	else cnt_vals[rev_index] = 0;
	if((cnt_vals[fwd_index] >= BLK_TAPE_READ) & (cnt_vals[rev_index] >= BLK_TAPE_READ))
	{
		cnt_vals[fwd_index] = 0;
		cnt_vals[rev_index] = 0;
		(*iterations)++;
	}
}

void intersectiondetect(uint32_t * lightsense_vals, uint8_t * cnt_vals, uint8_t * iterations)
{
	if((lightsense_vals[0] == 0x7) | (lightsense_vals[0] == 0x2))
		{
			cnt_vals[0]++;									// Increment BLK tape read counter
		}
	else cnt_vals[0] = 0;
	if((lightsense_vals[1] == 0x7) | (lightsense_vals[1] == 0x2))
		{
			cnt_vals[1]++;									// Increment BLK tape read counter
		}
	else cnt_vals[1] = 0;
	if((lightsense_vals[2] == 0x7) | (lightsense_vals[2] == 0x2))
		{
			cnt_vals[2]++;									// Increment BLK tape read counter
		}
	else cnt_vals[2] = 0;
	if((lightsense_vals[3] == 0x7) | (lightsense_vals[3] == 0x2))
		{
			cnt_vals[3]++;									// Increment BLK tape read counter
		}
	else cnt_vals[3] = 0;
	if(((cnt_vals[0] >= BLK_TAPE_READ) & (cnt_vals[1] >= BLK_TAPE_READ)) | ((cnt_vals[2] >= BLK_TAPE_READ) & (cnt_vals[3] >= BLK_TAPE_READ)))
	{
		cnt_vals[0] = 0;									// Clear BLK tape read counters
		cnt_vals[1] = 0;
		cnt_vals[2] = 0;
		cnt_vals[3] = 0;
		*(iterations)++;									// Increment Iteration Counter
	}
}
void linecheck(int32_t * u_fwd, int32_t * u_back, uint8_t * iterations, uint8_t moves)
{
	if((*iterations) >= moves)
	{
		(*u_fwd) = 0;
		(*u_back) = 0;
		(*iterations) = 0;
		stop_motors(0);										// Change state and stop motors
	}
}
