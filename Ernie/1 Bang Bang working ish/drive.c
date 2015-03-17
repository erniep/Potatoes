/*
 * drive.c
 *
 *  Created on: Feb 3, 2015
 *      Author: Wicho
 */
#include "drive.h"
#include "inc.h"

/* Global Variables */
uint8_t dutycycle = 0;											// Initial Duty Cycle
uint8_t drivestate = 0;											// Drive Task State
uint8_t linedetect_state = LINEDETECT_NULLS_0;					// Line detection state counter

//BY ERNIE
int dir_1=1,
	dir_2=1,
	dir_3=1,
	dir_4=1;
uint32_t *front_sensor;
uint32_t *back_sensor;

/* Externs */
extern uint32_t lightsnsr_val[4];
extern uint8_t num_moves;
/* Functions */
void Robot_drive_task(void)
{
	int initialSpeed = 50;
	int motor1 = initialSpeed;
	int motor2 = initialSpeed;
	int motor3 = initialSpeed;
	int motor4 = initialSpeed;

	int FR0L1 = 0;	//which side of line
	int BR0L1 = 0;	//which side of line

	int speed8 = 80;
	int speed7 = 70;
	int speed6 = 60;
	int speed5 = 50;
	int speed4 = 40;
	int speed3 = 30;
	int speed2 = 20;

	front_sensor = &lightsnsr_val[0];
	back_sensor = &lightsnsr_val[1];

	Robot_PWM_init();
	while (1) {
		Semaphore_pend(Sema_lightsense, BIOS_WAIT_FOREVER);

		switch(*front_sensor & 0x07) {
			case WWW:
				//means on right side, go left			
				if (FR0L1 == 1) {	motor1 = speed8;	motor2 = speed2; }
			 	else {				motor1 = speed2;	motor2 = speed8; }
				break;
			case WWB:				motor1 = speed7;	motor2 = speed3;	FR0L1 = 1;
				break;
			case WBB:				motor1 = speed6;	motor2 = speed4;	FR0L1 = 1;
				break;
			case WBW:
			case BWB:
			case BBB:				motor1 = speed5;	motor2 = speed5;
				break;
			case BBW:				motor1 = speed4;	motor2 = speed6;	FR0L1 = 0;
				break;
			case BWW:				motor1 = speed3;	motor2 = speed7;	FR0L1 = 0;
				break;
		}

		switch(*back_sensor & 0x07) {
			case WWW:
				//means on right side, go left			
				if (BR0L1 == 1) {	motor3 = speed8;	motor4 = speed2; }
			 	else {				motor3 = speed2;	motor4 = speed8; }
				break;
			case WWB:				motor3 = speed7;	motor4 = speed3;	BR0L1 = 1;
				break;
			case WBB:				motor3 = speed6;	motor4 = speed4;	BR0L1 = 1;
				break;
			case WBW:
			case BWB:
			case BBB:				motor3 = speed5;	motor4 = speed5;
				break;
			case BBW:				motor3 = speed4;	motor4 = speed6;	BR0L1 = 0;
				break;
			case BWW:				motor3 = speed3;	motor4 = speed7;	BR0L1 = 0;
				break;
		}
		
		motorAll((motor1)*dir_1,(motor2)*dir_2,(motor3)*dir_3,(motor4)*dir_4);
		//motorAll(50,50,50,50);
	}//infinite while loop
}

void setDir(int dir, int value) {
	switch(dir) {
		case 1:
			dir_1 = value;
			break;
		case 2:
			dir_2 = value;
			break;
		case 3:
			dir_3 = value;
			break;
		case 4:
			dir_4 = value;
			break;
	}
}

void linedetection(uint32_t * lightsense_vals, uint16_t * reads, uint8_t fwd_index, uint8_t rev_index, uint8_t * iterations)
{
	switch(linedetect_state)
	{
	case LINEDETECT_NULLS_0:
		if((lightsense_vals[fwd_index]==0) & (lightsense_vals[rev_index]==0)) reads[0]++;
		else linedetect_state = LINEDETECT_BLACK_0;
		break;
	case LINEDETECT_BLACK_0:
		if((lightsense_vals[fwd_index]) | (lightsense_vals[rev_index]))
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
		if((lightsense_vals[0]==0) & (lightsense_vals[1]==0) & (lightsense_vals[2]==0) & (lightsense_vals[3]==0)) reads[0]++;
		else linedetect_state = LINEDETECT_BLACK_0;
		break;
	case LINEDETECT_BLACK_0:
		if(lightsense_vals[0] | (lightsense_vals[1] | lightsense_vals[2]) | lightsense_vals[3])
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
	int32_t temp;
	if((*iterations) >= moves)
	{
		/*ADDED BY ERNIE*/
		if (drivestate == DRIVESTATE_FWD) {
			//rv_motors(25, 1);
		} else {
			(*u_fwd) = 0;
			(*u_back) = 0;
			stop_motors(0);	// Change state and stop motors
		}
		/*END ADDED BY ERNIE*/
		(*iterations) = 0;
	}
}
void clearReadCNT(uint16_t * reads)
{
	// Reset read counters
	reads[0] = 0;
	reads[1] = 0;
}
