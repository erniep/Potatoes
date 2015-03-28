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
uint8_t linedetect_state = LINEDETECT_NULLS;					// Line detection state counter
uint8_t front_detect_state = FRONTLINEDETECT_NULLS;				// Front sensor detection counter

/* Externs */
extern uint8_t num_moves;										// Num moves for current state
extern uint32_t prd[NUM_ARRAYS][NUM_SENSORS];					// Output of median filter
/* Functions */
void Robot_drive_task(void)
{
	Robot_PWM_init();											// Initialize motors

	// Variables
	uint16_t side_reads[2][2] = {0};							// 0th element is nulls read, and 1st element is non-zero reads
	uint16_t front_reads[2] = {0};								// Same as above
	uint8_t sensor_flag[2] = {0};
	uint8_t slow_down_flag = 0;
	uint8_t num_iterations = 0;									// Iterations counter
	int16_t error[NUM_ARRAYS][NUM_SENSORS] = {0};				// Error arrays
	int32_t u = 0;												// Control efforts
	int32_t u2 = 0;
	uint8_t lightsnsr_val[NUM_ARRAYS]= {0};					// Values after threshold compare
	uint32_t lightsnsr_thresh[NUM_ARRAYS][NUM_SENSORS] = {0};	// Threshold compare values
	uint32_t calibration_minmax[NUM_ARRAYS][NUM_SENSORS][2] = {0};// Array to hold min/maxes for calibration
	uint8_t calibrationstate = 0;								// Calibration state machine holder
	uint16_t sample_counter = 0;								// calibration sample counter
	uint8_t ramp_up_counter = 0;								// DC ramp up counter
	uint8_t duty;												// SAMS FAVORITE WORD :p

	// Calibrate line sensors
	Calibrate_Sensors();

	uint8_t test = 1;
	//servo_tangent();

	while(TRUE)
	{
		// Pend on semaphore - Light Sensor values updated
		Semaphore_pend(Sema_lightsense, BIOS_WAIT_FOREVER);
		ThreshCompare(prd, lightsnsr_thresh, lightsnsr_val);//Assign values according to readings and threshold values
		PIDError(prd, error, calibration_minmax);				// Get error values
		// Driving state machine
		switch(drivestate)
		{
		case DRIVESTATE_IDLE:
			ramp_up_counter = 0;																// Reset ramp up counter
			slow_down_flag = 0;																	// Reset slow down flag
			//Park state needs to be done
			// Do nothing :)
			//test++;
			test^=1;
			if(test==0) fw_motors(60, 1);
			if(test==1) cw_motors(60, 1);
			break;
		case DRIVESTATE_FWD:
			Detect_Front(lightsnsr_val[0], front_reads, num_iterations, num_moves, &slow_down_flag);	// Is an intersection coming up? is so, cut the DC down
			Detect_Adjacent(lightsnsr_val, side_reads, 2, 3, &num_iterations);					// Are we at an intersection yet?
			Check_Iterations(&u, &u2, &num_iterations, num_moves);								// Check if we need to stop
			u = PIDController(error[0], u);														// Controller, error[0] is a pointer
			u2 = PIDController(error[1], u2);													// error[1] is a pointer
			if(dutycycle < (ramp_up_counter*2)) duty = dutycycle;								// ramp up the duty cycle
			else duty = 2*(++ramp_up_counter);
			fwd_pid(u, u2, duty);
			break;
		case DRIVESTATE_REV:
			Detect_Front(lightsnsr_val[1], front_reads, num_iterations, num_moves, &slow_down_flag);	// Is an intersection coming up? is so, cut the DC down
			Detect_Adjacent(lightsnsr_val, side_reads, 2, 3, &num_iterations);					// Are we at an intersection yet?
			Check_Iterations(&u, &u2, &num_iterations, num_moves);								// Check if we need to stop
			u = PIDController(error[0], u);														// Controller, error[0] is a pointer
			u2 = PIDController(error[1], u2);													// error[1] is a pointer
			if(dutycycle < (ramp_up_counter*2)) duty = dutycycle;								// ramp up the duty cycle
			else duty = 2*(++ramp_up_counter);
			rev_pid(u, u2, duty);
			break;
		case DRIVESTATE_STRAFELEFT:
			// Needs work
			Detect_Adjacent(lightsnsr_val, side_reads, 0, 1, &num_iterations);//Detect intersections
			Check_Iterations(&u, &u2, &num_iterations, num_moves);		// Check if we need to stop
			u = PIDController(error[2], u);						// Error[2] is an array
			u2 = PIDController(error[3], u2);					// Error[3] is an array
			tl_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_STRAFERIGHT:
			// Needs work
			Detect_Adjacent(lightsnsr_val, side_reads, 0, 1, &num_iterations);//Detect intersections
			Check_Iterations(&u, &u2, &num_iterations, num_moves);
			u = PIDController(error[2], u);
			u2 = PIDController(error[3], u2);
			tr_pid(u, u2, dutycycle);
			break;
		case DRIVESTATE_TURNCW:
			Detect_Intersection(lightsnsr_val, side_reads, &num_iterations, 0, 1, sensor_flag);
			Check_Iterations(&u, &u2, &num_iterations, num_moves);
			break;
		case DRIVESTATE_TURNCCW:
			Detect_Intersection(lightsnsr_val, side_reads, &num_iterations, 0, 1, sensor_flag);
			Check_Iterations(&u, &u2, &num_iterations, num_moves);
			break;
		case DRIVESTATE_CALIBRATE:
			sample_counter++;								// Increment sample counter
			Calibration_Get(calibration_minmax, prd);
			switch(calibrationstate)
			{
			case 0:
				cw_motors_openloop(50);
				if(sample_counter >= 2*CALIBRATION_NUM_SAMP) calibrationstate++;
				break;
			case 1:
				ccw_motors_openloop(50);
				if(sample_counter >= (6*CALIBRATION_NUM_SAMP)) calibrationstate++;
				break;
			case 2:
				cw_motors_openloop(50);
				if(sample_counter >= (8*CALIBRATION_NUM_SAMP))//4*CALIBRATION_NUM_SAMP))
				{
					stop_motors(0);							// Stop motors
					sample_counter = 0;						// Reset sample counter
					calibrationstate=0;
					// Update calibrated threshold values
					ThresholdApproximation(lightsnsr_thresh, calibration_minmax);
				}
				break;
			}
			break;
		}
	}
}
/*
 * PID CONTROLLER HELPER FUNCTIONS
 */
void PIDError(uint32_t periods[NUM_ARRAYS][NUM_SENSORS], int16_t error[NUM_ARRAYS][NUM_SENSORS], uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2])
{
	uint8_t n=0;
	for(n=0;n<NUM_ARRAYS;n++)
	{
		error[n][2] = error[n][1];
		error[n][1] = error[n][0];
		// Take weighted average of readings
		// 2000,1000,0
		uint32_t feedback = (0 * periods[n][0] + 100 * periods[n][1] + 200 * periods[n][2])/(periods[n][0] + periods[n][1] + periods[n][2]);
		uint32_t on_line_thresh = (CALIBRATIONSF * (minmax[n][0][i_MIN] + minmax[n][1][i_MIN] + minmax[n][2][i_MIN]));
		if((periods[n][0] + periods[n][1] + periods[n][2]) < on_line_thresh)// Check if we are too far left or right
		{
			if(error[n][1]<0) feedback = 0;												// if too far left, set error to 0
			if(error[n][1]>0) feedback = 200;											// If too far right, set error to 200
		}
		error[n][0] = (int16_t)feedback - LIGHTSENSEREF;				// if on line, Subtract the reference from the feedback for current error
	}
}
int32_t PIDController(int16_t * err, int32_t u)
{
	//int32_t result = u + (( (KP + KI + KD) * err[0] + (-KP - 2 * KD) * err[1] + KD * err[2])/100); // Difference method
	// Limit control effort
	int32_t result = err[0]/KP + (err[0] - err[1])/KI + 2*(err[0] - 2*err[1] + err[2])/KD;
	int32_t mag_u = abs(u);
	if(mag_u >= U_MAX) result = (u * U_MAX) / mag_u;
	return result;
}
void Calibration_Get(uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2], uint32_t sample[NUM_ARRAYS][NUM_SENSORS])
{
	uint8_t n=0;
	for(n=0; n<NUM_ARRAYS;n++)
	{
		uint8_t m=0;
		for(m=0; m<NUM_SENSORS;m++)
		{
			// Check mins
			if((minmax[n][m][i_MIN] == 0) | (sample[n][m] < minmax[n][m][i_MIN])) minmax[n][m][i_MIN] = sample[n][m];
			// Check maxes
			if(sample[n][m] > minmax[n][m][i_MAX]) minmax[n][m][i_MAX] = sample[n][m];
		}
	}
}
void Calibrate_Sensors(void)
{
	drivestate = DRIVESTATE_CALIBRATE;
}
/*
 * LINE DETECTION HELPER FUNCTIONS
 */
void Detect_Adjacent(uint8_t * readings, uint16_t reads[2][2], uint8_t index0, uint8_t index1, uint8_t * iterations)
{
	switch(linedetect_state)
	{
	case LINEDETECT_NULLS:
		if((readings[index0]==0) & (readings[index1]==0)) reads[0][NULLS]++;			// If white increment a counter
		else linedetect_state = LINEDETECT_BLACK;										// If not, change state to black counter
		break;
	case LINEDETECT_BLACK:
		if((readings[index0]) | (readings[index1]))										// Are sensors picking up black?
		{
			reads[0][BLACKS]++;															// if so, Increment a counter
			if(reads[0][BLACKS] > ((reads[0][NULLS] * LINEDETECT_K_NUM) / LINEDETECT_K_DEN))// Are we at our threshold?
			{
				if(reads[0][NULLS]<= NULLS_MIN)													// Did we even catch a null character the first read?
				{
					linedetect_state = LINEDETECT_NULLS;								// If so, Reset state counter and counters
					clearReadCNT(reads[0]);
				}
				else																	// If not,
				{
					(*iterations)++;													// We hit a line
					clearReadCNT(reads[0]);												// Reset counter
				}
			}
		}
		else
		{
			linedetect_state = LINEDETECT_NULLS;
			clearReadCNT(reads[0]);
		}
		break;
	}
}
void Detect_Intersection(uint8_t * val, uint16_t reads[2][2], uint8_t * iterations, uint8_t index_0, uint8_t index_1, uint8_t * flags)
{
	// If its white increment counter
	if(!val[index_0]) reads[0][NULLS]++;
	if(!val[index_1]) reads[1][NULLS]++;
	// Check if its black
	if(val[index_0])
	{
		reads[0][BLACKS]++;
		if(reads[0][NULLS] <= NULLS_MIN) clearReadCNT(reads[0]);
		if(reads[0][BLACKS]) if(reads[0][BLACKS] > ((reads[0][NULLS] * LINEDETECT_K_NUM_2) / LINEDETECT_K_DEN_2)) flags[0] = 1;
	}
	// check if its black
	if(val[index_1])
	{
		reads[1][BLACKS]++;
		if(reads[1][NULLS] <= NULLS_MIN) clearReadCNT(reads[1]);
		if(reads[1][BLACKS]) if(reads[1][BLACKS] > ((reads[1][NULLS] * LINEDETECT_K_NUM_2) / LINEDETECT_K_DEN_2)) flags[1] = 1;
	}
	// if both flags are up, do stuff
	if(flags[0] & flags[1])
	{
		(*iterations)++;									// We are at an intersection
		clearReadCNT(reads[0]);								// Clear Counters
		clearReadCNT(reads[1]);
		flags[0] = 0;										// Clear flags
		flags[1] = 0;
	}
}
void Detect_Front(uint8_t val, uint16_t * reads, uint8_t iterations, uint8_t moves, uint8_t * flag)
{
	switch(front_detect_state)
	{
	case FRONTLINEDETECT_NULLS:
		if(val != 0x7) reads[NULLS]++;														// If white increment a counter
		else front_detect_state = FRONTLINEDETECT_BLACK;									// If not, change state to black counter
		break;
	case FRONTLINEDETECT_BLACK:
		if(val == 0x7)																	// Are sensors picking up a black?
		{
			reads[BLACKS]++;															// if so, Increment a counter
			if(reads[BLACKS] > ((reads[NULLS] * LINEDETECT_K_NUM_3) / LINEDETECT_K_DEN_3))	// Are we at our threshold?
			{
				if(reads[NULLS]<= NULLS_MIN)											// Did we even catch a null character the first read?
				{
					front_detect_state = FRONTLINEDETECT_NULLS;							// If so, Reset state counter and counters
					clearReadCNT(reads);
				}
				else																	// If not, we hit a line!
				{
					if(((moves - iterations) == 1) & (!(*flag))) dutycycle /= FRONTLINEDETECT_CUTFACTOR;// Cut duty cycle down if we are stopping at this line
					clearReadCNT(reads);												// Reset counter
					(*flag) = 1;
				}
			}
		}
		else
		{
			front_detect_state = FRONTLINEDETECT_NULLS;									// If not, we go back to nulls state and clear counter
			clearReadCNT(reads);
		}
		break;
	}
}
void Check_Iterations(int32_t * u_fwd, int32_t * u_back, uint8_t * iterations, uint8_t moves)
{
	if((*iterations) >= moves)
	{
		// ERNIE!!!! ERNESTO!!! Put the ack statement here, if you need to tell what function it is, just read the current drive state, IT HAS to go before stop_motors, as that puts it in idle
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
void ThreshCompare(uint32_t raw_data[NUM_ARRAYS][NUM_SENSORS], uint32_t thresh[NUM_ARRAYS][NUM_SENSORS], uint8_t * result)
{
	uint8_t n;												// For loop index
	uint8_t m;
	for(m=0; m<NUM_ARRAYS; m++)
	{
		result[m] = 0;										// Reset value for bitset
		for(n=0; n<NUM_SENSORS; n++)						// Set value
			{
				if(raw_data[m][n]>thresh[m][n]) result[m] |= (1<<n);
			}
	}
}
void ThresholdApproximation(uint32_t thresh[NUM_ARRAYS][NUM_SENSORS], uint32_t minmax[NUM_ARRAYS][NUM_SENSORS][2])
{
	uint8_t n;
	uint8_t m;
	for(n=0;n<NUM_ARRAYS;n++)
	{
		for(m=0;m<NUM_SENSORS;m++)
		{
			thresh[n][m] = (minmax[n][m][i_MIN] + (minmax[n][m][i_MAX] - minmax[n][m][i_MIN])/CALIBRATION_SCALING_FACTOR);
		}
	}
}
/*
 * MISC HELPER FUNCTIONS
 */
int32_t abs(int32_t val)
{
	return val > 0 ? val : -val;
}
