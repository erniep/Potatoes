/*
 * motors.c
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */

#include "motors.h"
#include "drive.h"
/* Global vars */
uint8_t num_moves = 0;
/* Externs */
extern uint8_t dutycycle;
extern uint8_t drivestate;
void Robot_PWM_init(void)
{
//
// Configure PWM Module
//
	// Set PWM clock divider to 32
	SysCtlPWMClockSet(PWM_PRESCALER);
	// Turn on PWM0, and PWM1
	SysCtlPeripheralEnable(SYSCTL_MOTORSA);
	SysCtlPeripheralEnable(SYSCTL_MOTORSB);
	// Set PWM output pins to type PWM
	GPIOPinTypePWM(GPIO_BASE_TOPM, GPIO_M1 | GPIO_M2);
	GPIOPinTypePWM(GPIO_BASE_BOTM, GPIO_M3 |GPIO_M4);
	GPIOPinTypePWM(GPIO_BASE_SERVO_L,  GPIO_SERVO_L);
	GPIOPinTypePWM(GPIO_BASE_SERVO_R,  GPIO_SERVO_R);
	// GPIO Pin configure
	GPIOPinConfigure(M1);
	GPIOPinConfigure(M2);
	GPIOPinConfigure(M3);
	GPIOPinConfigure(M4);
	GPIOPinConfigure(SERVO_L);
	GPIOPinConfigure(SERVO_R);
	// Configure PWM Generators
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_BOTM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_TOPM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_SERVO_R_BASE, PWM_GEN_SERVO_R, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_SERVO_L_BASE, PWM_GEN_SERVO_L, PWM_GEN_MODE_UP_DOWN);
	// Calculate Period for Motors

	uint32_t clock = BUS_CLOCK/PWM_PRESCALER_VAL;
	uint32_t period = clock / SWFREQ_MOTORS;
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_BOTM, period);
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_TOPM, period);
	// Calculate Period for Servo
	period = clock / SWFREQ_ANA_SERVO;
	PWMGenPeriodSet(PWM_SERVO_R_BASE, PWM_GEN_SERVO_R, period);
	PWMGenPeriodSet(PWM_SERVO_L_BASE, PWM_GEN_SERVO_L, period);
//
// Set initial duty cycle to 0, and parallel servo's
//
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_L_BASE, SERVO_L_OUT, DUTY_PARALLEL);
	PWMPulseWidthSet(PWM_SERVO_R_BASE, SERVO_R_OUT, DUTY_PARALLEL);
//
// Enable PWM output
//
	PWMOutputState(PWM_MOTOR_BASE, M1_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M2_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M3_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M4_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_L_BASE, SERVO_L_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_R_BASE, SERVO_R_OUT_BIT, true);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_BOTM);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMGenEnable(PWM_SERVO_R_BASE, PWM_GEN_SERVO_R);
	PWMGenEnable(PWM_SERVO_L_BASE, PWM_GEN_SERVO_L);
//
// Initialize Control Pins
//
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, PA5_M3B);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B);
}
void stop_motors(uint8_t duty)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_IDLE;
	// Set Enable pin HI
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, (((100 * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, (((100 * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, (((100 * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, (((100 * period)/100) - 1));
	// Set all motors to brake
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A | PD3_M1B);
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE1_M2A | PE2_M2B | PE3_M3A);
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A | PB5_M4B);
}
void coast_motors(uint8_t duty)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_IDLE;
	// Set duty cycle to zero
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
}
void fwd_pid(int32_t ufwd,int32_t uback, uint8_t dutyref)
{
	int32_t mDC[4];
	mDC[0] = (int32_t)dutyref - ufwd;
	mDC[1] = (int32_t)dutyref + ufwd;
	mDC[2] = (int32_t)dutyref + uback;
	mDC[3] = (int32_t)dutyref - uback;
	control_effort_limit(mDC);
	setDC(mDC);
}
void rev_pid(int32_t ufwd, int32_t uback, uint8_t dutyref)
{
	int32_t mDC[4];
	mDC[0] = (int32_t)dutyref + ufwd;
	mDC[1] = (int32_t)dutyref - ufwd;
	mDC[2] = (int32_t)dutyref - uback;
	mDC[3] = (int32_t)dutyref + uback;
	control_effort_limit(mDC);
	setDC(mDC);
}
void tl_pid(int32_t uleft,int32_t uright, uint8_t dutyref)
{
	int32_t mDC[4];
	mDC[0] = dutyref - uleft;
	mDC[1] = dutyref + uright;
	mDC[2] = dutyref + uleft;
	mDC[3] = dutyref - uright;
	control_effort_limit(mDC);
	setDC(mDC);
}
void tr_pid(int32_t u_left,int32_t u_right, uint8_t dutyref)
{
	int32_t mDC[4];
	mDC[0] = dutyref + u_left;
	mDC[2] = dutyref - u_left;
	mDC[1] = dutyref - u_right;
	mDC[3] = dutyref + u_right;
	control_effort_limit(mDC);
	setDC(mDC);
}
void fw_motors(uint8_t duty, uint8_t num_cells)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_FWD;
	// Set num of turns wanted
	num_moves = num_cells;
	// Set duty cycle
	dutycycle = duty;

	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:FWD, M2:FWD, M3:FWD, M4:FWD
	//M1:FWD, 1A hi, 1B lo
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A);
	//M2:REV, 2A hi, 2B low, M3:FWD, 3A hi
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE1_M2A | PE3_M3A);
	//M3:FWD, 3B low
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
	//M4:REV, 4A hi, 4B lo
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A);
}
void cw_motors(uint8_t duty, uint8_t num_turns)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_TURNCW;
	// Set num of turns wanted
	num_moves = num_turns;
	// Set duty cycle
	dutycycle = duty;

	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:FWD, M2:REV, M3:FWD, M4:REV
	//M1:FWD, 1A hi, 1B lo
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A);
	//M2:REV, 2A lo, 2B hi, M3:FWD, 3A hi
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE2_M2B | PE3_M3A);
	//M3:FWD, 3B low
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
	//M4:REV, 4A lo, 4B hi
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB5_M4B);
}
void ccw_motors(uint8_t duty, uint8_t num_turns)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_TURNCCW;
	// Set num of turns wanted
	num_moves = num_turns;
	// Set duty cycle
	dutycycle = duty;

	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:REV, M2:FWD, M3:REV, M4:FWD
	//M1:REV, 1A lo, 1B hi
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD3_M1B);
	//M2:FWD, 2A hi, 2B lo, M3:REV, 3A lo
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE1_M2A);
	//M3:REV, 3B hi
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
	//M4:FWD, 4A hi, 4B lo
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A);
}
void rv_motors(uint8_t duty, uint8_t num_cells)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_REV;
	// Set num of turns wanted
	num_moves = num_cells;
	// Set duty cycle
	dutycycle = duty;

	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:REV, M2:REV, M3:REV, M4:REV
	//M1:REV, 1A lo, 1B hi
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD3_M1B);
	//M2:REV, 2A lo, 2B hi, M3:REV, 3A lo
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE2_M2B);
	//M3:REV, 3B hi
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
	//M4:REV, 4A lo, 4B hi
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB5_M4B);
}
void tl_motors(uint8_t duty, uint8_t num_cells)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_STRAFELEFT;
	// Set num of turns wanted
	num_moves = num_cells;

	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:REV, M2:FWD, M3:FWD, M4:REV
	//M1:REV, 1A lo, 1B hi
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD3_M1B);
	//M2:FWD, 2A hi, 2B lo, M3:FWD, 3A hi
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE1_M2A | PE3_M3A);
	//M3:FWD, 3B lo
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
	//M4:REV, 4A lo, 4B hi
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB5_M4B);
}
void tr_motors(uint8_t duty, uint8_t num_cells)
{
	// Change State counter on drive task
	drivestate = DRIVESTATE_STRAFERIGHT;
	// Set num of turns wanted
	num_moves = num_cells;
	// Set duty cycle
	dutycycle = duty;
	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:FWD, M2:REV, M3:REV, M4:FWD
	//M1:FWD, 1A hi, 1B lo
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A);
	//M2:REV, 2A lo, 2B hi, M3:REV, 3A lo
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE2_M2B);
	//M3:REV, 3B hi
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
	//M4:FWD, 4A hi, 4B lo
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A);
}
void cw_motors_openloop(uint8_t duty)
{
	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:FWD, M2:REV, M3:FWD, M4:REV
	//M1:FWD, 1A hi, 1B lo
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A);
	//M2:REV, 2A lo, 2B hi, M3:FWD, 3A hi
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE2_M2B | PE3_M3A);
	//M3:FWD, 3B low
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
	//M4:REV, 4A lo, 4B hi
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB5_M4B);
}
void ccw_motors_openloop(uint8_t duty)
{
	// Set periods on Motors
	setDC_all(duty);

	// Control Pins
	//M1:REV, M2:FWD, M3:REV, M4:FWD
	//M1:REV, 1A lo, 1B hi
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD3_M1B);
	//M2:FWD, 2A hi, 2B lo, M3:REV, 3A lo
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, PE1_M2A);
	//M3:REV, 3B hi
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
	//M4:FWD, 4A hi, 4B lo
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A);
}
void servo_parallel(void)
{
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_R_BASE, PWM_GEN_SERVO_R);
	PWMPulseWidthSet(PWM_SERVO_R_BASE, SERVO_R_OUT, (((DUTY_PARALLEL * period)/100)-1));
	PWMPulseWidthSet(PWM_SERVO_L_BASE, SERVO_L_OUT, (((DUTY_PARALLEL * period)/100)-1));
}
void servo_tangent(void)
{
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_R_BASE, PWM_GEN_SERVO_R);
	PWMPulseWidthSet(PWM_SERVO_R_BASE, SERVO_R_OUT, (((DUTY_TANGENT * period)/100)-1));
	PWMPulseWidthSet(PWM_SERVO_L_BASE, SERVO_L_OUT, (((DUTY_TANGENT * period)/100)-1));
}
void control_effort_limit(int32_t * DCmotors)
{
	uint8_t n = 0;
	for(n=0; n<4; n++)
	{
		if(DCmotors[n] > UPPERLIM) DCmotors[n] = UPPERLIM;
		else if (DCmotors[n] < LOWERLIM) DCmotors[n] = LOWERLIM;
	}
}
void setDC(int32_t * DCmotors)
{
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, (((DCmotors[0] * period)/100))-1);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, (((DCmotors[1] * period)/100))-1);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, (((DCmotors[2] * period)/100))-1);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, (((DCmotors[3] * period)/100))-1);
}
void setDC_all(uint8_t duty)
{
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, (((duty * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, (((duty * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, (((duty * period)/100) - 1));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, (((duty * period)/100) - 1));
}
