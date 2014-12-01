/*
 * motors.c
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */
#include "inc.h"

void Robot_PWM_init(void)
{
//
// Configure PWM Module
//
	// Set PWM clock divider to 1
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	// Turn on PWM0, and PWM1
	SysCtlPeripheralEnable(SYSCTL_MOTORS);
	SysCtlPeripheralEnable(SYSCTL_SERVOS);
	// Set PWM output pins to type PWM
	GPIOPinTypePWM(GPIO_BASE_TOPM, GPIO_M1 | GPIO_M2);
	GPIOPinTypePWM(GPIO_BASE_BOTM, GPIO_M3 |GPIO_M4);
	GPIOPinTypePWM(GPIO_BASE_SERVO, GPIO_SERVO1 | GPIO_SERVO2);
	// GPIO Pin configure
	GPIOPinConfigure(M1);
	GPIOPinConfigure(M2);
	GPIOPinConfigure(M3);
	GPIOPinConfigure(M4);
	GPIOPinConfigure(SERVO1);
	GPIOPinConfigure(SERVO2);
	// Configure PWM Generators
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_BOTM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_TOPM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_SERVO_BASE, PWM_GEN_SERVO, PWM_GEN_MODE_UP_DOWN);
	// Calculate Period for Motors
	int period = SysCtlClockGet() / swfreq_motors;
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_BOTM, period);
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_TOPM, period);
	// Calculate Period for Servo
	period = SysCtlClockGet() / swfreq_servos;
	PWMGenPeriodSet(PWM_SERVO_BASE, PWM_GEN_SERVO, period);
//
// Set initial duty cycle to 0
//
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, SERVO1_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, SERVO2_OUT, 0);
//
// Enable PWM output
//
	PWMOutputState(PWM_MOTOR_BASE, M1_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M2_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M3_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M4_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, SERVO1_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, SERVO2_OUT_BIT, true);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_BOTM);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMGenEnable(PWM_SERVO_BASE, PWM_GEN_SERVO);
//
// Initialize Control Pins
//
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, PA5_M3B);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B);
}

void motorAll(int motor1, int motor2, int motor3, int motor4) {
	// Speed Standard:
	// 150 			- Coast
	// 1 to 100 	- Forward
	// -100 to -1 	- Reverse
	// 0 			- Stop
	int motor1dir = 1, motor2dir = 1, motor3dir = 1, motor4dir = 1;

	switch (motor1) {
		case 150:
			// Set duty cycle to zero
			PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
		case 0:
			// M1: STOP, 1A lo, 1B lo
			GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, 0);
		default:
			if (motor1 < 0) {
				motor1dir = -1;
				// M1: REV, 1A lo, 1B hi
				GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD3_M1B);
			} else {
				// M1: FWD, 1A hi, 1B lo
				GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, PD2_M1A);
			}
	}

	switch (motor2) {
		case 150:
			// Set duty cycle to zero
			PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
		case 0:
			// M2: STOP, 1A lo, 1B lo
			GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B, 0);
		default:
			if (motor2 < 0) {
				motor2dir = -1;
				// M2: REV, 2A lo, 2B hi
				GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B, PE2_M2B);
			} else {
				// M2: FWD, 2A hi, 2B lo
				GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B, PE1_M2A);
			}
	}

	switch (motor3) {
		case 150:
			// Set duty cycle to zero
			PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
		case 0:
			// M3: STOP, 3A lo, 3B lo
			GPIOPinWrite(GPIO_PORTA_BASE, PE3_M3A | PA5_M3B, 0);
		default:
			if (motor3 < 0) {
				motor3dir = -1;
				// M3: REV, 3A hi, 3B low
				GPIOPinWrite(GPIO_PORTE_BASE, PE3_M3A, PE3_M3A);
				GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
			} else {
				// M3: FWD, 3A hi, 3B low
				GPIOPinWrite(GPIO_PORTE_BASE, PE3_M3A, 0);
				GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, PA5_M3B);
			}
	}

	switch (motor4) {
		case 150:
			// Set duty cycle to zero
			PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
		case 0:
			// M4: STOP, 4A lo, 4B lo
			GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, 0);
		default:
			if (motor4 < 0) {
				motor4dir = -1;
				// M4: REV, 4A lo, 4B hi
				GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB5_M4B);
			} else {
				// M4: FWD, 4A hi, 4B lo
				GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, PB4_M4A);
			}
	}

	int period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((motor1dir * motor1 * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((motor2dir * motor2 * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((motor3dir * motor3 * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((motor4dir * motor4 * period)/100));
}
void coast_motors (int duty) {
	motorAll(150, 150, 150, 150);
}
void stop_motors (int duty) {
	motorAll(0, 0, 0, 0);
}
void fw_motors (int duty) {
	//M1:FWD, M2:FWD, M3:FWD, M4:FWD
	motorAll(duty, duty, duty, duty);
}

void rv_motors (int duty) {
	//M1:REV, M2:REV, M3:REV, M4:REV
	motorAll( -1 * duty, -1 * duty, -1 * duty, -1 * duty);
}

void cw_motors (int duty) {
	//M1:FWD, M2:REV, M3:FWD, M4:REV
	motorAll(duty, -1 * duty, duty, -1 * duty);
}

void cw2_motors(int duty) {	
	//M1:FWD, M2:REV, M3:FWD, M4:REV
	motorAll(duty, -1 * duty, duty/2, -1 * duty/2);
}
void ccw_motors (int duty) {
	//M1:REV, M2:FWD, M3:REV, M4:FWD
	motorAll( -1 * duty, duty, -1 * duty, duty);
}
void ccw2_motors (int duty) {
	//M1:REV, M2:FWD, M3:REV, M4:FWD
	motorAll( -1 * duty/2, duty/2, -1 * duty, duty);
}

void tl_motors(int duty) {
	//M1:REV, M2:FWD, M3:FWD, M4:REV
	motorAll( -1 * duty, duty, duty, -1 * duty);
}

void tr_motors (int duty) {
	//M1:FWD, M2:REV, M3:REV, M4:FWD
	motorAll(duty, -1 * duty, -1 * duty, duty);
}
