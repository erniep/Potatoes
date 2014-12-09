/*
 * motors.c
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */
#include "motors.h"
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
	GPIOPinTypePWM(GPIO_BASE_PAN_SERVO,  GPIO_TILT_SERVO);
	GPIOPinTypePWM(GPIO_BASE_TILT_SERVO,  GPIO_PAN_SERVO);
	// GPIO Pin configure
	GPIOPinConfigure(M1);
	GPIOPinConfigure(M2);
	GPIOPinConfigure(M3);
	GPIOPinConfigure(M4);
	GPIOPinConfigure(PAN_SERVO);
	GPIOPinConfigure(TILT_SERVO);
	// Configure PWM Generators
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_BOTM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_MOTOR_BASE, PWM_GEN_TOPM, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_SERVO_BASE, PWM_GEN_TILT_SERVO, PWM_GEN_MODE_UP_DOWN);
	PWMGenConfigure(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO, PWM_GEN_MODE_UP_DOWN);
	// Calculate Period for Motors
	uint32_t period = SysCtlClockGet() / SWFREQ_MOTORS;
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_BOTM, period);
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_TOPM, period);
	// Calculate Period for Servo
	period = SysCtlClockGet() / SWFREQ_ANA_SERVO;
	PWMGenPeriodSet(PWM_SERVO_BASE, PWM_GEN_TILT_SERVO, period);
	period = SysCtlClockGet() / SWFREQ_DIGI_SERVO;
	PWMGenPeriodSet(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO, period);
//
// Set initial duty cycle to 0
//
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, PAN_SERVO_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, TILT_SERVO_OUT, 0);
//
// Enable PWM output
//
	PWMOutputState(PWM_MOTOR_BASE, M1_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M2_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M3_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M4_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, PAN_SERVO_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, TILT_SERVO_OUT_BIT, true);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_BOTM);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMGenEnable(PWM_SERVO_BASE, PWM_GEN_TILT_SERVO);
	PWMGenEnable(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO);
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
	// Set all motors to brake
	GPIOPinWrite(GPIO_PORTD_BASE, PD2_M1A | PD3_M1B, 0);
	GPIOPinWrite(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A, 0);
	GPIOPinWrite(GPIO_PORTA_BASE, PA5_M3B, 0);
	GPIOPinWrite(GPIO_PORTB_BASE, PB4_M4A | PB5_M4B, 0);
}
void coast_motors(uint8_t duty)
{
	// Set duty cycle to zero
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
}
void fw_motors(uint8_t duty)
{
	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void cw_motors(uint8_t duty)
{
	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void ccw_motors(uint8_t duty)
{

	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void rv_motors(uint8_t duty)
{
	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void tl_motors(uint8_t duty)
{
	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void tr_motors(uint8_t duty)
{
	// Set periods on Motors
	uint32_t period = PWMGenPeriodGet(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, ((duty * period)/100));
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, ((duty * period)/100));

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
void pan_servo_DC(uint8_t duty)
{
	// Get period
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO);
	PWMPulseWidthSet(PWM_SERVO_BASE, PAN_SERVO_OUT, ((duty * period)/100));
}
void tilt_servo_DC(uint8_t duty)
{
	// Get period
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_BASE, PWM_GEN_TILT_SERVO);
	PWMPulseWidthSet(PWM_SERVO_BASE, TILT_SERVO_OUT, ((duty * period)/100));
}
void pan_lt(void)
{
	// Get period
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO);
	PWMPulseWidthSet(PWM_SERVO_BASE, PAN_SERVO_OUT, ((duty_lt * period)/100));
}
void pan_fwd(void)
{
	// Get period
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO);
	PWMPulseWidthSet(PWM_SERVO_BASE, PAN_SERVO_OUT, ((duty_fwd * period)/100));
}

void pan_rt(void)
{
	// Get period
	uint32_t period = PWMGenPeriodGet(PWM_SERVO_BASE, PWM_GEN_PAN_SERVO);
	PWMPulseWidthSet(PWM_SERVO_BASE, PAN_SERVO_OUT, ((duty_rt * period)/100));
}

