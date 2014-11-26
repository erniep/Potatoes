/*
 * motors.c
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */
/* Standard Libraries */
//#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* TIVAC Function Library */
#include <driverlib/pin_map.h>
#include <inc/hw_memmap.h>
#include <driverlib/pwm.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>


// Switching frequencies
#define swfreq_motors 	50
#define swfreq_servos 	30
// PWM Clock Prescaler

// PWM out Pins
#define M1_OUT			PWM_OUT_2
#define M2_OUT			PWM_OUT_3
#define M3_OUT			PWM_OUT_0
#define M4_OUT			PWM_OUT_1
#define SERVO1_OUT		PWM_OUT_4
#define SERVO2_OUT		PWM_OUT_5
#define M1_OUT_BIT		PWM_OUT_2_BIT
#define M2_OUT_BIT		PWM_OUT_3_BIT
#define M3_OUT_BIT		PWM_OUT_0_BIT
#define M4_OUT_BIT		PWM_OUT_1_BIT
#define SERVO1_OUT_BIT	PWM_OUT_4_BIT
#define SERVO2_OUT_BIT	PWM_OUT_5_BIT
// PWM Gens
#define PWM_MOTOR_BASE	PWM1_BASE
#define PWM_SERVO_BASE	PWM0_BASE
#define PWM_GEN_BOTM	PWM_GEN_0
#define PWM_GEN_TOPM	PWM_GEN_1
#define PWM_GEN_SERVO	PWM_GEN_2
// GPIO Defines - Motor
#define GPIO_M1 		GPIO_PIN_6
#define GPIO_M2 		GPIO_PIN_7
#define GPIO_M3 		GPIO_PIN_0
#define GPIO_M4 		GPIO_PIN_1
#define GPIO_SERVO1 	GPIO_PIN_4
#define GPIO_SERVO2 	GPIO_PIN_5
#define GPIO_BASE_TOPM	GPIO_PORTA_BASE
#define GPIO_BASE_BOTM	GPIO_PORTD_BASE
#define GPIO_BASE_SERVO	GPIO_PORTE_BASE
#define M1 				GPIO_PA6_M1PWM2
#define M2 				GPIO_PA7_M1PWM3
#define M3 				GPIO_PD0_M1PWM0
#define M4 				GPIO_PD1_M1PWM1
#define SERVO1			GPIO_PE4_M0PWM4
#define SERVO2			GPIO_PE5_M0PWM5
// GPIO Defines - Control
#define PD2_M1A			GPIO_PIN_2
#define PD3_M1B			GPIO_PIN_3
#define PE1_M2A			GPIO_PIN_1
#define PE2_M2B			GPIO_PIN_2
#define PE3_M3A			GPIO_PIN_3
#define PA5_M3B			GPIO_PIN_5
#define PA6_M4A			GPIO_PIN_6
#define PA7_M4B			GPIO_PIN_7
// SYSCTL Defines
#define SYSCTL_MOTORS 	SYSCTL_PERIPH_PWM1
#define SYSCTL_SERVOS 	SYSCTL_PERIPH_PWM0

void Robot_PWM_init(void)
{
/*
 * Configure PWM Module
 */
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
	uint32_t period = SysCtlClockGet() / swfreq_motors;
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_BOTM, period);
	PWMGenPeriodSet(PWM_MOTOR_BASE, PWM_GEN_TOPM, period);
	// Calculate Period for Servo
	period = SysCtlClockGet() / swfreq_servos;
	PWMGenPeriodSet(PWM_SERVO_BASE, PWM_GEN_SERVO, period);
/*
 * Set initial duty cycle to 0
 */
	PWMPulseWidthSet(PWM_MOTOR_BASE, M1_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M2_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M3_OUT, 0);
	PWMPulseWidthSet(PWM_MOTOR_BASE, M4_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, SERVO1_OUT, 0);
	PWMPulseWidthSet(PWM_SERVO_BASE, SERVO2_OUT, 0);
/*
 * Enable PWM output
 */
	PWMOutputState(PWM_MOTOR_BASE, M1_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M2_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M3_OUT_BIT, true);
	PWMOutputState(PWM_MOTOR_BASE, M4_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, SERVO1_OUT_BIT, true);
	PWMOutputState(PWM_SERVO_BASE, SERVO2_OUT_BIT, true);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_BOTM);
	PWMGenEnable(PWM_MOTOR_BASE, PWM_GEN_TOPM);
	PWMGenEnable(PWM_SERVO_BASE, PWM_GEN_SERVO);
/*
 * Initialize Control Pins
 */
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, PD2_M1A | PD3_M2A);
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, PE1_M2A | PE2_M2B | PE3_M3A);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, PA5_M3B | PA6_M4A | PA7_M4B);
}
//stoprobot
//motorsfw
//motorsrv
//motorstl
//motorstr


