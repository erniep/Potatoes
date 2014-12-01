#ifndef ernieMotor_H
#define ernieMotor_H

/*
 * motors.c
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */
// Switching frequencies
#define swfreq_motors 	500
#define swfreq_servos 	30

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
#define PB4_M4A			GPIO_PIN_4
#define PB5_M4B			GPIO_PIN_5
// SYSCTL Defines
#define SYSCTL_MOTORS 	SYSCTL_PERIPH_PWM1
#define SYSCTL_SERVOS 	SYSCTL_PERIPH_PWM0


void Robot_PWM_init (void);
void coast_motors (int duty);

void stop_motors (int data);

void motorAll(int motor1, int motor2, int motor3, int motor4);

void fw_motors (int duty);

void rv_motors (int duty);
void cw_motors (int duty);

void ccw_motors (int duty);

void tl_motors(int duty);
void tr_motors (int duty);


#endif
