/*
 * motors.h
 *
 *  Created on: Nov 26, 2014
 *      Author: Wicho
 */

#ifndef MOTORS_H_
#define MOTORS_H_
/*
 * Library Includes
 */
/* Standard Libraries */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* TIVAC Function Library */
#include <driverlib/pin_map.h>
#include <inc/hw_memmap.h>
#include <driverlib/pwm.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
//
//		Robot
//------------------
//
//	M1			M2
//
//	M3			M4
//
//------------------
//
//	Camera Servos
// Pan: Servo 1
// Tilt: Servo 2
//-------------------

// Switching frequencies
#define SWFREQ_MOTORS 			50
#define SWFREQ_ANA_SERVO 		300
#define SWFREQ_DIGI_SERVO		300

// Duty Cycles for Pan servo
#define duty_lt 59
#define duty_rt 33
#define duty_fwd 46

// PWM out Pins
#define M1_OUT					PWM_OUT_2
#define M2_OUT					PWM_OUT_3
#define M3_OUT					PWM_OUT_0
#define M4_OUT					PWM_OUT_1
#define PAN_SERVO_OUT			PWM_OUT_5
#define TILT_SERVO_OUT			PWM_OUT_5
#define M1_OUT_BIT				PWM_OUT_2_BIT
#define M2_OUT_BIT				PWM_OUT_3_BIT
#define M3_OUT_BIT				PWM_OUT_0_BIT
#define M4_OUT_BIT				PWM_OUT_1_BIT
#define PAN_SERVO_OUT_BIT		PWM_OUT_5_BIT
#define TILT_SERVO_OUT_BIT		PWM_OUT_5_BIT
// PWM Gens
#define PWM_MOTOR_BASE			PWM1_BASE
#define PWM_SERVO_BASE			PWM0_BASE
#define PWM_GEN_BOTM			PWM_GEN_0
#define PWM_GEN_TOPM			PWM_GEN_1
#define PWM_GEN_TILT_SERVO		PWM_GEN_2
#define PWM_GEN_PAN_SERVO		PWM_GEN_1
// GPIO Defines - Motor
#define GPIO_M1 				GPIO_PIN_6
#define GPIO_M2 				GPIO_PIN_7
#define GPIO_M3 				GPIO_PIN_0
#define GPIO_M4 				GPIO_PIN_1
#define GPIO_PAN_SERVO 			GPIO_PIN_5
#define GPIO_TILT_SERVO			GPIO_PIN_5
#define GPIO_BASE_TOPM			GPIO_PORTA_BASE
#define GPIO_BASE_BOTM			GPIO_PORTD_BASE
#define GPIO_BASE_PAN_SERVO		GPIO_PORTB_BASE
#define GPIO_BASE_TILT_SERVO	GPIO_PORTE_BASE
#define M1 						GPIO_PA6_M1PWM2
#define M2 						GPIO_PA7_M1PWM3
#define M3 						GPIO_PD0_M1PWM0
#define M4 						GPIO_PD1_M1PWM1
// FIX THIS Pb5 ALREADY USED
#define PAN_SERVO				GPIO_PB5_M0PWM3
#define TILT_SERVO				GPIO_PE5_M0PWM5
// GPIO Defines - Control
#define PD2_M1A					GPIO_PIN_2
#define PD3_M1B					GPIO_PIN_3
#define PE1_M2A					GPIO_PIN_1
#define PE2_M2B					GPIO_PIN_2
#define PE3_M3A					GPIO_PIN_3
#define PA5_M3B					GPIO_PIN_5
#define PB4_M4A					GPIO_PIN_4
#define PB5_M4B					GPIO_PIN_5
// SYSCTL Defines
#define SYSCTL_MOTORS 			SYSCTL_PERIPH_PWM1
#define SYSCTL_SERVOS 			SYSCTL_PERIPH_PWM0
/*
 * Function prototypes
 */
//*****************************************************************************
void Robot_PWM_init(void);
//*****************************************************************************
//
//! Initializes the Robot
//!
//! \param none
//!
//! This initializes the robot to motors with PWM and servos
//!
//! \return none
//
//*****************************************************************************
void tr_motors(uint8_t duty);
//*****************************************************************************
//
//! Moves the robot to the right
//!
//! \param ui8duty is the intended duty cycle
//!
//! Given a duty cycle, this function moves the robot via mecanum wheels,
//! transationally right. This is done via force vectors.
//!
//! \return none
//
//*****************************************************************************
void tl_motors(uint8_t duty);
//*****************************************************************************
//
//! Moves the robot to the left
//!
//! \param ui8duty is the intended duty cycle
//!
//! Given a duty cycle, this function moves the robot via mecanum wheels,
//! transationally right. This is done via force vectors.
//!
//! \return none
//
//*****************************************************************************
void rv_motors(uint8_t duty);
//*****************************************************************************
//
//! Moves the robot in the reverse direction
//!
//! \param ui8duty is the intended duty cycle
//!
//! Given a duty cycle, this function moves the robot backwards via mecanum wheels
//!
//! \return none
//
//*****************************************************************************
void fw_motors(uint8_t duty);
//*****************************************************************************
//
//! Moves the robot in the foward direction
//!
//! \param ui8duty is the intended duty cycle
//!
//! Given a duty cycle, this function moves the foward backwards via mecanum wheels
//!
//! \return none
//
//*****************************************************************************
void coast_motors(uint8_t duty);
//*****************************************************************************
//
//! Coast robots
//!
//! \param ui8duty is the intended duty cycle
//!
//! This function allows the motors to coast by turning off the PWM signal
//!
//! \return none
//
//*****************************************************************************
void stop_motors(uint8_t duty);
//*****************************************************************************
//
//! Stop_motors
//!
//! \param none
//!
//! This function puts the motors in brake mode, causing the robot to cease movement
//!
//! \return none
//
//*****************************************************************************
void cw_motors(uint8_t duty);
//*****************************************************************************
//
//! Turn Clockwise
//!
//! \param ui8duty is the intended duty cycle
//!
//! This function turns the robot clockwise
//!
//! \return none
//
//*****************************************************************************
void ccw_motors(uint8_t duty);
//*****************************************************************************
//
//! Counter clockwise turn
//!
//! \param ui8duty is the intended duty cycle
//!
//! This function turns the robot counter clockwise
//!
//! \return none
//
//*****************************************************************************
void pan_servo_DC(uint8_t duty);
//*****************************************************************************
//
//! Pan servo - Change duty cycle
//!
//! \param ui8duty is the intended duty cycle
//!
//! This function adjusts the duty cycle for the pan servo
//!
//! \return none
//
//*****************************************************************************
void tilt_servo_DC(uint8_t duty);
//*****************************************************************************
//
//! Tilt servo - Change duty cycle
//!
//! \param ui8duty is the intended duty cycle
//!
//! This function changes the duty cycle of the tilt servo
//!
//! \return none
//
//*****************************************************************************
void pan_lt(void);
//*****************************************************************************
//
//! Pan servo 90 deg to left
//!
//! \param none
//!
//! \return none
//
//*****************************************************************************
void pan_fwd(void);
//*****************************************************************************
//
//! Pan servo 0 deg forward
//!
//! \param none
//!
//! \return none
//
//*****************************************************************************
void pan_rt(void);
//*****************************************************************************
//
//! Pan servo 90 to right
//!
//! \param none
//!
//! \return none
//
//*****************************************************************************
#endif /* MOTORS_H_ */
