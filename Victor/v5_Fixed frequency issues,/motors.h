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

#include <inc/hw_memmap.h>
#include "driverlib/pin_map.h"
#include <driverlib/pwm.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include "inc/hw_types.h"
#include "inc/hw_pwm.h"
/*
 * RTOS Variables
 *
 * HWI
 * ISR_NAME						Interrupt number
 *
 * TASK
 * TASK_NAME					Priority
 *
 * CLOCK (main clock 10ms)
 *
 * SEMAPHORES
 *
 */
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
#define SWFREQ_ANA_SERVO 		50

// Max Duty Cycles for PID
#define LOWERLIM				5
#define UPPERLIM				95
// Duty Cycles for Servos
#define DUTY_PARALLEL 7
#define DUTY_TANGENT 11

// PWM out Pins
#define M1_OUT					PWM_OUT_2
#define M2_OUT					PWM_OUT_3
#define M3_OUT					PWM_OUT_0
#define M4_OUT					PWM_OUT_1
#define SERVO_L_OUT				PWM_OUT_5
#define SERVO_R_OUT				PWM_OUT_5
#define M1_OUT_BIT				PWM_OUT_2_BIT
#define M2_OUT_BIT				PWM_OUT_3_BIT
#define M3_OUT_BIT				PWM_OUT_0_BIT
#define M4_OUT_BIT				PWM_OUT_1_BIT
#define SERVO_L_OUT_BIT			PWM_OUT_5_BIT
#define SERVO_R_OUT_BIT			PWM_OUT_5_BIT
// PWM Gens
#define PWM_MOTOR_BASE			PWM1_BASE
#define PWM_SERVO_L_BASE		PWM1_BASE
#define PWM_SERVO_R_BASE		PWM0_BASE
#define PWM_GEN_BOTM			PWM_GEN_0
#define PWM_GEN_TOPM			PWM_GEN_1
#define PWM_GEN_SERVO_R			PWM_GEN_2
#define PWM_GEN_SERVO_L			PWM_GEN_2
// GPIO Defines - Motor
#define GPIO_M1 				GPIO_PIN_6
#define GPIO_M2 				GPIO_PIN_7
#define GPIO_M3 				GPIO_PIN_0
#define GPIO_M4 				GPIO_PIN_1
#define GPIO_SERVO_L 			GPIO_PIN_1
#define GPIO_SERVO_R			GPIO_PIN_5
#define GPIO_BASE_TOPM			GPIO_PORTA_BASE
#define GPIO_BASE_BOTM			GPIO_PORTD_BASE
#define GPIO_BASE_SERVO_L		GPIO_PORTF_BASE
#define GPIO_BASE_SERVO_R		GPIO_PORTE_BASE
#define M1 						GPIO_PA6_M1PWM2
#define M2 						GPIO_PA7_M1PWM3
#define M3 						GPIO_PD0_M1PWM0
#define M4 						GPIO_PD1_M1PWM1
#define SERVO_L					GPIO_PF1_M1PWM5
#define SERVO_R					GPIO_PE5_M0PWM5
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
#define SYSCTL_MOTORSA 			SYSCTL_PERIPH_PWM1
#define SYSCTL_MOTORSB 			SYSCTL_PERIPH_PWM0
// Clock defines
#define BUS_CLOCK				80000000
#define PWM_PRESCALER			SYSCTL_PWMDIV_32
#define PWM_PRESCALER_VAL		32
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
void tr_motors(uint8_t duty, uint8_t num_cells);
//*****************************************************************************
//
//! Moves the robot to the right
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of cells to be moved
//!
//! Given a duty cycle, this function moves the robot via mecanum wheels,
//! transationally right. This is done via force vectors.
//!
//! \return none
//
//*****************************************************************************
void tl_motors(uint8_t duty, uint8_t num_cells);
//*****************************************************************************
//
//! Moves the robot to the left
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of cells to be moved
//!
//! Given a duty cycle, this function moves the robot via mecanum wheels,
//! transationally right. This is done via force vectors.
//!
//! \return none
//
//*****************************************************************************
void rv_motors(uint8_t duty, uint8_t num_cells);
//*****************************************************************************
//
//! Moves the robot in the reverse direction
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of cells to be moved
//!
//! Given a duty cycle, this function moves the robot backwards via mecanum wheels
//!
//! \return none
//
//*****************************************************************************
void fw_motors(uint8_t duty, uint8_t num_cells);
//*****************************************************************************
//
//! Moves the robot in the foward direction
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of cells to be moved
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
void cw_motors(uint8_t duty, uint8_t num_turns);
//*****************************************************************************
//
//! Turn Clockwise
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of 90 degree turns intended
//!
//! This function turns the robot clockwise
//!
//! \return none
//
//*****************************************************************************
void ccw_motors(uint8_t duty, uint8_t num_turns);
//*****************************************************************************
//
//! Counter clockwise turn
//!
//! \param ui8duty is the intended duty cycle
//!
//! \param ui8num_turns is the number of 90 degree turns intended
//!
//! This function turns the robot counter clockwise
//!
//! \return none
//
//*****************************************************************************
void fwd_pid(int32_t ufwd,int32_t uback, uint8_t dutyref);
//*****************************************************************************
//
//! fwd_pid - forward PID movement
//!
//! \param ufwd is the controller effort for front wheels
//!
//! \param uback is the controller effort for rear wheels
//!
//! \param dutyref is the reference duty cycle
//!
//! This function adjusts the duty cycle according to controller effort
//!
//! \return none
//
//*****************************************************************************
void rev_pid(int32_t ufwd,int32_t uback, uint8_t dutyref);
//*****************************************************************************
//
//! rev_pid - Reverse PID movement
//!
//! \param u is the controller effort
//!
//! \param dutyref is the reference duty cycle
//!
//! This function adjusts the duty cycle according to controller effort
//!
//! \return none
//
//*****************************************************************************
void tl_pid(int32_t uleft,int32_t uright, uint8_t dutyref);
//*****************************************************************************
//
//! tl_pid - translational left PID movement
//!
//! \param u is the controller effort
//!
//! \param dutyref is the reference duty cycle
//!
//! This function adjusts the duty cycle according to controller effort
//!
//! \return none
//
//*****************************************************************************
void tr_pid(int32_t uleft,int32_t uright, uint8_t dutyref);
//*****************************************************************************
//
//! tr_pid - translational right PID movement
//!
//! \param u is the controller effort
//!
//! \param dutyref is the reference duty cycle
//!
//! This function adjusts the duty cycle according to controller effort
//!
//! \return none
//
//*****************************************************************************
void servo_parallel(void);
//*****************************************************************************
//
//! Lift servos parallel to ground
//!
//! \param none
//!
//! \return none
//
//*****************************************************************************
void servo_tangent(void);
//*****************************************************************************
//
//! Lift servos tangent to ground
//!
//! \param none
//!
//! \return none
//
//*****************************************************************************
void control_effort_limit(int8_t * DCmotors);
//*****************************************************************************
//
//! Limits Duty cycle on motors to upper and lower limits defined in this file
//!
//! \param int8_t * DCmotors - Pointer to motor array representation 0th order
//!
//! being m1, 1st order being m2, and so on...
//!
//! \return none
//
//*****************************************************************************
void setDC(int8_t * DCmotors);
//*****************************************************************************
//
//! Set duty cycle to motors
//!
//! \param int8_t * DCmotors - Pointer to motor array representation 0th order
//!
//! being m1, 1st order being m2, and so on...
//!
//! \return none
//
//*****************************************************************************
void setPWMperiod(uint32_t ui32Base, uint32_t ui32Gen, uint32_t ui32Period);
//*****************************************************************************
//
//! Set PWM load value - replacement for faulty function
//!
//! \param uint32_t ui32Base - Base address for PWM module
//!
//! \param uint32_t ui32Gen - Gen offset
//!
//! \param uint32_t ui32Period - Period value to be loaded
//!
//! \return none
//
//*****************************************************************************
#endif /* MOTORS_H_ */
