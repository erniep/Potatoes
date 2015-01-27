/*
 * light_sensor.c
 *
 *  Created on: Jan 21, 2015
 *      Author: Wicho
 */
#include "light_sensor.h"
/*
 * Global Variables
 */
uint8_t lightsnsr_val1 = 0;
uint8_t lightsnsr_val2 = 0;
uint8_t lightsnsr_val3 = 0;
uint8_t lightsnsr_val4 = 0;

void Robot_lightsnsr_task(void)
{
	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);																		// Enable Timer clock source
	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);																// Set Timer clock source
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP | TIMER_CFG_B_ONE_SHOT_UP);				// Configure Timer2 as two 16 bit timers
	uint32_t Period = ((SysCtlClockGet() * timer2A_prd) / fixedpoint_microsec_coeff);									// set periods
	TimerLoadSet(TIMER2_BASE, TIMER_A, (uint16_t)Period -1);
	Period = ((SysCtlClockGet() * timer2B_prd) / fixedpoint_microsec_coeff);
	TimerLoadSet(TIMER2_BASE, TIMER_B, (uint16_t)Period -1);															// Enable interrupts for Timer2A and Timer2B
	IntEnable(INT_TIMER2A);
	IntEnable(INT_TIMER2B);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
	// Begin Task
	while(TRUE)
	{
		// Pend on semaphore - Sample frequency
		Semaphore_pend(Sema_lightsense_f, BIOS_WAIT_FOREVER);
		// Set lightsense GPIO to output
		GPIOPinTypeGPIOOutput(LIGHTSNSR1_BASE, LIGHTSNSR1);
		GPIOPinTypeGPIOOutput(LIGHTSNSR2_BASE, LIGHTSNSR2);
		GPIOPinTypeGPIOOutput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
		GPIOPinTypeGPIOOutput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
		GPIOPinTypeGPIOOutput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
		GPIOPinTypeGPIOOutput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
		// Set lightsense pins hi
		GPIOPinWrite(LIGHTSNSR1_BASE, LIGHTSNSR1, LIGHTSNSR1);
		GPIOPinWrite(LIGHTSNSR2_BASE, LIGHTSNSR2, LIGHTSNSR2);
		GPIOPinWrite(LIGHTSNSR3A_BASE, LIGHTSNSR3A, LIGHTSNSR3A);
		GPIOPinWrite(LIGHTSNSR3B_BASE, LIGHTSNSR3B, LIGHTSNSR3B);
		GPIOPinWrite(LIGHTSNSR4A_BASE, LIGHTSNSR4A, LIGHTSNSR4A);
		GPIOPinWrite(LIGHTSNSR4B_BASE, LIGHTSNSR4B, LIGHTSNSR4B);
		TimerEnable(TIMER2_BASE, TIMER_A);																				// Enable Timer2A
		// Pend on semaphore - Data Collection Complete
		Semaphore_pend(Sema_lightsense, BIOS_WAIT_FOREVER);
	}
}
void timer2A_ISR(void)
{

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);																		// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);																	// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
	GPIOPinTypeGPIOInput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
	GPIOPinTypeGPIOInput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
	GPIOPinTypeGPIOInput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
	TimerEnable(TIMER2_BASE, TIMER_B);																					// Enable Timer 2B
}
void timer2B_ISR(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);																		// Clear interrupt
	lightsnsr_val1 = GPIOPinRead(LIGHTSNSR1_BASE, LIGHTSNSR1) >> 2;														// Read light sensors and update global vars
	lightsnsr_val2 = GPIOPinRead(LIGHTSNSR2_BASE, LIGHTSNSR2) >> 5;
	lightsnsr_val3 = (GPIOPinRead(LIGHTSNSR3A_BASE, LIGHTSNSR3A) >> 6)|(GPIOPinRead(LIGHTSNSR3B_BASE, LIGHTSNSR3B) >> 2);
	lightsnsr_val4 = (GPIOPinRead(LIGHTSNSR4A_BASE, LIGHTSNSR4A) >> 2)|(GPIOPinRead(LIGHTSNSR4B_BASE, LIGHTSNSR4B) >> 1);
	Semaphore_post(Sema_lightsense);																					// Post data rdy Semaphore
}
void lightsense_CLK(void)
{
	Semaphore_post(Sema_lightsense_f);																					// Post sema if needed
}
