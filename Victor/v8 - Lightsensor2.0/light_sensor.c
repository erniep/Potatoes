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
uint32_t lightsnsr_val[4];																								// Value read after threshold compare
uint32_t lightsnsr2_val[4][3];																							// Holds values for periods measured
uint32_t t_0 = 0;																										// t = 0
uint32_t t_1 = 0;																										// t = T, where T is sampling period
uint64_t delta_t = 0;																									// Change in time placeholder
uint16_t lightsnsr_thresh[4];																							// Threshold compare values

void Robot_lightsnsr_task(void)
{
	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);																		// Enable Timer clock source
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);																// Set Timer clock source
	TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP | TIMER_CFG_B_ONE_SHOT_UP);				// Configure Timer2 as two 16 bit timers
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP);										// Configure WTimer0A as a 32bit periodic timer

	// Unlock PD6 and PD7 from Intrinsic NMI Function
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Load example threshold values into array later to be calibrated
	lightsnsr_prd[0] = thresh1;
	lightsnsr_prd[1] = thresh2;
	lightsnsr_prd[2] = thresh3;
	lightsnsr_prd[3] = thresh4;

	// Load period values of each light sensor into reg																		//SysCtlClockGet();
	uint32_t Period = ((Clock * timer2A_prd) / fixedpoint_microsec_coeff);								// set periods
	TimerLoadSet(TIMER2_BASE, TIMER_A, (uint16_t)Period -1);											// Enable interrupts for Timer2A and Timer2B

	//Enable Interrupts
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);

	// Set GPIO pins to interrupt
	GPIOIntEnable(LIGHTSNSR1_BASE, LIGHTSNSR1_INT);
	//GPIOIntEnable(LIGHTSNSR2_BASE, LIGHTSNSR2_INT);
	//GPIOIntEnable(LIGHTSNSR3A_BASE, LIGHTSNSR3A_INT);
	//GPIOIntEnable(LIGHTSNSR3B_BASE, LIGHTSNSR3B_INT);
	//GPIOIntEnable(LIGHTSNSR4A_BASE, LIGHTSNSR4A_INT);
	//GPIOIntEnable(LIGHTSNSR4B_BASE, LIGHTSNSR4B_INT);
	// Set interrupt type to falling edge (consult discrete interrupts in future)
	GPIOIntTypeSet(LIGHTSNSR1_BASE, LIGHTSNSR1, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR2_BASE, LIGHTSNSR2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3A_BASE, LIGHTSNSR3A, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3B_BASE, LIGHTSNSR3B, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4A_BASE, LIGHTSNSR4A, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4B_BASE, LIGHTSNSR4B, GPIO_FALLING_EDGE);

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
		TimerEnable(WTIMER0_BASE, TIMER_A);																				// Enable WTimer0A
	}
}
void timer2A_ISR(void)
{
	t_0 = HWREG(WTIMER0_BASE + TIMER_O_TAR);																			// Grab Timer Value
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);																		// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);																	// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
	GPIOPinTypeGPIOInput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
	GPIOPinTypeGPIOInput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
	GPIOPinTypeGPIOInput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
}
void gpio_A_ISR(void)
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTA_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTA_BASE, int_mask);									// Clear interrupt
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR1_1_INT) lightsnsr2_val[0][0] = (delta_t * fixedpoint_microsec_coeff) / Clock;
	if(int_mask & LIGHTSNSR1_2_INT) lightsnsr2_val[0][1] = (delta_t * fixedpoint_microsec_coeff) / Clock;
	if(int_mask & LIGHTSNSR1_3_INT) lightsnsr2_val[0][2] = (delta_t * fixedpoint_microsec_coeff) / Clock;
}
void gpio_B_ISR(void)
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTA_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTB_BASE, int_mask);									// Clear interrupt
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR4A_1_INT) lightsnsr2_val[3][0] = delta_t;				// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR4A_2_INT) lightsnsr2_val[3][1] = delta_t;
}
void gpio_C_ISR(void)
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTC_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTC_BASE, int_mask);									// Clear interrupt
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR2_1_INT) lightsnsr2_val[1][0] = delta_t;			// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR2_2_INT) lightsnsr2_val[1][1] = delta_t;
	if(int_mask & LIGHTSNSR2_3_INT) lightsnsr2_val[1][2] = delta_t;
	if(int_mask & LIGHTSNSR4B_3_INT) lightsnsr2_val[3][2] = delta_t;
}
void gpio_D_ISR(void)
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTD_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTD_BASE, int_mask);									// Clear interrupt
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR3A_1_INT) lightsnsr2_val[2][0] = delta_t;			// Set delta values for each array
	if(int_mask & LIGHTSNSR3A_2_INT) lightsnsr2_val[2][1] = delta_t;
}
void gpio_F_ISR(void)
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTF_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTF_BASE, int_mask);									// Clear interrupt
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR3B_3_INT) lightsnsr2_val[2][2] = delta_t;			// Set delta values for each sensor array
}
void lightsense_CLK(void)
{
		Semaphore_post(Sema_lightsense_f);										// Post sema if needed
}
