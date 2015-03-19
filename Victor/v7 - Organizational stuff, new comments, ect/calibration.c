/*
 * calibration.c
 *
 *  Created on: Feb 25, 2015
 *      Author: Wicho
 */
#include "calibration.h"
/*
 * Global Variables
 */
uint32_t t_0 = 0;																// Placeholder for change in time value
uint32_t delta_arr[4][3][2] = {0};												// Mins and maxes of each light sensor
/*
 * Extern Variables
 */
extern uint16_t lightsnsr_prd[4];
extern int32_t lightsnsr_val[4];
extern uint8_t  lightsnsr_used;
//
#include "calibration.h"
void calibrate_lightsensor(void)
{
	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);								// Enable Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);								// Enable Wide timer 0
	TimerClockSourceSet(TIMER5_BASE, TIMER_CLOCK_SYSTEM);						// Set Timer clock sources
	TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);						// Set clock source for WTIMER0
	TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP);// Configure Timer5A as a one-shot timer
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP);// Configure WTimer0A as a 32bit periodic timer

	// Load period values
	uint32_t Clock = 80000000;													// SysCtlClockGet(); does not work
	uint32_t Period = ((Clock * timer5A_prd) / fixedpoint_microsec_coeff);		// set periods (us)
	TimerLoadSet(TIMER5_BASE, TIMER_A, Period - 1);
	Period = ((Clock * Wtimer0_prd) /  fixedpoint_millisec_coeff);				// ms (Wtimer0_prd cannot exceed 10... or overflow occurs)
	TimerLoadSet(WTIMER0_BASE, TIMER_A, Period - 1);

	// Enable Interrupts
	IntEnable(INT_TIMER5A);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);							// Interrupt at timeout
	IntEnable(INT_WTIMER0A);
	TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);							// Interrupt at timeout

	// Unlock PD6 and PD7 from Intrinsic NMI Function for lightsensor use
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Set GPIO pins to interrupt
	GPIOIntEnable(LIGHTSNSR1_BASE, LIGHTSNSR1_INT);
	GPIOIntEnable(LIGHTSNSR2_BASE, LIGHTSNSR2_INT);
	GPIOIntEnable(LIGHTSNSR3A_BASE, LIGHTSNSR3A_INT);
	GPIOIntEnable(LIGHTSNSR3B_BASE, LIGHTSNSR3B_INT);
	GPIOIntEnable(LIGHTSNSR4A_BASE, LIGHTSNSR4A_INT);
	GPIOIntEnable(LIGHTSNSR4B_BASE, LIGHTSNSR4B_INT);

	// Set interrupt type to falling edge (consult discrete interrupts too)
	GPIOIntTypeSet(LIGHTSNSR1_BASE, LIGHTSNSR1, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR2_BASE, LIGHTSNSR2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3A_BASE, LIGHTSNSR3A, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3B_BASE, LIGHTSNSR3B, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4A_BASE, LIGHTSNSR4A, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4B_BASE, LIGHTSNSR4B, GPIO_FALLING_EDGE);

	// Start calibration
	lightsensorcharge();														// Charge line sensors
	//TimerEnable(TIMER5_BASE, TIMER_A);											// Wait for charge up
	TimerEnable(WTIMER0_BASE, TIMER_A);											// Start periodic sample timer
}
void timer5A_ISR(void)
{
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);								// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);							// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
	GPIOPinTypeGPIOInput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
	GPIOPinTypeGPIOInput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
	GPIOPinTypeGPIOInput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
	t_0 = TimerValueGet(WTIMER0_BASE, TIMER_A);									// Take first timed value
}
void Wtimer0A_ISR(void)
{
	TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);							// Clear interrupt
	lightsensorcharge();														// Charge linesensors
	TimerEnable(TIMER5_BASE, TIMER_A);											// Wait for charge up
}
void gpio_A_ISR(void)
{
	uint32_t int_mask = (GPIOIntStatus(GPIO_PORTA_BASE, 1));					// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTA_BASE, int_mask);									// Clear interrupt
	uint32_t delta_t = TimerValueGet(WTIMER0_BASE, TIMER_A) - t_0;				// Find time difference
	if(int_mask & LIGHTSNSR1_1_INT) min_and_max(1, 1, delta_t, delta_arr);		// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR1_2_INT) min_and_max(1, 2, delta_t, delta_arr);
	if(int_mask & LIGHTSNSR1_3_INT) min_and_max(1, 3, delta_t, delta_arr);
}
void gpio_B_ISR(void)
{
	uint32_t int_mask = GPIOIntStatus(GPIO_PORTB_BASE, 1);						// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTB_BASE, int_mask);									// Clear interrupt
	uint32_t delta_t = TimerValueGet(WTIMER0_BASE, TIMER_A) - t_0;				// Find time difference
	if(int_mask & LIGHTSNSR4A_1_INT) min_and_max(4, 1, delta_t, delta_arr);		// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR4A_2_INT) min_and_max(4, 2, delta_t, delta_arr);
}
void gpio_C_ISR(void)
{
	uint32_t int_mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);						// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTC_BASE, int_mask);									// Clear interrupt
	uint32_t delta_t = TimerValueGet(WTIMER0_BASE, TIMER_A) - t_0;				// Find time difference
	if(int_mask & LIGHTSNSR2_1_INT) min_and_max(2, 1, delta_t, delta_arr);		// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR2_2_INT) min_and_max(2, 2, delta_t, delta_arr);
	if(int_mask & LIGHTSNSR2_3_INT) min_and_max(2, 3, delta_t, delta_arr);
	if(int_mask & LIGHTSNSR4B_3_INT) min_and_max(4, 3, delta_t, delta_arr);
}
void gpio_D_ISR(void)
{
	uint32_t int_mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);						// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTC_BASE, int_mask);									// Clear interrupt
	uint32_t delta_t = TimerValueGet(WTIMER0_BASE, TIMER_A) - t_0;				// Find time difference
	if(int_mask & LIGHTSNSR3A_1_INT) min_and_max(3, 1, delta_t, delta_arr);		// Set mins/maxes for each sensor array
	if(int_mask & LIGHTSNSR3A_2_INT) min_and_max(3, 2, delta_t, delta_arr);
}
void gpio_F_ISR(void)
{
	uint32_t int_mask = GPIOIntStatus(GPIO_PORTC_BASE, 1);						// Find which gpio pin interrupted
	GPIOIntClear(GPIO_PORTC_BASE, int_mask);									// Clear interrupt
	uint32_t delta_t = TimerValueGet(WTIMER0_BASE, TIMER_A) - t_0;				// Find time difference
	if(int_mask & LIGHTSNSR3B_3_INT) min_and_max(3, 3, delta_t, delta_arr);		// Set mins/maxes for each sensor array
}
void lightsensorcharge(void)
{
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
}
void min_and_max(uint8_t index_sensor, uint8_t index_pin, uint32_t delta_val, uint32_t (*delta)[3][2])
{
	if((delta[index_sensor-1][index_pin-1][min] == 0)|(delta_val < delta[index_sensor-1][index_pin-1][min])) delta[index_sensor-1][index_pin-1][min] = delta_val;
	if(delta_val > delta[index_sensor-1][index_pin-1][max]) delta[index_sensor-1][index_pin-1][max] = delta_val;
}
