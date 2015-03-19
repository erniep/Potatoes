/*
 * light_sensor.c
 *
 *  Created on: Jan 21, 2015
 *      Author: Wicho
 */
#include "light_sensor.h"
#include "calibration.h"
/*
 * Global Variables
 */
uint32_t lightsnsr_val[4];														// Lightsensor values read
uint16_t lightsnsr_prd[4];														// Lightsensor periods
uint8_t  lightsnsr_flag = 0;													// Lightsensor flag to check if all 4 are ready

void Robot_lightsnsr_task(void)
{
	calibrate_lightsensor();
	// Pend on calibration semaphore
	Semaphore_pend(Sema_lightsense_calibrate, BIOS_WAIT_FOREVER);

	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);								// Enable Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);						// Set Timer clock sources
	TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_SYSTEM);
	TimerClockSourceSet(TIMER4_BASE, TIMER_CLOCK_SYSTEM);

	// Configure Timer2, Timer4, and Timer3A as one-shot timers
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP | TIMER_CFG_B_ONE_SHOT_UP);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP | TIMER_CFG_B_ONE_SHOT_UP);

	// Unlock PD6 and PD7 from Intrinsic NMI Function for lightsensor use
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Load example threshold values into array to be replaced with calibration code :)
	lightsnsr_prd[0] = thresh1;
	lightsnsr_prd[1] = thresh2;
	lightsnsr_prd[2] = thresh3;
	lightsnsr_prd[3] = thresh4;


	// Load period values of each light sensor into reg
	uint32_t Clock = 80000000;													//SysCtlClockGet(); does not work
	uint32_t Period = ((Clock * timer2A_prd) / fixedpoint_microsec_coeff);		// set periods
	TimerLoadSet(TIMER2_BASE, TIMER_A, (uint16_t)Period -1);
	Period = ((Clock / fixedpoint_microsec_coeff) * lightsnsr_prd[0]);
	TimerLoadSet(TIMER2_BASE, TIMER_B, (uint16_t)Period -1);
	Period = ((Clock / fixedpoint_microsec_coeff) * lightsnsr_prd[1]);
	TimerLoadSet(TIMER3_BASE, TIMER_A, (uint16_t)Period -1);
	Period = ((Clock / fixedpoint_microsec_coeff) * lightsnsr_prd[2]);
	TimerLoadSet(TIMER4_BASE, TIMER_A, (uint16_t)Period -1);
	Period = ((Clock / fixedpoint_microsec_coeff) * lightsnsr_prd[3]);
	TimerLoadSet(TIMER4_BASE, TIMER_B, (uint16_t)Period -1);

	IntEnable(INT_TIMER2A);														//Enable Interrupts
	IntEnable(INT_TIMER2B);
	IntEnable(INT_TIMER3A);
	IntEnable(INT_TIMER4A);
	IntEnable(INT_TIMER4B);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);

	// Begin Task
	while(TRUE)
	{
		// Pend on semaphore - Sample frequency
		Semaphore_pend(Sema_lightsense_f, BIOS_WAIT_FOREVER);
		GPIOPinTypeGPIOOutput(LIGHTSNSR1_BASE, LIGHTSNSR1);						// Set lightsense GPIO to output
		GPIOPinTypeGPIOOutput(LIGHTSNSR2_BASE, LIGHTSNSR2);
		GPIOPinTypeGPIOOutput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
		GPIOPinTypeGPIOOutput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
		GPIOPinTypeGPIOOutput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
		GPIOPinTypeGPIOOutput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
		GPIOPinWrite(LIGHTSNSR1_BASE, LIGHTSNSR1, LIGHTSNSR1);					// Set lightsense pins hi
		GPIOPinWrite(LIGHTSNSR2_BASE, LIGHTSNSR2, LIGHTSNSR2);
		GPIOPinWrite(LIGHTSNSR3A_BASE, LIGHTSNSR3A, LIGHTSNSR3A);
		GPIOPinWrite(LIGHTSNSR3B_BASE, LIGHTSNSR3B, LIGHTSNSR3B);
		GPIOPinWrite(LIGHTSNSR4A_BASE, LIGHTSNSR4A, LIGHTSNSR4A);
		GPIOPinWrite(LIGHTSNSR4B_BASE, LIGHTSNSR4B, LIGHTSNSR4B);
		TimerEnable(TIMER2_BASE, TIMER_A);										// Start sampling
	}
}
void timer2A_ISR(void)
{

	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);								// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);							// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
	GPIOPinTypeGPIOInput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
	GPIOPinTypeGPIOInput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
	GPIOPinTypeGPIOInput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
	// Enable timers
	TimerEnable(TIMER2_BASE, TIMER_B);
	TimerEnable(TIMER3_BASE, TIMER_A);
	TimerEnable(TIMER4_BASE, TIMER_A);
	TimerEnable(TIMER4_BASE, TIMER_B);
}
void timer2B_ISR(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMB_TIMEOUT);								// Clear interrupt
	// Grab value from GPIO port for lightsensor, and store in global variable
	lightsnsr_val[0] = GPIOPinRead(LIGHTSNSR1_BASE, LIGHTSNSR1) >> 2;			// Read light sensors and update global vars
	lightsnsr_val[0] = lightsense_bit_reverse(lightsnsr_val[0]);				// Reverse bits because lightsensor is backwards
	lightsnsr_flag |= 0x1;														// Lightsensor 1 is ready
	lightsense_check();															// Are all light sensors ready?
}
void timer3A_ISR(void)
{
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);								// Clear interrupt
	// Grab value from GPIO port for lightsensor, and store in global variable
	lightsnsr_val[1] = GPIOPinRead(LIGHTSNSR2_BASE, LIGHTSNSR2) >> 5;			// Read light sensors and update global vars
	lightsnsr_flag |= 0x2;														// Lightsensor 2 is ready
	lightsense_check();															// Are all light sensors ready?
}
void timer4A_ISR(void)
{
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);								// Clear interrupt
	// Grab value from GPIO port for lightsensor, and store in global variable
	lightsnsr_val[2] = (GPIOPinRead(LIGHTSNSR3A_BASE, LIGHTSNSR3A) >> 6)|(GPIOPinRead(LIGHTSNSR3B_BASE, LIGHTSNSR3B) >> 2);
	lightsnsr_flag |= 0x4;														// lightsensor 3 is ready
	lightsense_check();															// Are all light sensors ready?
}
void timer4B_ISR(void)
{
	TimerIntClear(TIMER4_BASE, TIMER_TIMB_TIMEOUT);								// Clear interrupt
	// Grab value from GPIO port for lightsensor, and store in global variable
	lightsnsr_val[3] = (GPIOPinRead(LIGHTSNSR4A_BASE, LIGHTSNSR4A) >> 2)|(GPIOPinRead(LIGHTSNSR4B_BASE, LIGHTSNSR4B) >> 2);
	lightsnsr_flag |= 0x8;														// lightsensor 4 is ready
	lightsense_check();															// Are all light sensors ready?
}
void lightsense_CLK(void)
{
		Semaphore_post(Sema_lightsense_f);										// Post lightsensor semaphore at a given sampling frequency
}
uint32_t lightsense_bit_reverse(uint32_t lightsensor_val)
{
	uint8_t bit1 = lightsensor_val & 0x2;										// Take 2nd bit and store into a value
	uint8_t bit0 = lightsensor_val >> 2;										// Shift value right twice so we only have the 3rd bit in the 1st bit placeholder
	uint8_t bit2 = (lightsensor_val << 2) & 0x4;								// Shift left twice, then bitmask so we have the 1st bit in the 3rd bit placeholder
	return lightsensor_val = bit0 | bit1 | bit2;								// Bitwise or values to complete bit reversing, and return the value
}
void lightsense_check(void)
{
	if(lightsnsr_flag == 0xF)													// are all lightsensors ready?
	{
		lightsnsr_flag = 0;														// if so, Clear data set flag
		Semaphore_post(Sema_lightsense);										// and Post data rdy Semaphore
	}
}
