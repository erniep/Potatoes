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
	calibrate_lightsensor_init();
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

	IntEnable(INT_TIMER5A);
	IntEnable(INT_TIMER5B);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMB_TIMEOUT | TIMER_TIMA_TIMEOUT);

	// Start Sense
	TimerEnable(TIMER1_BASE, TIMER_B);
}
void calibrate_lightsensor_init(void)
{
	lightsnsr_used = 1;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);
	TimerClockSourceSet(TIMER5_BASE, TIMER_CLOCK_SYSTEM);																// Set Timer clock source
	TimerConfigure(TIMER5_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC_UP);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT_UP);

	uint32_t Clock = 80000000;
	uint32_t Period = ((Clock * timer1B_prd) / fixedpoint_microsec_coeff);									// set periods
	TimerLoadSet(TIMER1_BASE, TIMER_B, (uint16_t)Period -1);
	Period = ((Clock * timer5B_prd) / fixedpoint_microsec_coeff);											// set periods
	TimerLoadSet(TIMER5_BASE, TIMER_B, (uint16_t)Period -1);

	// Clear periods
	lightsnsr_prd[0] = 0;
	lightsnsr_prd[1] = 0;
	lightsnsr_prd[2] = 0;
	lightsnsr_prd[3] = 0;
}
void timer1B_ISR(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);																		// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);																	// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3A_BASE, LIGHTSNSR3A);
	GPIOPinTypeGPIOInput(LIGHTSNSR3B_BASE, LIGHTSNSR3B);
	GPIOPinTypeGPIOInput(LIGHTSNSR4A_BASE, LIGHTSNSR4A);
	GPIOPinTypeGPIOInput(LIGHTSNSR4B_BASE, LIGHTSNSR4B);
	TimerEnable(TIMER5_BASE, TIMER_B);
}
void timer5B_ISR(void)
{
	TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT);																		// Clear interrupt
	lightsnsr_val[0] = GPIOPinRead(LIGHTSNSR1_BASE, LIGHTSNSR1) >> 2;
	//lightsnsr_val[1] = GPIOPinRead(LIGHTSNSR2_BASE, LIGHTSNSR2) >> 5;
	//lightsnsr_val[2] = (GPIOPinRead(LIGHTSNSR3A_BASE, LIGHTSNSR3A) >> 6)|(GPIOPinRead(LIGHTSNSR3B_BASE, LIGHTSNSR3B) >> 2);
	//lightsnsr_val[3] = (GPIOPinRead(LIGHTSNSR4A_BASE, LIGHTSNSR4A) >> 2)|(GPIOPinRead(LIGHTSNSR4B_BASE, LIGHTSNSR4B) >> 2);
	if(!(lightsnsr_val[0] /*| lightsnsr_val[1] | lightsnsr_val[2] | lightsnsr_val[3]*/))
	{
		TimerDisable(TIMER5_BASE, TIMER_B);
		//lightsnsr_prd[0] = (lightsnsr_prd[0]);
		//lightsnsr_prd[1] = (lightsnsr_prd[1] * k_num)/k_den;
		//lightsnsr_prd[2] = (lightsnsr_prd[2] * k_num)/k_den;
		//lightsnsr_prd[3] = (lightsnsr_prd[3] * k_num)/k_den;
		lightsnsr_used = 0;
		Semaphore_post(Sema_lightsense_calibrate);
	}
	else
	{
		if(lightsnsr_val[0]) lightsnsr_prd[0]+=5;
		//if(lightsnsr_val[1]) lightsnsr_prd[1]++;
		//if(lightsnsr_val[2]) lightsnsr_prd[2]++;
		//if(lightsnsr_val[3]) lightsnsr_prd[3]++;
	}
}
