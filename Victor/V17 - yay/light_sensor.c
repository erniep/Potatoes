/*
 * light_sensor.c
 *
 *  Created on: Jan 21, 2015
 *      Author: Wicho
 */
#include "light_sensor.h"
/*
 * This define statement turns the median filter on and off,
 * Comment line to remove filter
 */
#define MEDIAN_FILTER_ON 1
/*
 * Global Variables
 */																														// Threshold compare values
uint32_t raw_prd[NUM_ARRAYS][NUM_SENSORS][NUM_SAMPLES] = {0};															// Holds values for periods measured																			// Holds values for periods measured
uint32_t prd[NUM_ARRAYS][NUM_SENSORS] = {0};																			// Holds values for periods filtered
uint32_t t_0 = 0;																										// t = 0
uint32_t t_1 = 0;																										// t = T, where T is sampling period
uint64_t delta_t = 0;																									// Change in time placeholder
uint16_t lightsense_flag = 0;

void Robot_lightsnsr_task(void)
{
	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);																		// Enable Timer clock source
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);																// Set Timer clock source
	TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP | TIMER_CFG_B_ONE_SHOT_UP);				// Configure Timer2 as two 16 bit timers
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC_UP);										// Configure WTimer0A as a 32bit periodic timer

	// Unlock PD7 from Intrinsic NMI Function
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Load period values of each light sensor into reg
	uint32_t Period = ((Clock * CHARGE_PRD) / fixedpoint_microsec_coeff);								// set period
	TimerLoadSet(TIMER2_BASE, TIMER_A, (uint16_t)Period -1);											// Enable interrupts for Timer2A

	//Enable Interrupts
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);

	// Set GPIO pins to interrupt
	GPIOIntEnable(LIGHTSNSR1_BASE, LIGHTSNSR1_INT);
	GPIOIntEnable(LIGHTSNSR2_BASE, LIGHTSNSR2_INT);
	GPIOIntEnable(LIGHTSNSR3_BASE, LIGHTSNSR3_INT);
	GPIOIntEnable(LIGHTSNSR4_BASE, LIGHTSNSR4_INT);
	GPIOIntEnable(LIGHTSNSR5_BASE, LIGHTSNSR5_INT);
	GPIOIntEnable(LIGHTSNSR6_BASE, LIGHTSNSR6_INT);
	// Set interrupt type to falling edge (consult discrete interrupts in future)
	GPIOIntTypeSet(LIGHTSNSR1_BASE, LIGHTSNSR1, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR2_BASE, LIGHTSNSR2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3_BASE, LIGHTSNSR3, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4_BASE, LIGHTSNSR4, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR5_BASE, LIGHTSNSR5, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR6_BASE, LIGHTSNSR6, GPIO_FALLING_EDGE);

	TimerEnable(WTIMER0_BASE, TIMER_A);																	// Enable WTimer0A

	// Begin Task
	while(TRUE)
	{
		// Pend on semaphore - Sample frequency
		Semaphore_pend(Sema_lightsense_f, BIOS_WAIT_FOREVER);
		// Set lightsense GPIO to output
		GPIOPinTypeGPIOOutput(LIGHTSNSR1_BASE, LIGHTSNSR1);
		GPIOPinTypeGPIOOutput(LIGHTSNSR2_BASE, LIGHTSNSR2);
		GPIOPinTypeGPIOOutput(LIGHTSNSR3_BASE, LIGHTSNSR3);
		GPIOPinTypeGPIOOutput(LIGHTSNSR4_BASE, LIGHTSNSR4);
		GPIOPinTypeGPIOOutput(LIGHTSNSR5_BASE, LIGHTSNSR5);
		GPIOPinTypeGPIOOutput(LIGHTSNSR6_BASE, LIGHTSNSR6);
		// Set lightsense pins hi
		GPIOPinWrite(LIGHTSNSR1_BASE, LIGHTSNSR1, LIGHTSNSR1);
		GPIOPinWrite(LIGHTSNSR2_BASE, LIGHTSNSR2, LIGHTSNSR2);
		GPIOPinWrite(LIGHTSNSR3_BASE, LIGHTSNSR3, LIGHTSNSR3);
		GPIOPinWrite(LIGHTSNSR4_BASE, LIGHTSNSR4, LIGHTSNSR4);
		GPIOPinWrite(LIGHTSNSR5_BASE, LIGHTSNSR5, LIGHTSNSR5);
		GPIOPinWrite(LIGHTSNSR6_BASE, LIGHTSNSR6, LIGHTSNSR6);
		lightsense_flag = 0;																							// Reset flag for lightsense completion
		TimerEnable(TIMER2_BASE, TIMER_A);																				// Enable Timer2A
	}
}
void median_filter_task(void)
{
#ifdef MEDIAN_FILTER_ON
	uint32_t temp_arr[NUM_SAMPLES] = {0};
	int8_t k = 0;
#endif
	uint8_t n = 0;
	uint8_t m = 0;
	while(1)
	{
		// Pend on semaphore - Raw Data ready
		Semaphore_pend(Sema_lightsense_filter, BIOS_WAIT_FOREVER);
		for(n=0; n<(NUM_ARRAYS); n++)
		{
			for(m=0; m<(NUM_SENSORS); m++)
			{
#ifdef MEDIAN_FILTER_ON																									// A preprocessor to turn on the median filter
				memcpy(temp_arr, raw_prd[n][m], 4*NUM_SAMPLES);															// Copy Raw data into a temporary sorting array
				sort_arr(temp_arr);																						// Sort array - If NUM_SAMPLES > 3 change this code
				if(NUM_SAMPLES % 2 ==0) prd[n][m] = (temp_arr[NUM_SAMPLES/2] + temp_arr[NUM_SAMPLES/2 - 1])/2;
				else prd[n][m] = temp_arr[NUM_SAMPLES/2];																// Grab median value - if NUM_SAMPLES is even change this line
				raw_prd[n][m][2] = raw_prd[n][m][1];
				raw_prd[n][m][1] = raw_prd[n][m][0];
				for(k = (NUM_SAMPLES-1); k <= 0; k--)																	// Update Raw data for next iteration
				{
					raw_prd[n][m][k] = raw_prd[n][m][k-1];
				}
#else
				prd[n][m] = raw_prd[n][m][0];
#endif
			}
		}
		// Post Semaphore - Filtered Data ready
		Semaphore_post(Sema_lightsense);
	}
}
void timer2A_ISR(void)
{
	t_0 = HWREG(WTIMER0_BASE + TIMER_O_TAR);																			// Grab Timer Value
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);																		// Clear interrupt
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);																	// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIOPinTypeGPIOInput(LIGHTSNSR3_BASE, LIGHTSNSR3);
	GPIOPinTypeGPIOInput(LIGHTSNSR4_BASE, LIGHTSNSR4);
	GPIOPinTypeGPIOInput(LIGHTSNSR5_BASE, LIGHTSNSR5);
	GPIOPinTypeGPIOInput(LIGHTSNSR6_BASE, LIGHTSNSR6);
}
void gpio_A_ISR(void)															// 0_0, 0_1, 0_2
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTA_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTA_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR1_1_INT)
	{
		raw_prd[0][0][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[0][0][0] > MAX_PERIOD) raw_prd[0][0][0] = MAX_PERIOD;
		lightsense_flag |= BIT0;
	}
	if(int_mask & LIGHTSNSR1_2_INT)
	{
		raw_prd[0][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[0][1][0] > MAX_PERIOD) raw_prd[0][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT1;
	}
	if(int_mask & LIGHTSNSR1_3_INT)
	{
		raw_prd[0][2][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[0][2][0] > MAX_PERIOD) raw_prd[0][2][0] = MAX_PERIOD;
		lightsense_flag |= BIT2;
	}
	if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);		// Check for completion
}
void gpio_B_ISR(void)															// 3_1, 5_1
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTB_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTB_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR4_INT)
	{
		raw_prd[3][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[3][1][0] > MAX_PERIOD) raw_prd[3][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT3;
	}
	if(int_mask & LIGHTSNSR6_INT)
	{
		raw_prd[5][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[5][1][0] > MAX_PERIOD) raw_prd[5][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT4;
	}
	if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);		// Check for completion
}
void gpio_C_ISR(void)															//1_0,1_1,1_2
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTC_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTC_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR2_1_INT)
	{
		raw_prd[1][0][0] = (delta_t / fixedpoint_count_to_time);				// Put new value into result matrix
		if(raw_prd[1][0][0] > MAX_PERIOD) raw_prd[1][0][0] = MAX_PERIOD;
		lightsense_flag |= BIT5;												// Adjust completion flag
	}																			// Repeat for rest of sensors
	if(int_mask & LIGHTSNSR2_2_INT)
	{
		raw_prd[1][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[1][1][0] > MAX_PERIOD) raw_prd[1][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT6;
	}
	if(int_mask & LIGHTSNSR2_3_INT)
	{
		raw_prd[1][2][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[1][2][0] > MAX_PERIOD) raw_prd[1][2][0] = MAX_PERIOD;
		lightsense_flag |= BIT7;
	}
	if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);		// Check for completion
}
void gpio_D_ISR(void)															// 2_1, 4_1
{
	t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);									// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTD_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTD_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	delta_t = t_1 - t_0;														// Find time difference
	if(int_mask & LIGHTSNSR3_INT)											// Repeat for each sensor
	{
		raw_prd[2][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[2][1][0]> MAX_PERIOD) raw_prd[2][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT8;
	}
	if(int_mask & LIGHTSNSR5_INT)											// Repeat for each sensor
	{
		raw_prd[4][1][0] = (delta_t / fixedpoint_count_to_time);
		if(raw_prd[4][1][0]> MAX_PERIOD) raw_prd[4][1][0] = MAX_PERIOD;
		lightsense_flag |= BIT9;
	}
	if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);		// Check for completion
}
void lightsense_CLK(void)
{
		Semaphore_post(Sema_lightsense_f);										// Post sema if needed
}
void sort_arr(uint32_t * arr)
{
	uint8_t i;
	uint8_t j;
	uint32_t flipflop;
    for(i=0;i<NUM_SAMPLES;i++)															// Nested for loop to fill sample buffer with ascending values
    {
        for(j=i;j<NUM_SAMPLES;j++)														// start filling cell
        {
            if(arr[i] > arr[j])																	// If one is larger, set that cell to that value, and move on
            {
                flipflop=arr[i];
                arr[i]=arr[j];
                arr[j]=flipflop;
            }
        }
    }
}
