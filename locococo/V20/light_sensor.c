/*
 * light_sensor.c
 *
 *  Created on: Jan 21, 2015
 *      Author: Wicho
 */
#include "light_sensor.h"
/*
 * Global Variables
 */																														// Threshold compare values
uint32_t raw_prd[NUM_ARRAYS][NUM_SENSORS][NUM_SAMPLES] = {0};															// Holds values for periods measured																			// Holds values for periods measured
uint32_t prd[NUM_ARRAYS][NUM_SENSORS] = {0};																			// Holds values for periods filtered
uint32_t t_0 = 0;																										// t = 0
uint32_t delta_t[10] = {0};																									// Change in time placeholder
uint16_t lightsense_flag = 0;																							// Interrupt mask
uint16_t max_period = MAX_PERIOD;																								// Max period measured
uint8_t lightsensor_sample = 0;																							// Light sensor sample en
uint8_t calibrate_sample = 0;																							// Calibrate routine en
uint8_t filter_en = 0;																									// Filter turn on flag
uint8_t filter_order = NUM_SAMPLES;
uint8_t timeout = 0;

void median_filter_task(void)
{
	uint32_t temp_arr[NUM_SAMPLES] = {0};
	int8_t k = 0;
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
				if(filter_en)																							// Is the filter on?
				{
					memcpy(temp_arr, raw_prd[n][m], 4*NUM_SAMPLES);															// Copy Raw data into a temporary sorting array
					//sort_arr(temp_arr);																					// Sort array - If NUM_SAMPLES > 3 change this code
					quicksort(temp_arr, 0, NUM_SAMPLES-1);
					if((NUM_SAMPLES % 2)==0) prd[n][m] = (temp_arr[NUM_SAMPLES/2] + temp_arr[(NUM_SAMPLES/2)-1])/2;
					else prd[n][m] = temp_arr[NUM_SAMPLES/2];																	// Grab median value - if NUM_SAMPLES is even change this line
					for(k = (NUM_SAMPLES-1); k > 0; k--) raw_prd[n][m][k] = raw_prd[n][m][k-1];								// Update Raw data for next iteration
				}
				else prd[n][m] = raw_prd[n][m][0];
			}
		}
		// Post Semaphore - Filtered Data ready
		Semaphore_post(Sema_lightsense);
	}
}
void Robot_lightsnsr_task(void)
{
	// Configure Timers
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);																		// Enable Timer clock source
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);																// Set Timer clock source
	TimerClockSourceSet(WTIMER0_BASE, TIMER_CLOCK_SYSTEM);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP);										// Configure Timer2 as two 16 bit timers
	TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP);										// Configure WTimer0A as a 32bit timer

	// Unlock PD7 from Intrinsic NMI Function
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Configure Long range IR sensor for higher current
	GPIOPadConfigSet(LIGHTSNSR3_BASE, LIGHTSNSR3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(LIGHTSNSR4_BASE, LIGHTSNSR4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(LIGHTSNSR5_BASE, LIGHTSNSR5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(LIGHTSNSR6_BASE, LIGHTSNSR6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

	// Load period values of each light sensor into reg
	uint32_t Period = ((Clock * CHARGE_PRD) / fixedpoint_microsec_coeff);												// set period
	TimerLoadSet(TIMER2_BASE, TIMER_A, (uint16_t)Period -1);															// Enable interrupts for Timer2A
	Period = ((Clock / fixedpoint_microsec_coeff)* max_period);
	TimerLoadSet(WTIMER0_BASE, TIMER_A, (uint32_t)Period -1);

	//Enable Interrupts
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntEnable(INT_WTIMER0A);
	TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);

	Hwi_enableInterrupt(INT_GPIOA | INT_GPIOB | INT_GPIOC | INT_GPIOD);

	// Set interrupt type to falling edge (consult discrete interrupts in future)
	GPIOIntTypeSet(LIGHTSNSR1_BASE, LIGHTSNSR1, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR2_BASE, LIGHTSNSR2, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR3_BASE, LIGHTSNSR3, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR4_BASE, LIGHTSNSR4, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR5_BASE, LIGHTSNSR5, GPIO_FALLING_EDGE);
	GPIOIntTypeSet(LIGHTSNSR6_BASE, LIGHTSNSR6, GPIO_FALLING_EDGE);

	// Set GPIO pins to interrupt
	GPIOIntEnable(LIGHTSNSR1_BASE, LIGHTSNSR1_INT);
	GPIOIntEnable(LIGHTSNSR2_BASE, LIGHTSNSR2_INT);
	GPIOIntEnable(LIGHTSNSR3_BASE, LIGHTSNSR3_INT);
	GPIOIntEnable(LIGHTSNSR4_BASE, LIGHTSNSR4_INT);
	GPIOIntEnable(LIGHTSNSR5_BASE, LIGHTSNSR5_INT);
	GPIOIntEnable(LIGHTSNSR6_BASE, LIGHTSNSR6_INT);

	// Begin Task
	while(TRUE)
	{
		// Pend on semaphore - Sample frequency
		Semaphore_pend(Sema_lightsense_f, BIOS_WAIT_FOREVER);
		// Set lightsense GPIO to output
		GPIOPinTypeGPIOOutput(LIGHTSNSR1_BASE, LIGHTSNSR1);
		GPIOPinTypeGPIOOutput(LIGHTSNSR2_BASE, LIGHTSNSR2);
		GPIODirModeSet(LIGHTSNSR3_BASE, LIGHTSNSR3, GPIO_DIR_MODE_OUT);
		GPIODirModeSet(LIGHTSNSR4_BASE, LIGHTSNSR4, GPIO_DIR_MODE_OUT);
		GPIODirModeSet(LIGHTSNSR5_BASE, LIGHTSNSR5, GPIO_DIR_MODE_OUT);
		GPIODirModeSet(LIGHTSNSR6_BASE, LIGHTSNSR6, GPIO_DIR_MODE_OUT);

		// Set lightsense pins hi
		GPIOPinWrite(LIGHTSNSR1_BASE, LIGHTSNSR1, LIGHTSNSR1);
		GPIOPinWrite(LIGHTSNSR2_BASE, LIGHTSNSR2, LIGHTSNSR2);
		GPIOPinWrite(LIGHTSNSR3_BASE, LIGHTSNSR3, LIGHTSNSR3);
		GPIOPinWrite(LIGHTSNSR4_BASE, LIGHTSNSR4, LIGHTSNSR4);
		GPIOPinWrite(LIGHTSNSR5_BASE, LIGHTSNSR5, LIGHTSNSR5);
		GPIOPinWrite(LIGHTSNSR6_BASE, LIGHTSNSR6, LIGHTSNSR6);

		// Reset timeout and sensor flag
		timeout = 0;
		lightsense_flag = 0;

		// Update period
		TimerDisable(WTIMER0_BASE, TIMER_A);
		Period = ((Clock / fixedpoint_microsec_coeff)* max_period);
		TimerLoadSet(WTIMER0_BASE, TIMER_A, (uint32_t)Period -1);
		TimerEnable(TIMER2_BASE, TIMER_A);																				// Start charging line sensors
	}
}
void timer2A_ISR(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	// Re-enable interrupts for timeout process
	//Hwi_enableInterrupt(INT_GPIOA | INT_GPIOB | INT_GPIOC | INT_GPIOD);
	GPIOPinTypeGPIOInput(LIGHTSNSR1_BASE, LIGHTSNSR1);																	// Set pins to input
	GPIOPinTypeGPIOInput(LIGHTSNSR2_BASE, LIGHTSNSR2);
	GPIODirModeSet(LIGHTSNSR3_BASE, LIGHTSNSR3, GPIO_DIR_MODE_IN);
	GPIODirModeSet(LIGHTSNSR4_BASE, LIGHTSNSR4, GPIO_DIR_MODE_IN);
	GPIODirModeSet(LIGHTSNSR5_BASE, LIGHTSNSR5, GPIO_DIR_MODE_IN);
	GPIODirModeSet(LIGHTSNSR6_BASE, LIGHTSNSR6, GPIO_DIR_MODE_IN);
	TimerEnable(WTIMER0_BASE, TIMER_A);																					// Start timeout timer
	t_0 = HWREG(WTIMER0_BASE + TIMER_O_TAR);																			// Grab Timer Value
}
void gpio_A_ISR(void)															// 0_0, 0_1, 0_2
{
	uint32_t int_mask = HWREG(GPIO_PORTA_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTA_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	uint32_t t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);							// Grab timer value 2
	// Grab raw values as well as well as raise a flag
	if(int_mask & LIGHTSNSR1_1_INT)
	{
		delta_t[0] = t_1 - t_0;
		lightsense_flag |= BIT0;
	}
	if(int_mask & LIGHTSNSR1_2_INT)
	{
		delta_t[1] = t_1 - t_0;
		lightsense_flag |= BIT1;
	}
	if(int_mask & LIGHTSNSR1_3_INT)
	{
		delta_t[2] = t_1 - t_0;
		lightsense_flag |= BIT2;
	}
	if((lightsense_flag & (BIT0|BIT1|BIT2)) == (BIT0|BIT1|BIT2)) Swi_post(GPIOASWI);
}
void gpio_B_ISR(void)															// 3_1, 5_1
{
	uint32_t t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);							// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTB_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTB_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	// Grab raw values as well as well as raise a flag
	if(int_mask & LIGHTSNSR4_INT)
	{
		delta_t[3] = t_1 - t_0;
		lightsense_flag |= BIT3;
	}
	if(int_mask & LIGHTSNSR6_INT)
	{
		delta_t[4] = t_1 - t_0;
		lightsense_flag |= BIT4;
	}
	if(lightsense_flag & ((BIT3|BIT4) == (BIT3|BIT4))) Swi_post(GPIOBSWI);
}
void gpio_C_ISR(void)															//1_0,1_1,1_2
{
	uint32_t t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);							// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTC_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTC_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	// Grab raw values as well as well as raise a flag
	if(int_mask & LIGHTSNSR2_1_INT)
	{
		delta_t[5] = t_1 - t_0;
		lightsense_flag |= BIT5;
	}
	if(int_mask & LIGHTSNSR2_2_INT)
	{
		delta_t[6] = t_1 - t_0;
		lightsense_flag |= BIT6;
	}
	if(int_mask & LIGHTSNSR2_3_INT)
	{
		delta_t[7] = t_1 - t_0;
		lightsense_flag |= BIT7;
	}
	if((lightsense_flag & (BIT5|BIT6|BIT7)) == (BIT5|BIT6|BIT7)) Swi_post(GPIOCSWI);
}
void gpio_D_ISR(void)															// 2_1, 4_1
{
	uint32_t t_1 = HWREG(WTIMER0_BASE + TIMER_O_TAR);							// Grab timer value 2
	uint32_t int_mask = HWREG(GPIO_PORTD_BASE + GPIO_O_MIS);					// Get interrupt mask
	HWREG(GPIO_PORTD_BASE + GPIO_O_ICR) = int_mask;								// Clear Interrupts
	// Grab raw values as well as well as raise a flag
	if(int_mask & LIGHTSNSR3_INT)
	{
		delta_t[8] = t_1 - t_0;
		lightsense_flag |= BIT8;
	}
	if(int_mask & LIGHTSNSR5_INT)
	{
		delta_t[9] = t_1 - t_0;
		lightsense_flag |= BIT9;
	}
	if((lightsense_flag & (BIT8|BIT9)) == (BIT8|BIT9)) Swi_post(GPIODSWI);
}
void gpio_A_SWI(void)
{
	if(!timeout)
	{
		raw_prd[0][0][0] = (delta_t[0] / fixedpoint_count_to_time);
		raw_prd[0][1][0] = (delta_t[1] / fixedpoint_count_to_time);
		raw_prd[0][2][0] = (delta_t[2] / fixedpoint_count_to_time);
		if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);
	}
}
void gpio_B_SWI(void)
{
	if(!timeout)
	{
		raw_prd[3][1][0] = (delta_t[3] / fixedpoint_count_to_time);
		raw_prd[5][1][0] = (delta_t[4] / fixedpoint_count_to_time);
		if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);
	}
}
void gpio_C_SWI(void)
{
	if(!timeout)
	{
		raw_prd[1][0][0] = (delta_t[5] / fixedpoint_count_to_time);
		raw_prd[1][1][0] = (delta_t[6] / fixedpoint_count_to_time);
		raw_prd[1][2][0] = (delta_t[7] / fixedpoint_count_to_time);
		if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);
	}

}
void gpio_D_SWI(void)
{
	if(!timeout)
	{
		raw_prd[2][1][0] = (delta_t[8] / fixedpoint_count_to_time);
		raw_prd[4][1][0] = (delta_t[9] / fixedpoint_count_to_time);
		if(lightsense_flag == 0x3FF) Semaphore_post(Sema_lightsense_filter);
	}
}
void Wtimer0A_ISR(void)
{
	TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);							// Clear current interrupt
	// Enable timeout flag
	timeout = 1;
	// Set values for sensors that timed out
	if(!(lightsense_flag & BIT0)) raw_prd[0][0][0] = max_period;
	if(!(lightsense_flag & BIT1)) raw_prd[0][1][0] = max_period;
	if(!(lightsense_flag & BIT2)) raw_prd[0][2][0] = max_period;
	if(!(lightsense_flag & BIT3)) raw_prd[3][1][0] = max_period;
	if(!(lightsense_flag & BIT4)) raw_prd[5][1][0] = max_period;
	if(!(lightsense_flag & BIT5)) raw_prd[1][0][0] = max_period;
	if(!(lightsense_flag & BIT6)) raw_prd[1][1][0] = max_period;
	if(!(lightsense_flag & BIT7)) raw_prd[1][2][0] = max_period;
	if(!(lightsense_flag & BIT8)) raw_prd[2][1][0] = max_period;
	if(!(lightsense_flag & BIT9)) raw_prd[4][1][0] = max_period;
	// post sema, data is ready
	if(lightsense_flag < 0x3FF) Semaphore_post(Sema_lightsense_filter);
	lightsense_flag = 0;
}
void lightsense_CLK(void)
{
	if(lightsensor_sample)
	{
		Semaphore_post(Sema_lightsense_f);									// Post sema for sampling
	}
}
void calibrate_CLK(void)
{
	if(calibrate_sample)
	{
		uint32_t Period = ((Clock / fixedpoint_microsec_coeff)* max_period);
		TimerLoadSet(WTIMER0_BASE, TIMER_A, (uint32_t)Period -1);
		Semaphore_post(Sema_lightsense_f);										// Post sema for sampling
	}
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
void quicksort( uint32_t a[], int l, int r)
{
   int j;

   if( l < r )
   {
   	// divide and conquer
        j = partition( a, l, r);
       quicksort( a, l, j-1);
       quicksort( a, j+1, r);
   }

}
int partition( uint32_t a[], int l, int r) {
   int pivot, i, j, t;
   pivot = a[l];
   i = l; j = r+1;

   while( 1)
   {
   	do ++i; while( a[i] <= pivot && i <= r );
   	do --j; while( a[j] > pivot );
   	if( i >= j ) break;
   	t = a[i]; a[i] = a[j]; a[j] = t;
   }
   t = a[l]; a[l] = a[j]; a[j] = t;
   return j;
}
