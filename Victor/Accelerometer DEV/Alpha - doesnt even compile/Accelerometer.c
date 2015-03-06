/*
 * Accelerometer.c
 *
 *  Created on: Feb 13, 2015
 *      Author: Wicho
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>
#include "sensorlib/hw_lsm303dlhc.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/lsm303dlhc_accel.h"

#include "sensorlib/hw_lsm303dlhc.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/lsm303dlhc_accel.h"
/* Global Variables */
//
// A boolean that is set when a LSM303DLHCAccel command has completed.
//
volatile bool g_bLSM303DLHCAccelDone;

void initI2C0(void);
void LSM303DLHCAccelCallback(void *pvCallbackData, uint_fast8_t ui8Status);
void LSM303DLHCAccel_task(void);

void initI2C0(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

   //reset I2C module
   SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

   //enable GPIO peripheral that contains I2C
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

   // Configure the pin muxing for I2C0 functions on port B2 and B3.
   GPIOPinConfigure(GPIO_PB2_I2C0SCL);
   GPIOPinConfigure(GPIO_PB3_I2C0SDA);

   // Select the I2C function for these pins.
   GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
   GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

   // Enable and initialize the I2C0 master module.
   I2CMasterInitExpClk(I2C0_BASE, 80000000, true);

   //clear I2C FIFOs
   HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}
void LSM303DLHCAccelCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
	//
	// See if an error occurred.
	//
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.
	}
	// Indicate that the LSM303DLHCAccel transaction has completed.
	g_bLSM303DLHCAccelDone = true;
}
void LSM303DLHCAccel_task(void)
{
	// Init I2C
	initI2C0();

	float fAccel[3];
	tI2CMInstance sI2CInst;
	tLSM303DLHCAccel sLSM303DLHCAccel;

	//
	// Initialize the LSM303DLHCAccel. This code assumes that the I2C master
	// instance has already been initialized.
	//
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelInit(&sLSM303DLHCAccel, &sI2CInst, 0x19, LSM303DLHCAccelCallback, 0);
	while(!g_bLSM303DLHCAccelDone){}

	//	Configure for 400Hz operation
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelReadModifyWrite(&sLSM303DLHCAccel, LSM303DLHC_O_CTRL1, LSM303DLHC_CTRL1_ODR_M, LSM303DLHC_CTRL1_ODR_400HZ, LSM303DLHCAccelCallback, 0);

	//
	 // Configure on board HP filter. ft = fsamp / (6 * HPc) where HPc is the value
	 // For Fsamp = 100Hz, 00 = 2Hz, 01 = 1Hz, 10 = 1/2Hz, 11 = 0.25Hz
	//
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelReadModifyWrite(&sLSM303DLHCAccel, LSM303DLHC_O_CTRL2, LSM303DLHC_CTRL2_HPCUTOFF_M, 0x3, LSM303DLHCAccelCallback, 0);

	// HPF mode selection - Normal Mode
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelReadModifyWrite(&sLSM303DLHCAccel, LSM303DLHC_O_CTRL2, LSM303DLHC_CTRL2_HPMODE_M, LSM303DLHC_CTRL2_HPMODE_NORMAL, LSM303DLHCAccelCallback, 0);

	// Filtered Data Selection - Filter enabled
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelReadModifyWrite(&sLSM303DLHCAccel, LSM303DLHC_O_CTRL2, LSM303DLHC_CTRL2_FDS_M, LSM303DLHC_CTRL2_FDS_FILTERED, LSM303DLHCAccelCallback, 0);

	// Configure the LSM303DLHCAccel for +/- 4 g accelerometer range.
	g_bLSM303DLHCAccelDone = false;
	LSM303DLHCAccelReadModifyWrite(&sLSM303DLHCAccel, LSM303DLHC_O_CTRL4, ~LSM303DLHC_CTRL4_FS_M, LSM303DLHC_CTRL4_FS_4G, LSM303DLHCAccelCallback, 0);

	while(!g_bLSM303DLHCAccelDone){}
	// Enter the task
	while(1)
	{
	// Request another reading from the LSM303DLHCAccel.
		g_bLSM303DLHCAccelDone = false;
		LSM303DLHCAccelDataRead(&sLSM303DLHCAccel, LSM303DLHCAccelCallback, 0);
		while(!g_bLSM303DLHCAccelDone){}
		// Get the new accelerometer readings.
		LSM303DLHCAccelDataAccelGetFloat(&sLSM303DLHCAccel, &fAccel[0], &fAccel[1], &fAccel[2]);
		// Do something with the new accelerometer readings.
	}
}
