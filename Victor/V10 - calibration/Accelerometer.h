/*
 * Accelerometer.h
 *
 *  Created on: Feb 13, 2015
 *      Author: Wicho
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
/* Standard Header Files */
#include <stdint.h>
#include <stdbool.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
/* TivaC includes */
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include <driverlib/i2c.h>
#include <driverlib/sysctl.h>
#include <driverlib/gpio.h>

/* Sensor Headers */
#include "sensorlib/hw_lsm303dlhc.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/lsm303dlhc_accel.h"

void initI2C0(void);
void LSM303DLHCAccel_task(void);
void LSM303DLHCAccelCallback(void *pvCallbackData, uint_fast8_t ui8Status);
#endif /* ACCELEROMETER_H_ */
