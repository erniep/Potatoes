/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-A44
 */

#include <xdc/std.h>

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle Sema_lightsense_f;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle Sema_lightsense;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle Timer2A;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Drive_task;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle Sema_accelerometer;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Lightsense_task;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle GPIOA;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle GPIOB;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle GPIOC;

#include <ti/sysbios/hal/Hwi.h>
extern const ti_sysbios_hal_Hwi_Handle GPIOD;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle Lightsense_sample;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle Sema_lightsense_filter;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle Medianfilter;

#define TI_DRIVERS_WIFI_INCLUDED 0

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

#ifndef ti_sysbios_knl_Task__include
#ifndef __nested__
#define __nested__
#include <ti/sysbios/knl/Task.h>
#undef __nested__
#else
#include <ti/sysbios/knl/Task.h>
#endif
#endif

extern ti_sysbios_knl_Task_Struct TSK_idle;

