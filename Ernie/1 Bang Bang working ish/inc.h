#ifndef INC_H
#define INC_H

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/hal/Timer.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdint.h>
#include <math.h>
#include <string.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"

#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

//#include "Motors_Ernie.h"

#include "Motors_Ernie.h"
#include "light_sensor.h"
#include "drive.h"

#include "serial.h"

//Global Variables
UART_Handle *globalUART;
UART_Handle *piUART;
uint32_t lightsnsr_val[4];

#define WWW 0x00
#define WWB 0x01
#define WBB 0x03
#define BBB 0x07
#define WBW 0x02
#define BWB 0x05
#define BBW 0x06
#define BWW 0x04

#endif
