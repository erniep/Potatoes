#include "inc.h"
#include "serial.h"

//Reference:
//UART_Handle *piUART;

//This file contains all of the serial communication functions

/*CFG Info:
Semaphore:	Sema_ack
Task:		Ack_task	sendAck()
*/

void sendAck() {
	while(1) {
		Semaphore_pend(Sema_ack, BIOS_WAIT_FOREVER);
		UART_write(piUART, "~", 1);
	}
}

