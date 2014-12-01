#include "inc.h"

void testing(uint8_t);

UART_Handle* globalUART;
void (*lastCommand) (uint8_t) = testing;

void testing(uint8_t data) {
	UART_write(*globalUART, "!",1);
}

int dutyMain = 75;

void dutyplus(uint8_t data) {
	if (dutyMain < 91) {
	    GPIO_write(Board_LED0, Board_LED_OFF);
		dutyMain += 5;
		lastCommand(dutyMain);
	} else {
		GPIO_write(Board_LED0, Board_LED_ON);
		dutyMain = 90;
	}
}

void dutyminus(uint8_t data) {
	if (dutyMain > 10) {
	    GPIO_write(Board_LED1, Board_LED_OFF);
		dutyMain -= 5;
		lastCommand(dutyMain);
	} else {
	    GPIO_write(Board_LED1, Board_LED_ON);
		dutyMain = 15;
	}
}

char *cmdList[]					=	{"T",
									"7","8","9",
									"4","5","6",
									"2",
									"0"};
									//"+","-",
									//"1","3"};
void (*cmdFunctions[])(int)	=	{testing,
									ccw_motors,	fw_motors,	cw_motors,
									tl_motors,	stop_motors,tr_motors,
									rv_motors,
									coast_motors};
									//dutyplus,dutyminus,
									//cw2_motors,ccw2_motors};

void cmdExecute (char* cmdInput, uint8_t data) {
	int i;
	char inA, inB;
	for (i = 0; i<sizeof(cmdList)/sizeof(char*); i++) {
		inA = *cmdList[i];
		inB = *cmdInput;

		if (inA == inB) {
			System_printf("cmdExecute\n");
			System_flush();
			(*cmdFunctions[i])(dutyMain);
			if ((inA != '+') && (inA != '-')) {
				lastCommand = cmdFunctions[i];
			}
			return;
		}
	}
}

void cmdTerp() {
	UART_Handle uart;
	UART_Params uartParams;

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 9600;
	uart = UART_open(Board_UART1, &uartParams);
	globalUART = &uart;

	if (uart == NULL) {
	        System_abort("Error opening the UART");
	}

	char input[2];
	uint8_t duty;

	while(1) {
		UART_write(uart,">",1);
		UART_read(uart,input,1);
		UART_write(uart,input,1);

		cmdExecute(input,duty);
	}
}


/*
 *  ======== main ========
 */
int main(void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();

    Robot_PWM_init();
    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    while (1) {

    }
}
