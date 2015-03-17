#include "inc.h"
#include "light_sensor.h"

void testing(uint8_t);

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

char returnBit(uint32_t data, int bit) {
	switch (bit) {
	case 1:
		if ((data & 1) != 0) return '1';
		else return '0';
	case 2:
		if ((data & 2) != 0 ) return '1';
		else return '0';
	case 3:
		if ((data & 4) != 0) return '1';
		else return '0';
	}
}

void moveW(uint8_t data) {
	motorAll(50,0,0,0);
	//fw_motors(50);

	char output[6];
	output[0] = '\r';
	output[1] = '\n';
	output[2] = returnBit(lightsnsr_val[0], 1);
	output[3] = returnBit(lightsnsr_val[0], 2);
	output[4] = returnBit(lightsnsr_val[0], 3);
	output[5] = 0;

	UART_write(*piUART,output,6);
}

void moveA(uint8_t data) {
	motorAll(0,50,0,0);
	char output[6];
	output[0] = '\r';
	output[1] = '\n';
	output[2] = returnBit(lightsnsr_val[2], 1);
	output[3] = returnBit(lightsnsr_val[2], 2);
	output[4] = returnBit(lightsnsr_val[2], 3);
	output[5] = 0;

	UART_write(*piUART,output,6);

}

void moveS(uint8_t data) {
	motorAll(0,0,50,0);
	//rv_motors(20, 1);
	char output[6];
	output[0] = '\r';
	output[1] = '\n';
	output[2] = returnBit(lightsnsr_val[1], 1);
	output[3] = returnBit(lightsnsr_val[1], 2);
	output[4] = returnBit(lightsnsr_val[1], 3);
	output[5] = 0;

	UART_write(*piUART,output,6);
}

void moveD(uint8_t data) {
	motorAll(0,0,0,50);
	char output[6];
	output[0] = '\r';
	output[1] = '\n';
	output[2] = returnBit(lightsnsr_val[3], 1);
	output[3] = returnBit(lightsnsr_val[3], 2);
	output[4] = returnBit(lightsnsr_val[3], 3);
	output[5] = 0;

	UART_write(*piUART,output,6);
}

void moveQ(uint8_t data) {
	fw_motors(50);
}

void moveE(uint8_t data) {
	//cw_motors(50, 1);
}

void moveX(uint8_t data) {
	//cw_motors(50, 1);
}

char *cmdList[]					=	{"T",
									"7","8","9",
									"4","5","6",
									"2",
									"0",
									"+","-",
									"w","a","s","d",
									"q","e","x"};
									//"1","3"};
void (*cmdFunctions[])(int)	=	{testing,
									ccw_motors,	fw_motors,	cw_motors,
									tl_motors,	stop_motors,tr_motors,
									rv_motors,
									coast_motors,
									dutyplus,dutyminus,
									moveW,moveA,moveS,moveD,
									moveQ,moveE,moveX};
									//cw2_motors,ccw2_motors};
int isToggleLED = 0;
void toggleLED(int n) {
	if (isToggleLED == 0) {
		if (n == 0) GPIO_write(Board_LED1, Board_LED_ON);
		else GPIO_write(Board_LED2, Board_LED_ON);
		isToggleLED = 1;
	} else {
		if (n == 0) GPIO_write(Board_LED1, Board_LED_OFF);
		else GPIO_write(Board_LED2, Board_LED_OFF);
		isToggleLED = 0;
	}
}
void cmdExecute (char* cmdInput, uint8_t data) {
	int i;
	char inA, inB;
	for (i = 0; i<sizeof(cmdList)/sizeof(char*); i++) {
		inA = *cmdList[i];
		inB = *cmdInput;

		if (inA == inB) {
			System_printf("cmdExecute\n");
			System_flush();
			//(*cmdFunctions[i])(dutyMain);
			(*cmdFunctions[i])(data);
			if ((inA != '+') && (inA != '-')) {
				lastCommand = cmdFunctions[i];
			}
			return;
		}
	}
}

void cmdTerp() {
	UART_Handle uart;
	UART_Handle uart2;
	UART_Params uartParams,uartParams2;

	UART_Params_init(&uartParams);
	uartParams.writeDataMode = UART_DATA_BINARY;
	uartParams.readDataMode = UART_DATA_BINARY;
	uartParams.readReturnMode = UART_RETURN_FULL;
	uartParams.readEcho = UART_ECHO_OFF;
	uartParams.baudRate = 9600;

	uart = UART_open(Board_UART0, &uartParams);
	globalUART = &uart;

	UART_Params_init(&uartParams2);
	uartParams2.writeDataMode = UART_DATA_BINARY;
	uartParams2.readDataMode = UART_DATA_BINARY;
	uartParams2.readReturnMode = UART_RETURN_FULL;
	uartParams2.readEcho = UART_ECHO_OFF;
	uartParams2.baudRate = 9600;

	uart2 = UART_open(Board_UART1, &uartParams2);
	piUART = &uart2;

	if (uart == NULL) {
	        System_abort("Error opening the UART");
	}
	if (uart2 == NULL) {
		        System_abort("Error opening the UART");
	}

	char input[2];
	uint8_t duty = 50;

	while(1) {
		UART_write(uart2,">",1);
		UART_read(uart2,input,1);
		UART_write(uart2,input,1);

		cmdExecute(input,duty);
		toggleLED(0);
	}
}

void outputToPi() {
	char input2[10] = "0123456789";
	while(1) {
		//UART_read(*globalUART,input2,1);
		UART_write(*globalUART,input2,10);
		toggleLED(1);
		break;
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
    //GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    while (1) {

    }
}
