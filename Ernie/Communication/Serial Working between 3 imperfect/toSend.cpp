#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#include <wiringPi.h>
#include <wiringSerial.h>

#include "serialComm2.h"

//#define AMA // Serial through TX/RX
#define USB // Serial through USB

using namespace std;

int main() {
	char ackMessage[100];
	char eggs[51];
	#ifdef AMA
	int serialHandle = serialOpen("/dev/ttyAMA0",9600);
	#endif
	#ifdef USB
	int serialHandleR = serialOpen("/dev/ttyUSB0",9600);
	int serialHandleL = serialOpen("/dev/ttyUSB1",9600);
	int *serialHandlePtr;
	#endif
	char address[2];
	char egg[5];
	char cmd[3];

		

	while(1) {
		cout << "Address: ";
		cin >> address[0];
		address[1]=0;
		cout << "Command: ";
		cin >> cmd;
		
		if (address[0] == 'L') serialHandlePtr = &serialHandleL;
		else serialHandlePtr = &serialHandleR;

		char temp[2] = {'~',0};
		sendMessage(*serialHandlePtr, *address, cmd, temp);
		//wait for ack (THIS WAS A PROBLEM), NOW FIXED
		//waitAck(serialHandle,'M');
		if(!strcmp(cmd, "GET")){
			waitAck(*serialHandlePtr,'M');
			cout << "Get message started" << endl;
			getMessage(*serialHandlePtr, 'M', egg);
			cout << "Egg received: " << egg[0] << endl;
		}
		cout << endl;
	}

	return 0;
}
