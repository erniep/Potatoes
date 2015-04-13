#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <sys/types.h> 
#include <sys/stat.h>   
#include <wiringPi.h>
#include <wiringSerial.h>

#include "serialComm2.h"

#define AMA // Serial through TX/RX
//#define USB // Serial through USB
/*
April 9, 2015

-Started amending Ernie's existing toReceive.cpp to be able to interpret different commands from the main pi (PIC, DNE, EGG, etc.)

-This main function is specific to the left raspberry pi
*/

using namespace std;

string GetStdoutFromCommand(string cmd) {
	string data;
	FILE * stream;
	const int max_buffer = 256;
	char buffer[max_buffer];

	stream = popen(cmd.c_str(), "r");
	if (stream) {
		while (!feof(stream))
			if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
				pclose(stream);
	}
	return data;
}

int main() {
	//This code assumes that the FIFO pipes 'takePicPipe' and 'eggRead' have been created in the directory
	char message[100];
	char extractedCMD[4];
	char temp[2] = {'~',0};
	string scannedEggString;
	char scannedEgg;
	char eggs[51] = {'A', 'B', 'C', 'D', 'E'}; 
	char EGG[3] = {'E', 'G', 'G'};
	int ei = 0; // Eggs iterator
	int readEI = 0;
	
	eggs[50] = 0; //Null terminate the eggs array so it can be sent through sendMessage
	#ifdef AMA
	int serialHandle = serialOpen("/dev/ttyAMA0", 9600);
	#endif
	#ifdef USB
	int serialHandle = serialOpen("/dev/ttyUSB1", 9600);
	#endif

	serialFlush(serialHandle);

	while(1) {
		cout << "Waiting for message" << endl;
		getMessage(serialHandle,'L', message);	
		cout << "Message received: " << message << endl;
		//cout << "CMD: " << getCMD(message,extractedCMD) << endl;
		cout << "comp: " << strcmp(message, "PIC") << endl;
		if(!strcmp(message, "PIC")){
			cout << "PIC TIME!" << endl;
			system(" echo 'go' > takePicPipe");	
			scannedEggString = GetStdoutFromCommand("cat < takePicPipe");
			scannedEgg = scannedEggString[0];
			eggs[ei] = scannedEgg;
			ei++;
		}
		else if(!strcmp(message, "DNE")){
			eggs[ei] = ' ';
			ei++;
		}
		else if(!strcmp(message, "GET")){ // Used when consolidating eggs from all 3 pis.		
			char eggTemp[3] = {eggs[readEI], '_', '_'};
			cout << "Sending character: " << eggTemp[0] << eggTemp[1] << eggTemp[2] << endl;			
			sendMessage(serialHandle, 'M', eggTemp, temp);
			cout << "Char sent" << endl;
			if (readEI < (ei-1)) readEI++;
		}
		else if(!strcmp(message, "ID_")){
			sendMessage(serialHandle, 'M', EGG, temp);
		}
		else if(!strcmp(message, "TST")){
			cout << "Connection with left pi successful" << endl;
		}
		serialFlush(serialHandle);
	}


	return 0;
}
