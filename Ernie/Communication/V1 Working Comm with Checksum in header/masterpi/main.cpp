#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <cstdlib>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

bool executeCMD(string cmd) {
	cout << "Command: " << cmd << endl;
	return true;
}

int getChecksum(char* text) {
	int i;
	int count = 0;
	char temp = *text;
	char* ptr = text;
	while(*ptr != 0) {
		temp = *ptr;
		for (i=0; i<(sizeof(char)*8); i++) {
			if (temp & 1) count++;
			temp = temp >> 1;
		}
		ptr++;
		text++;
	}
	return count;
}

void getCharacters(int handle, char* text, int count) {
	char letter;
	char* temp = text;
	while(count > 0) {
		letter = serialGetchar(handle);
		if (letter == 255) continue;
		else {
			*temp = letter;
			temp+=1;
			count--;
		}
	}
	temp = 0;
}

int main() {
	wiringPiSetup();

	int serialHandle;
	cout << "Opening serial..." << endl;
	serialHandle = serialOpen ("/dev/ttyAMA0", 9600) ;

	cout << "serialHandle: " << serialHandle << endl;

	// ~_ _   _ _  _ _ _   _ _ _
	//  length  addr  cmd    chksm

	char letter;
	int length, address, checksum;
	char text[10];
	char* ptr = text;
	char cmd [4];
	int calcCS = 0;

	int argLength = 0;
	char args[100];

	while(1) {
		letter = serialGetchar(serialHandle);
		if (letter == 255) continue;
		if (letter == 3) {
			cout << "End" << endl;
			serialClose(serialHandle);
			return 0;
		}
		if (letter == '~') {
			calcCS = 0;
			cout << "CMD start!" << endl;

			getCharacters(serialHandle,ptr,2);	//length
			length = atoi(text);
			cout << "Length: " << length << endl;

			argLength = length - 2 - 3 - 3;

			if (argLength < 0) argLength = 0;

			getCharacters(serialHandle,ptr,2);	//address
			address = atoi(text);
			calcCS += getChecksum(ptr);
			cout << "Address: " << address << endl;

			getCharacters(serialHandle,cmd,3);	//cmd
			calcCS += getChecksum(cmd);
			cout << "CMD: " << cmd << endl;

			if (argLength > 0) {
					getCharacters(serialHandle,args,argLength);	//address
					calcCS += getChecksum(args);
					cout << "Args: " << args << endl;
			}

			getCharacters(serialHandle,ptr,3);	//address
			checksum = atoi(text);
			cout << "Checksum: " << checksum << endl;

			cout << "calcCS: " << calcCS << endl;	//Calculated Checksum includes Address, Command, and Args
		}
	}

	return 0;
}