#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

void getCharacters(int serialHandle, char* text, int count);
void sendCharacters(int serialHandle, char* text);
int getChecksum(char* text);
int getMessage(int serialHandle, char _address, char * _data);
int sendMessage(int serialHandle, char address, char cmd[3], char *args);
int sendAck(int serialHandle, char address, int result);
bool waitAck(int serialHandle, char address);

void getCharacters(int serialHandle, char* text, int count) {
	char letter;
	char* temp = text;
	while(count > 0) {
		letter = serialGetchar(serialHandle);
		if (letter == 255) continue;
		else {
			*temp = letter;
			temp+=1;
			count--;
		}
	}
	*temp = 0;
}

void sendCharacters(int serialHandle, char* text) {
	serialFlush(serialHandle);
	while (*text != 0) {
		serialPutchar(serialHandle, *(text++));
	}
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

int getMessage(int serialHandle, char _address, char *_data) {
	// ~  _ _   _ _  _ _ _   _ _ _
	//  length  addr  cmd    chksm

	char letter;
	int length, checksum;
	char address[2];
	address[1] = 0;
	char text[10];
	char* ptr = text;
	char cmd [4];
	char* cmdptr = cmd;
	int calcCS = 0;

	int argLength = 0;
	char args[100];

	serialFlush(serialHandle);

	while(1) {
		letter = serialGetchar(serialHandle);
		if (letter == 255) continue;	//timeout
		if (letter == 3) {	//ctrl+c
			cout << "End" << endl;
			serialClose(serialHandle);
			return 0;
		}
		if (letter == '~') {
			calcCS = 0;
			cout << endl << "CMD start!" << endl;

			getCharacters(serialHandle,ptr,2);	//length
			length = atoi(text);
			cout << "Length: " << length << endl;

			argLength = length - 1 - 3;

			if (argLength < 0) argLength = 0;

			getCharacters(serialHandle,ptr,1);	//address
			*address = *text;
			//address = atoi(text);
			calcCS += getChecksum(address);
			cout << "Address: " << address << endl;

			if (*address != _address) {
				//means not our message
				sendAck(serialHandle, _address, 1);
				continue;
			}

			getCharacters(serialHandle,cmdptr,3);	//cmd
			calcCS += getChecksum(cmd);
			cout << "CMD: " << cmd << endl;

			if (argLength > 0) {
					getCharacters(serialHandle,args,argLength);	//address
					calcCS += getChecksum(ptr);
					cout << "Args: " << address << endl;
			}

			getCharacters(serialHandle,ptr,3);	//address
			checksum = atoi(text);
			cout << "Checksum: " << checksum << endl;

			cout << "calcCS: " << calcCS << endl;	//Calculated Checksum includes Address, Command, and Args

			if (calcCS != checksum) {
				//failed send, resend
				char temp[] = {'~','~','0'};
				sendCharacters(serialHandle,temp);
				//sendAck(serialHandle, _address, 0);
			} else {
				serialFlush(serialHandle);
				//good, send "~~" back for ack
				char temp[] = {'~','~','1'};
				sendCharacters(serialHandle,temp);
				sendAck(serialHandle, _address, 1);
				break;	//break out of while
			}
		}
	}
}

int sendMessage(int serialHandle, char address, char cmd[3], char *args) {
	//if read args is ~ then no args
	int argsSize;
	if (*args == '~') argsSize = 0;
	else {
		//loop to count argsSize
	}
	int size = 1+3+argsSize;
	char message[100];char tempMessage[100];
	char *finalMessage = message;
	char temp[2];
	temp[0] = address;
	temp[1] = 0;
	char *addptr = temp;
	strcpy(finalMessage, "~");
	snprintf(tempMessage,sizeof(tempMessage),"0%d",size);
	*(tempMessage+2)= 0;
	strcat(finalMessage, tempMessage);
	strcat(finalMessage, addptr);
	strcat(finalMessage, cmd);
	if (argsSize != 0) strcat(finalMessage, args);
	strcat(finalMessage,"011");	
	//cout << "Message: " << finalMessage << "|" << endl;
	//cout << "handle: " << serialHandle << endl;
	//cout << "addr: " << address << endl;
	serialFlush(serialHandle);
	while (1) {
		sendCharacters(serialHandle, finalMessage);
		while (waitAck(serialHandle, address) == false) {}
		break;
	}

	return 0;
}

int sendAck(int serialHandle, char address, int result) {
	char message[] = { '~' , '~' , '0' , 0 };
	if (result == 0) sendCharacters(serialHandle, message);
	else {
		message[2] = '1';
		sendCharacters(serialHandle, message);
	}
}

bool waitAck(int serialHandle, char address) {
	char single;
	char message[100];
			while (1) {
				single = serialGetchar(serialHandle);
				if (single == '~') {
					single = serialGetchar(serialHandle);
					if (single == '~') {
						//done!
						single = serialGetchar(serialHandle);
						if (single == '1') {
							//ack good
							return true;
						} else {
							//ack bad
							return false;
						}
					}
				}
			}
			//break;
}

int main() {
	char message[100];
	int serialHandle = serialOpen("/dev/ttyAMA0", 9600);

	serialFlush(serialHandle);

	while(1) {
		getMessage(serialHandle,'L',message);	
		cout << "\tGOOD!" << endl;
		serialFlush(serialHandle);
	}

	return 0;
}

#endif
