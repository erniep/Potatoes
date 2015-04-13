#ifndef SERIAL_H
#define SERIAL_H

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <cstdio>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

/*March 27, 2015
-Ernie made putChecksum() so sendMessage() can transmit any CMD
-Made cmd passed to sendMessage needed to be null terminated, so fixed that
*/

char csString[4];

void getCharacters(int serialHandle, char* text, int count);
void sendCharacters(int serialHandle, char* text);
int getChecksum(char* text);
char* putChecksum(int cs);
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
	if (count < 999) return count;
	else return 0;
}

char* putChecksum(int cs) {
	//put the checksum into 3 byte field. Fill it up right aligned (011[message] is 11[decimal])
	csString[3] = 0;
	char output[4],temp[4];
	char* ptr = temp;
	int count = 0;
	snprintf(temp, sizeof(temp), "%d", cs);
	ptr = temp;
	cout << "(serialComm2.sendMessage) temp: " << temp << "|" << endl;
	while (*ptr++ != 0) {
		count++;
		if (count > 3) {
			return csString;
		}
	}
	cout << "(serialComm2.sendMessage) cs: " << cs << endl;
	cout << "(serialComm2.sendMessage) count: " << count << endl;
	switch (count) {
		case 1:
			*(output) = '0';
			*(output+1) = '0';
			*(output+2) = temp[0];
			break;
		case 2:
			*(output) = '0';
			*(output+1) = temp[0];
			*(output+2) = temp[1];
			break;
		case 3:
			*(output) = temp[0];
			*(output+1) = temp[1];
			*(output+2) = temp[2];
			break;
		default:
			strcpy(output,"000");
	}
	strcpy(csString, output);
	csString[3]  = 0;
	cout << "(serialComm2.sendMessage) fin : " << csString << "|" << endl;
	return csString;
}

int getMessage(int serialHandle, char _address, char *_data) {
	// ~  _ _   _ _  _ _ _   _ _ _
	//  length  addr  cmd    chksm

	char messageToReturn[100];
	messageToReturn[0] = 0;
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

	//serialFlush(serialHandle);

	while(1) {
		letter = serialGetchar(serialHandle);
		if (letter == 255) continue;	//timeout
		if (letter == 3) {	//ctrl+c
			cout << "(serialComm2.getMessage) End" << endl;
			serialClose(serialHandle);
			return 0;
		}
		if (letter == '~') {
			calcCS = 0;
			cout << endl << "CMD start!" << endl;

			getCharacters(serialHandle,ptr,2);	//length
			cout << "Length string: " << text << endl;
			length = atoi(text);
			//strcat(messageToReturn,text);
			cout << "(serialComm2.getMessage) Length: " << length << endl;

			argLength = length - 1 - 3;

			if (argLength < 0) argLength = 0;

			getCharacters(serialHandle,ptr,1);	//address
			//strcat(messageToReturn,text);
			*address = *text;
			//address = atoi(text);
			calcCS += getChecksum(address);
			cout << "(serialComm2.getMessage) Address: " << address << endl;

			if (*address != _address) {
				//means not our message
				sendAck(serialHandle, _address, 1);
				continue;
			}

			getCharacters(serialHandle,cmdptr,3);	//cmd
			strcat(messageToReturn,cmdptr);
			calcCS += getChecksum(cmd);
			cout << "(serialComm2.getMessage) CMD: " << cmd << endl;

			if (argLength > 0) {
					getCharacters(serialHandle,args,argLength);	//address
					//strcat(messageToReturn,args);
					calcCS += getChecksum(ptr);
					cout << "(serialComm2.getMessage) Args: " << address << endl;
			}

			getCharacters(serialHandle,ptr,3);	//address
			//strcat(messageToReturn,text);
			checksum = atoi(text);
			cout << "(serialComm2.getMessage) Checksum: " << checksum << endl;

			cout << "(serialComm2.getMessage) calcCS: " << calcCS << endl;	//Calculated Checksum includes Address, Command, and Args

			if (checksum == 0 || calcCS == 0) {
				break;
			} 
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
				break;
			}
		}
	}
	cout << "Message To Return: " << messageToReturn << endl;
	strcpy(_data,messageToReturn);
}

char* getCMD(char* frame, char* returned) {
	//assume message: ~LLACCC
	//LL LEngth
	//A address
	//CCC Command

	memcpy(returned, &frame[3], 3);
	cout <<"(getCMD): " << returned << "|" << endl;
	return returned;
}

int sendMessage(int serialHandle, char address, char cmd[3], char *args) {
	cout << "(serialComm2.sendMessage) sendMessage started" << endl;
	//if read args is ~ then no args
	int argsSize;
	char tempCMD[4] = {*cmd,*(cmd+1),*(cmd+2),0};
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
	strcat(finalMessage, tempCMD);
	if (argsSize != 0) strcat(finalMessage, args);
	char cstext[] = {address, 0};

	strcat(finalMessage,putChecksum(getChecksum(strcat(cstext,tempCMD))));	
	cout << "(serialComm2.sendMessage) Message: " << finalMessage << "|" << endl;
	//cout << "(serialComm.sendMessage) handle: " << serialHandle << endl;
	//cout << "(serialComm.sendMessage) addr: " << address << endl;
	serialFlush(serialHandle);
	while (1) {
		sendCharacters(serialHandle, finalMessage);
		while (waitAck(serialHandle, address) == false) {}
		break;
	}
	cout << "(serialComm2.sendMessage) sendMessage completed" << endl;
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
	cout << "(serialComm2.waitAck) waitAck started" << endl;
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

#endif