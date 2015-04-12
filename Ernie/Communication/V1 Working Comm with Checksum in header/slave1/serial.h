#ifndef SERIAL_H
#define SERIAL_H

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
	temp = 0;
}

void sendCharacters(int serialHandle, char* text) {
	while (*text++ != 0) {
		serialPutchar(serialHandle, *text);
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
	int length, address, checksum;
	char text[10];
	char* ptr = text;
	char cmd [4];
	int calcCS = 0;

	int argLength = 0;
	char args[100];

	serialFlush();

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
			cout << "CMD start!" << endl;

			getCharacters(serialHandle,ptr,2);	//length
			length = atoi(text);
			cout << "Length: " << length << endl;

			argLength = length - 1 - 3 - 3;

			if (argLength < 0) argLength = 0;

			getCharacters(serialHandle,ptr,1);	//address
			address = atoi(text);
			calcCS += getChecksum(ptr);
			cout << "Address: " << address << endl;

			if (address != _address) {
				//means not our message
				continue;
			}

			getCharacters(serialHandle,cmd,3);	//cmd
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
			} else {
				serialFlush();
				//good, send "~~" back for ack
				sendAck(serialHandle, _address, 1);
				break;	//break out of while
			}
		}
	}
}

int sendMessage(int serialHandle, char address, char cmd[3], char *args) {
	//if read args is ~ then no args
	int argsSize;
	if (*args == "~") argsSize = 0;
	else {
		//loop to count argsSize
	}
	int size = 1+3+argsSize;
	char message[100];
	char *finalMessage = message;

	strcpy(finalMessage, "~");
	strcat(finalMessage, atoi(size));
	strcat(finalMessage, address);
	strcat(finalMessage, cmd);
	if (argsSize != 0) strcat(finalMessage, args);

	while (1) {
		sendMessage(serialHandle, finalMessage);
		if (waitAck(serialHandle, _address)) break;
	}

	return 1;
}

int sendAck(int serialHandle, char address, int result) {
	if (result == 0) sendCharacters(serialHandle, "~~0");
	else sendCharacters(serialHandle, "~~1");
}

bool waitAck(int serialHandle, char address) {
	char single;
	char message[100];
	switch (address) {
		case 1: //L
			while (1) {
				serialGetchar(serialHandle, single);
				if (single == "~") {
					serialGetchar(serialHandle, single);
					if (single == "~") {
						//done!
						serialGetchar(serialHandle, single);
						if (single == "1") {
							//ack good
							return true;
						} else {
							//ack bad
							return false;
						}
					}
				}
			}
			break;
		case 2: //R
			break;
	}
}

#endif