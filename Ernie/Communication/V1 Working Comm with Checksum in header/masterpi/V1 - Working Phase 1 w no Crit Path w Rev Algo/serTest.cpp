#include <wiringPi.h>
#include <wiringSerial.h>
  
#include <iostream>

using namespace std;

char receiveChar(int serialHandle);
char sendChar(int serialHandle, char data);

int main(){
	int serialHandle; 
	char send;

	serialHandle = serialOpen("/dev/ttyAMA0", 9600); // String argument found in dev folder, this opens serial comm.
	
	serialFlush(serialHandle);
	while(1){
		cin >> send;
		sendChar(serialHandle, send);
	}
	return 0;
}

char receiveChar(int serialHandle){
	char rec;
	rec = serialGetchar(serialHandle);
	return rec;
}

char sendChar(int serialHandle, char data){
	serialPutchar(serialHandle, data);
	return data;
}