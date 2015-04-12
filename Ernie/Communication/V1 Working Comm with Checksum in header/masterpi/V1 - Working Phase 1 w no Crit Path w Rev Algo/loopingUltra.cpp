#include <time.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>

#include <wiringPi.h>

using namespace std;
//*****************************************************
//*****************************************************
//********** DELAY FOR # uS WITHOUT SLEEPING **********
//*****************************************************
//*****************************************************
//Using delayMicroseconds lets the linux scheduler decide to jump to another process.  Using this function avoids letting the
//scheduler know we are pausing and provides much faster operation if you are needing to use lots of delays.
void DelayMicrosecondsNoSleep (int delay_us)
{
	long int start_time;
	long int time_difference;
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;		//Get nS value
	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0)
			time_difference += 1000000000;				//(Rolls over every 1 second)
		if (time_difference > (delay_us * 1000))		//Delay for # nS
			break;
	}
}
int getUltra2(int unit, int compare);

int getUltra(int unit) {
	int count = 3;
	int i;
	int array = 0;
	for (i = 0; i < count; i++) array += getUltra2(unit,800);
	
	cout << "Sensor " << unit << ": " << array << endl;
	if (array >= 2) return 1;
	else return 0;
}

int getUltra2(int unit, int compare) {
	cout << "Calling sensor " << unit << ": ";
	string response;
	int triggerPin, echoPin;
	//routine
	wiringPiSetup();

	switch(unit) {
		default:	//goes to case 1
		case 0:
			triggerPin = 8;		//pi3
			echoPin = 7;
			break;
		case 1:
			triggerPin = 12;		//pi19
			echoPin = 13;		//pi21
			break;
		case 2:
			triggerPin = 10;	//pi24
			echoPin = 11;		//pi26
			break;
	}
	//initialize pins
	pinMode(triggerPin, OUTPUT);	//trigger
	digitalWrite(triggerPin, LOW);
	pinMode(echoPin, INPUT);	//echo

	//trigger 10us
	digitalWrite(triggerPin, HIGH);		//trigger HIGH
	delayMicroseconds(15);
	digitalWrite(triggerPin, LOW);

	//detecting echo
	struct timespec gettime_now, gettime_later, gettime_hang;

	/// To prevent overflow ///
	struct timespec current;
	current.tv_sec = 0;
	current.tv_nsec = 0;
	clock_settime(CLOCK_REALTIME, &current);
	/// ******************* ///
	clock_gettime(CLOCK_REALTIME, &gettime_hang);	
	while (digitalRead(echoPin) == 0) {
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		if(gettime_hang.tv_nsec > gettime_now.tv_nsec){
			return getUltra(unit);
		}
	}

	while (digitalRead(echoPin) == 1) {
		clock_gettime(CLOCK_REALTIME, &gettime_later);
	}
	if (gettime_later.tv_nsec < gettime_now.tv_nsec) return getUltra2(unit, compare);
	long difference = gettime_later.tv_nsec - gettime_now.tv_nsec;
	cout << "Time = " << difference/1000 << "us" << endl;
	

	//cout << "T/850= " << (difference/1000)/1400 << "num cells away" << endl;
	//if ((difference/1000) < 700) cout << unit << " is True" << endl;
	//else cout << unit << " is False" << endl;
	pinMode(triggerPin, INPUT);
	pinMode(echoPin, INPUT);
	
	//cout << "(loopingUltra.getUltra) Difference: " << difference << endl;
	if ((difference/1000) < compare) return 1;
	else return 0;
}

char getAllUltra() {
	char outChar = 0;
	if (getUltra(1)) {
		outChar += 0b001;
	}
	if (getUltra(2)) {
		outChar += 0b010;
	}
	if (getUltra(3)) {
		outChar += 0b100;
	}
	return outChar;
}

/*int main() {
	cout << "START!" << endl;
	char output;

	output = getAllUltra();

	cout << "Output: " << (int) output << endl;

	return 0;
}*/
