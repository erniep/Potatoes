#include "wiringPi.h"
#include "loopingUltra.cpp"

using namespace std;

int main() {
	char blah;
	
	while(1) {
		getUltra2(0,800);
		getUltra2(1,800);
		getUltra2(2,800);
		cin >> blah;
	}
	return 0;
}