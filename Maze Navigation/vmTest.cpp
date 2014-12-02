#include <stdio.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include "virtualMap.h"

using namespace std;

//Move around the maze using a virtualMap. This test does not take into account walls.

int main(){

	char dir;
	virtualMap vm = virtualMap(); // Create a default 5x5 map
	cout << vm.getCurrNum() << endl;
	cout << "Where would you like to travel to?" << endl;
	cout << "Please use wasd to move" << endl;
	
	while(dir != 'q'){ 
		cin >> dir;
		if(dir == 'w'){
			vm.move('N');
		}
		else if(dir == 'a'){
			vm.move('W');
		}
		else if(dir == 's'){
			vm.move('S');
		}
		else if(dir == 'd'){
			vm.move('E');
		}
		else{
		}
		cout << vm.getCurrNum() << endl;
	}
}