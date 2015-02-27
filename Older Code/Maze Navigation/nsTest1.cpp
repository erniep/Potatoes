#include <stdio.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include "naviSyst.h"

using namespace std;

// This code allows you to manually travel through the maze.

int main(){
	naviSyst ns = naviSyst();
	char opt;
	ns.callWallSensorsSim();
	cout << "Starting face: " << ns.getCurrFace()<< endl;
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	cout << "Where would you like to travel to?" << endl;
	cout << "Please use wasd to move" << endl;
	cout << "Please use p to perform a wall scan" << endl;
	cout << "Please use o to check your travel history" << endl;
	
	while(opt != 'q'){ 
		cin >> opt;
		if(opt == 'w'){
			ns.moveCell('N');
			ns.callWallSensorsSim();
		}
		else if(opt == 'a'){
			ns.moveCell('W');
			ns.callWallSensorsSim();
		}
		else if(opt == 's'){
			ns.moveCell('S');
			ns.callWallSensorsSim();
		}
		else if(opt == 'd'){
			ns.moveCell('E');
			ns.callWallSensorsSim();
		}
		else if(opt == 'p'){
			ns.callWallSensorsSim();
			cout << "N: " << ns.getCurrCell()->getWall('N') << " E: " << ns.getCurrCell()->getWall('E') << " S: " << ns.getCurrCell()->getWall('S') << " W: " << ns.getCurrCell()->getWall('W') << endl;
		}
		else if(opt == 'o'){
			ns.printTravHist();
		}
		else{
			cout << "Input not recognized" << endl;
		}
		cout << "Current face: " << ns.getCurrFace()<< endl;
		cout << "Current cell: " << ns.getCurrNum() << endl;
	}	
	
}