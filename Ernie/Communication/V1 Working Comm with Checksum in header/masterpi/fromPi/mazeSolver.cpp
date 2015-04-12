/*
Feb 27, 2015
This is the code that the robot will use to solve the physical maze
March 2, 2015
Code includes ultrasonic sensors and serial
March 5, 2015
Log file functions added
*/

#define LOG // Enables log file mode

#include <wiringPi.h>
#include <stdio.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include "naviSyst2.h"

using namespace std;

int main(){
	ofstream file("testLog.txt");	
	bool done = false;
	naviSyst ns = naviSyst();
	char nextPath;
	ns.Tiva_init();
	ns.serialCalibrate();
	ns.incCellsVisited(); // Must count the starting cell towards cells visited
	

	/*
	int count=5;
	//Wait for start button
	while (digitalRead(START_GPIO) == 0) {
		if (digitalRead(SIZE_GPIO) == 1) { 
			count++;
			if (count <= 7) {
				//good
			} else {
				//reset
				count = 5;
			}
		}
		set activeMap according to count
	}*/
	
	wiringPiSetup();
	ns.Tiva_init();
	// ns.callWallSensorsSim();
	// ns.checkFreePaths();
	// nextPath = ns.getNextPath();
	// cout << "Next path is: " << nextPath << endl;
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	ns.scanWalls();
	ns.logMapInfo(file);
	
	while(!done){
	
		ns.scanWalls();
		ns.checkFreePaths();
		if(!ns.isEmptyRevisit()){
			cout << " Cell to revisit: " << ns.peekRevisit() << endl;
		}
		else{
			cout << " No cell in revisit stack" << endl;
		}
		nextPath = ns.getNextPath();
		
		if((nextPath == 'X')&&(ns.getTotalCells() != ns.getCellsVisited())){
			cout << "Revisiting cell: " << ns.peekRevisit() << endl;
			ns.printTravHist();
			ns.revisitCell();
			cout << "Revisited cell: " << ns.getCurrNum() << endl;
		}
		else if((nextPath == 'X')&&(ns.getTotalCells() == ns.getCellsVisited())){
			cout << "Moving to ending cell. " << endl;
			ns.moveToFin();
			cout << "Reached ending cell: " << ns.getCurrNum() << endl;
			done = true;
		}
		else{
			ns.moveCell(nextPath);
			cout << "Current cell: " << ns.getCurrNum() << endl;
			ns.incCellsVisited();
		}
		cout << "Current face: " << ns.getCurrFace() << endl;
	}
	
	
	cout << "Total cells to visit: " << ns.getTotalCells() << endl;
	cout << "Unique cells visited: " << ns.getCellsVisited() << endl;
	cout << "Total L/R/About rotates: " << ns.leftRotates << "/" << ns.rightRotates << "/" << ns.aboutFaceRotates << endl;
	cout << "Travel history: ";
	ns.printTravHist();
	ns.logTravHist(file);
	cout << "Critical path: "; 
	ns.printCritPath();
	ns.logCritPath(file);

	
	return 0;

}