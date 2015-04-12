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

//This program tests the maze algorithm solving a 5x5 maze

int main(){ 	
	bool done = false;
	naviSyst ns = naviSyst();
	char nextPath;
	ns.incCellsVisited(); // Must count the starting cell towards cells visited

	/*wiringPiSetup();
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
	// ns.callWallSensorsSim();
	// ns.checkFreePaths();
	// nextPath = ns.getNextPath();
	cout << "Next path is: " << nextPath << endl;
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	
	while(!done){
	
		ns.callWallSensorsSim();
		ns.checkFreePaths();
		ns.logMapInfo(file);
		nextPath = ns.getNextPath();
		
		if((nextPath == 'X')&&(ns.getTotalCells() != ns.getCellsVisited())){
			cout << "Revisiting cell: " << ns.peekRevisit() << endl;
			ns.printTravHist();
			ns.revisitCell();
			cout << "Revisited cell: " << ns.getCurrNum() << endl;
		}
		else if((nextPath == 'X')&&(ns.getTotalCells() == ns.getCellsVisited())){
			ns.moveToFin();
			cout << "Reached ending cell: " << ns.getCurrNum() << endl;
			done = true;
		}
		else{
			ns.moveCell(nextPath);
			cout << "Current cell: " << ns.getCurrNum() << endl;
			ns.incCellsVisited();
		}
	}
	
	cout << "Total cells to visit: " << ns.getTotalCells() << endl;
	cout << "Unique cells visited: " << ns.getCellsVisited() << endl;
	cout << "Total L/R/About rotates: " << ns.leftRotates << "/" << ns.rightRotates << "/" << ns.aboutFaceRotates << endl;
	cout << "Travel history: ";
	ns.logMapInfo(file);
	ns.printTravHist();
	ns.logTravHist(file);
	cout << "Critical path: "; 
	ns.printCritPath();
	ns.logCritPath(file);
	file.close();
	
	return 0;

}