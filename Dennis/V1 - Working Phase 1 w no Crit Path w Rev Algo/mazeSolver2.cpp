//This code is intended to work serially with the other two raspberry pis that take pictures

/*
March 19, 2015
-Started serial comm functions with camera raspberry pi
-Started code to compile easter egg data from auxilliary pis to write to USB
*/

/*March 26, 2015
-Started camera system class to coordinate the three cameras
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
#include "camSyst.h"

using namespace std;

int main(){

	bool done = false;
	naviSyst ns = naviSyst();
	camSyst cs = camSyst();
	char nextPath;
	ns.incCellsVisited(); // Must count the starting cell towards cells visited	
	wiringPiSetup();
	ns.Tiva_init();
		// ns.callWallSensorsSim();
		// ns.checkFreePaths();
		// nextPath = ns.getNextPath();
		// cout << "Next path is: " << nextPath << endl;
		
	//Phase 1
	
	//First cell protocol, must check if the rear wall has an easter egg	
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	ns.scanWalls();
	cs.takePics(ns.getCurrFace(), ns.checkWall('N'), ns.checkWall('E'), false, ns.checkWall('W'));
	ns.faceLeft();
	cs.takePics(ns.getCurrFace(), false, false, true, false); // Left auxi pi will have the ASCII character of the first cell's southern wall. This means that the last two values in the eggs stack for each auxi pi will be for the starting cell.
	ns.faceRight();
	ns.logMapInfo(file);
	
	while(!done){

		cs.addUniquePath(ns.getCurrNum()); // add the new cell in the unique path order
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
			cs.takePics(ns.getCurrFace(), ns.checkWall('N'), ns.checkWall('E'), ns.checkWall('S'), ns.checkWall('W'));
			cout << "Current cell: " << ns.getCurrNum() << endl;
			ns.incCellsVisited();
		}
		cout << "Current face: " << ns.getCurrFace() << endl;
	}
	
	// cs.consolEggs(ns.getMapSize());
	cout << "Total cells to visit: " << ns.getTotalCells() << endl;
	cout << "Unique cells visited: " << ns.getCellsVisited() << endl;
	cout << "Total L/R/About rotates: " << ns.leftRotates << "/" << ns.rightRotates << "/" << ns.aboutFaceRotates << endl;
	cout << "Travel history: ";
	ns.printTravHist();
	ns.logTravHist(file);
	cout << "Critical path: "; 
	ns.printCritPath();
	ns.logCritPath(file);

	//Phase 2
	
	ns.resetCurrCell();
	ns.travCritPath();
	
	return 0;
}

