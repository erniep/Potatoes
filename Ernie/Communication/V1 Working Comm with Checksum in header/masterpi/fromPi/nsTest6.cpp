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

//March 26, 2015
//This program is intended to run the critical path after solving the maze

int main(){ 	
	bool p1done = false;
	naviSyst ns = naviSyst();
	char nextPath;
	ns.incCellsVisited(); // Must count the starting cell towards cells visited


	cout << "Next path is: " << nextPath << endl;
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	
	// Phase 1
	
	while(!p1done){
	
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
			p1done = true;
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

	//Phase 2
	
	ns.resetCurrCell();
	ns.travCritPath();
	
	return 0;

}