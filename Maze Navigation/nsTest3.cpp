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

int main(){
	bool done = false;
	naviSyst ns = naviSyst();
	char nextPath;
	ns.incCellsVisited(); // Must count the starting cell towards cells visited
	
	ns.callWallSensorsSim();
	ns.checkFreePaths();
	nextPath = ns.getNextPath();
	//cout << "Next path is: " << nextPath << endl;
	cout << "Starting cell: " << ns.getCurrNum() << endl;
	
	while(!done){
	
		ns.callWallSensorsSim();
		ns.checkFreePaths();
		nextPath = ns.getNextPath();
		
		if((nextPath == 'X')&&(ns.getTotalCells() != ns.getCellsVisited())){
			// cout << "Revisiting cell: " << ns.peekRevisit() << endl;
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
	ns.printTravHist();

	//Solves maze in 50 moves (49 not including scanning the initial square for walls)
	
	return 0;

}