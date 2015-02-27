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
	naviSyst ns = naviSyst();
	char nextPath;
	ns.incCellsVisited(); // Must count the starting cell towards cells visited
	
	for(int i = 0; i < 30; i++){ //Algorithm reaches cell 38 at i = 30, meaning it must now return to cell 9
		ns.callWallSensorsSim();
		ns.checkFreePaths();
		nextPath = ns.getNextPath();
		//cout << "Next path is: " << nextPath << endl;
		if((nextPath == 'X')&&(ns.getTotalCells() != ns.getCellsVisited())){
			cout << "Revisiting cell: " << ns.peekRevisit() << endl;
			ns.revisitCell();
			cout << "Now in cell for revisiting: " << ns.getCurrNum() << endl;
		}
		else if((nextPath == 'X')&&(ns.getTotalCells() == ns.getCellsVisited())){
			cout << "Revisiting cell: " << ns.peekRevisit() << endl;
			ns.revisitCell();
			cout << "Now in cell for revisiting: " << ns.getCurrNum() << endl;
		}
		else{
			ns.moveCell(nextPath);
			cout << "Now in cell: " << ns.getCurrNum() << endl;
			ns.incCellsVisited();
		}
	}
	
	if(ns.getTotalCells() == ns.getCellsVisited()){
		ns.moveToFin();
		cout << "Now in ending cell: " << ns.getCurrNum() << endl;
	}
	
	cout << "Total cells to visit: " << ns.getTotalCells() << endl;
	cout << "Unique cells visited: " << ns.getCellsVisited() << endl;
	cout << "Travel history: ";
	ns.printTravHist();

	//Solves maze in 50 moves (49 not including scanning the initial square for walls)
	
	return 0;

}