#include "virtualMap.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <stack>
#include <deque>
#include <list>
#define LOG // Enables log file mode
//#define ACK // Enables having to receive simulated acknowledgement before continuing the maze algorithm after sending a movement instruction to the tiva

ofstream file("testLog.txt");	
using namespace std;

/*January 24th, 2015: 
-Fixed revisitCell function. 
-Changed wallSensorsSim to Ernie's version.
-activeMap can now be changed with setActiveMap function
*/

/*February 27, 2015:
-Added moveCell2(), allows the robot to travel backwards if necessary. Only used to when revisiting a cell. This is intended to reduce the number of overall 90 degree turns.
*/

/*March 5, 2015:
-Added logfile functions
*/

/*March 6, 2015
-Ernie fixed issue with revisitCell where routeToOpt list was not being initialized properly because popTravHist() would return the top of travHist and THEN pop travHist.
*/

/*March 26, 2015
-Added resetCurrCell() and travCritPath()
*/

//#define MAZE5
// #define MAZE6
//define MAZE7

// #ifdef MAZE5
// #define TOTALCELLS 26

// #endif

// #ifdef MAZE6
// #define TOTALCELLS 37

// #endif

// #ifdef MAZE7
// #define TOTALCELLS 49

// #endif


class naviSyst{

private:
	
	//[Internal variables]
	virtualMap map4x4, map5x5, map6x6, map7x7;
	virtualMap* activeMap; // Points at the map that the robot is using, since the map size changes for each round of the competition
	int totalCells; // Total number of cells to visit for the current active map
	int cellsVisited; // Number of unique cells currently visited
	char currFace; // Current cardinal direction of front face
	queue<int> critPath; // Cell# order of critical path
	stack<int> travHist; // Travel history of robot. Used to retrace steps
	stack<int> revisit; // Stack of cells that need to be revisited because there are still paths to explore there
	queue<int> eggLocs; // Cell numbers of cells that contain Easter eggs

	//[Sensor variables]
	bool wallReadings[4]; // Values returned by the wall sensors for the current cell. [0], [1], and [2] correspond to left, front, and right sensors. There is a fourth index in case a fourth sensor is added to the south of the robot. Since the wall sensors do not return values relative to the NESW convention used in this code, this function must use the robot's current face to translate these sensor readings into the NESW convention.
	
public: 
	
	//[Constructors]
	naviSyst();
	
	//[Debug variables]
	int leftRotates;
	int rightRotates;
	int aboutFaceRotates;	

	//[Navigation get functions]
	int getTotalCells();
	int getCellsVisited();
	char getCurrFace();
	int getCurrNum();
	unitCell* getCurrCell();
	int peekCritPath();
	int popCritPath();
	int printCritPath();
	int peekTravHist();
	int popTravHist();
	int printTravHist();
	int peekRevisit();
	int popRevisit();
	int printRevisit();
	int peekEggLocs();
	int popEggLocs();
	
	//[Navigation set functions]
	void setCellsVisited(int n);
	void incCellsVisited();
	void setCurrFace(int crdnlDir);
	void resetCurrCell();
	void pushCritPath(int cellNum);
	void pushTravHist(int cellNum);
	void pushRevisit(int cellNum);
	void pushEggLocs(int eggLoc);
	void setActiveMap(string mapSize); // Set the active map of the robot
	
	//[Movement functions]
	int moveCell(int nextDir);
	int moveCell(char nextDir); // Travel to an adjacent cell indicated by the cardinal direction passed as an argument. Checks against current face of robot, then updates the new face when the robot reaches the new cell. This movement is basic and does not use strafing.
	int moveCell2(int nextDir);	
	int moveCell2(char nextDir); // Move to an adjacent cell moving backwards, if possible. Reduces rotations
	void revisitCell(); // Travel to a cell that must be revisited. Intended to be called after the getNextPath function returns 'X'
	int moveToFin(); // Travel to the end of the maze after all the cells have been visited.
	void moveToCell(int returnCell);
	void travCritPath();
	int forward(); // Move forward until the tivaC returns a flag saying that the robot is in a new cell.
	int back();
	int strafeLeft();
	int strafeRight();
	int faceLeft();
	int faceRight();
	int aboutFace();
	void waitForAck(); // Simulates serial handshaking for each movement instruction
	
	//[Unit cell check functions]
	bool callUltras(); //Function to call the wall sensors. Updates the wallReadings array (Left, midddle, and right correspond to indexes 0, 1, and 2). This function assumes that the direction not covered by the wall sensors has no wall. 
	void callWallSensorsSim(); //Dummy function to call the wall sensors. Directly updates the current cell's walls. In this version, this uses Ernie's callWallSensorsSim code
	void callRPiCam(); // Dummy function to call the raspberry pi camera.
	int firstCellProtocol(); // Information retrieval function for the first cell, which has a few differences from other cells in the maze. This function must be called first to initialize the maze navigation algorithm. Assuming the robot is pointed north at the initial cell, this function ensures that the wall without a distance sensor is recorded as a wall in the virtual map. The wallScan() function assumes that the wall without a distance sensor is an open path, since it's assumed the robot entered the cell from there.
	void scanWalls(); // Checks for walls in the current cell and update the virtual map to reflect the new information.
	void checkFreePaths(); // Intended to be called after the scanWalls function has been used to detect and record walls in the current cell. This function will push onto the current cell's unexplPaths stack the free paths that the robot can travel to.
	char getNextPath(); // Intended to be called after the checkFreePaths function. Determines the next direction to travel to, based on the unitCell's unexploredPaths stack. Returns a capital 'X' if there is no more unexplored paths to travel.

	//[Diagnostic functions]
	void logMapInfo(ofstream & f);
	void logTravHist(ofstream & f);
	void logCritPath(ofstream & f);
};

naviSyst::naviSyst(){ 
	leftRotates = 0;
	rightRotates = 0;
	aboutFaceRotates = 0;
	map4x4.set4x4();
	map6x6.set6x6();
	map7x7.set7x7();
	activeMap = &map7x7;
	//activeMap = &map6x6;
	// activeMap = &map7x7;	
	totalCells = activeMap->getTotalCells();
	cellsVisited = 0; // Number of unique cells visited
	currFace = 'N'; // Default face of the robot is north
	pushTravHist(getCurrNum()); // Add the beginning cell to the travel history. Necessary for critical path calculation and route decisions
}

int naviSyst::getTotalCells(){
	return totalCells;
}
int naviSyst::getCellsVisited(){
	return cellsVisited;
}

char naviSyst::getCurrFace(){
	return currFace;
}

int naviSyst::getCurrNum(){
	unitCell* c = activeMap->getCurr();
	return c->getNum();
}

unitCell* naviSyst::getCurrCell(){
	unitCell* c = activeMap->getCurr();
	return c;
}

int naviSyst::printCritPath(){

	queue<int> optPath; 
	list<int> routeToOpt; // Queue that must be optimized
	stack<int> temp; // Stack used to restore the travelHist stack

	 temp.push(popTravHist()); // Top of the travHist stack is the current node, so we don't want to include it in the optPath queue
	// cout << "(naviSyst::printCritPath) Travel history: " << endl; 
	// printTravHist();
	int travHistTop = peekTravHist();
	while(!travHist.empty()){ // Push the cells that lead to the destination onto the routeToOpt list
		routeToOpt.push_back(travHist.top());
		temp.push(travHist.top());
		travHist.pop();
		if(!travHist.empty()){
			travHistTop = travHist.top();
		}
	}
	
	while(!temp.empty()){ //Restore the travel history stack
		pushTravHist(temp.top());
		temp.pop();
	}
	
	//Begin Looping Route Elimination Algorithm (LREA) on the routeToOpt list
	queue<int> routeToOptRev; //aka RTOR, used to check for loops against the top value of the CFL. I want to iterate through the routeToOpt list in reverse because I want the longest loop. (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then the longest loop is 16, 17, 18, 17, 16, not 16, 17, 16).
	stack<int> checkForLoop; // aka CFL. This stack contains values that must be checked for loops (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then 16 is a cell where a loop begins)
	queue<int> restoreRTOR; // Used to restore the RTOR so we can iterate through it again through each value of the CFL stack that needs to be checked.
	int cfl; // Current top value of the CFL that is being checked to see if it is the start of a redundant loop.
	
	//LREA Step 1: Initialize RTOR and CFL 
	while(routeToOpt.empty() == false){ // The RTOR list must contain the contents of routeToOpt in reverse. I know I could have just accessed the last element of routeToOpt to accomplish the same thing, but it's easier for me to handle this algorithm this way
		routeToOptRev.push(routeToOpt.back());
		checkForLoop.push(routeToOpt.back());
		routeToOpt.pop_back();
	}
	checkForLoop.push(getCurrNum()); // Need to have the current occupied cell in the CFL stack because we want to check loops that may return to the current cell
	
	// Continue the LREA until the checkForLoop stack is empty, i.e., all possible values loops have been checked
	while(checkForLoop.empty() == false){
		cfl = checkForLoop.top();
		//LREA Step 2: Check front value of CFL against all values of RTOR
		while(routeToOptRev.empty() == false){
			//LREA Step 3: If the current front value in the RTOR does not match cfl, then store the non-matching value in the restoreRTOR stack 
			if(cfl != routeToOptRev.front()){
					restoreRTOR.push(routeToOptRev.front());
					routeToOptRev.pop();
			}
			//LREA Step 4a: If the current front value in the RTOR matches cfl, then save the value onto the optPath
			else if(cfl == routeToOptRev.front()){
				optPath.push(cfl);
				//LREA Step 4b: If the RTOR is empty after the matching cfl value, then no loop actually exists. Must verify that a loop actually exists to get rid of it.
				routeToOptRev.pop();
				if(routeToOptRev.empty() == false){
					//LREA Step 4c: If the RTOR is not empty after the matching cfl value, then discard the rest of the contents, since the remaining values in the RTOR are the redundant loop.
					while(routeToOptRev.empty() == false){
						routeToOptRev.pop();
					}
					//LREA Step 4d: Since a loop is found, the CFL must be popped to omit iterating through the loop. Pop the CFL stack until the top value is repeated. Pop once more so the value right after the repeated one will be checked during the next iteration of the LREA.
					checkForLoop.pop();
					while(checkForLoop.top() != cfl){
						checkForLoop.pop();
					}
				}
			}
		}
		//LREA Step 4e: Restore the RTOR
		while(restoreRTOR.empty() == false){
			routeToOptRev.push(restoreRTOR.front());
			restoreRTOR.pop();
		}
		//LREA Step 5: Pop the checkForLoop stack to ready the next value to be checked
		checkForLoop.pop();	
	}
	
	//Print out critical path
	if(optPath.front() != activeMap->getFin()->getNum()){ // If the rear of the critical path is not the ending cell, add the ending cell
		queue<int> tempOpt;
		while(optPath.empty() == false){
			tempOpt.push(optPath.front());
			optPath.pop();
		}
		optPath.push(activeMap->getFin()->getNum());
		while(tempOpt.empty() == false){
			optPath.push(tempOpt.front());
			tempOpt.pop();
		}
	} 
	critPath = optPath;
	cout << "(printCritPath)" << endl; 
	while(optPath.empty() == false){
		cout << optPath.front() << " "; 
		optPath.pop();
	}
	return 0;
}

int naviSyst::peekTravHist(){
	//cout << "(peekTravHist) Size: " << travHist.size() << endl;
	int p = travHist.top();
	return p;
}

int naviSyst::popTravHist(){
	int t = travHist.top();
	travHist.pop();
	return t;
}

int naviSyst::printTravHist(){
	int moves = 0;
	stack<int> temp = travHist;
	cout << "(printTravHist) Stack top: " << endl;
	while(!temp.empty()){
		cout << temp.top() << " ";
		temp.pop();
		moves++;
	}
	cout << "Total moves: " << moves << endl;
	cout << endl;
	return 0;
}

int naviSyst::peekRevisit(){
	int t = revisit.top();
	return t;
}

int naviSyst::popRevisit(){
	int t = revisit.top();
	revisit.pop();
	return t;
}

int naviSyst::printRevisit(){
	stack<int> temp = revisit;
	cout << "(printRevisit) Stack top: ";
	while(!temp.empty()){
		cout << temp.top() << " ";
		temp.pop();
	}
	cout << endl;
	return 0;
}

void naviSyst::setCellsVisited(int n){
	cellsVisited = n;
}

void naviSyst::incCellsVisited(){
	cellsVisited++;
}

void naviSyst::setCurrFace(int crdnlDir){
	currFace = crdnlDir;
}

void naviSyst::resetCurrCell(){
	activeMap->setCurr(activeMap->getStart());
}

void naviSyst::pushCritPath(int cellNum){
	critPath.push(cellNum);
}

void naviSyst::pushTravHist(int cellNum){
	travHist.push(cellNum);
}

void naviSyst::setActiveMap(string mapSize){
	if(strcmp(&mapSize[0], "6x6") == 0){
		activeMap = &map6x6;
	}
	else if(strcmp(&mapSize[0], "7x7") == 0){
		activeMap = &map7x7;
	}
	else{
		activeMap = &map5x5;
	}
}

void naviSyst::pushRevisit(int cellNum){
	revisit.push(cellNum);
}

int naviSyst::moveCell(int nextDir){
	if (nextDir == 0){
		moveCell('N');
	}
	else if(nextDir == 1){
		moveCell('E');
	}
	else if(nextDir == 2){
		moveCell('S');
	}
	else if(nextDir == 3){
		moveCell('W');
	}
	else{
		cout << "(naviSyst::moveCell) Invalid direction" << endl; 
	}
}

int naviSyst::moveCell(char nextDir){
	unitCell* curr = activeMap->getCurr();
	if(curr->getWall(nextDir) == true){
		cout << "(naviSyst::moveCell) Cannot move in that direction, wall exists " << endl;
		return 1;
	}
	else if (currFace == 'N'){
		 if(nextDir == 'N'){
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			aboutFace();
			aboutFaceRotates++;
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			faceLeft();
			leftRotates++;
			cout << "3to4 1" << endl;
			forward();
			cout << "3to4 2" << endl;
			curr = curr->getAdj('W');
			currFace = 'W';
		}			
	}
	else if (currFace == 'E'){
		if(nextDir == 'N'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			aboutFace();
			aboutFaceRotates++;
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}				
	}
	else if (currFace == 'S'){
		if(nextDir == 'N'){
			aboutFace();
			aboutFaceRotates++;
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}				
	}
	else if (currFace == 'W'){
		if(nextDir == 'N'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			aboutFace();
			aboutFaceRotates++;
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}				
	}
	pushTravHist(curr->getNum()); // Record the new cell travelled to
	activeMap->setCurr(curr); // Set the current position on the active map
	//callWallSensorsSim(); // Must remove this later, this is only for debugging
	return 0;
}

int naviSyst::moveCell2(int nextDir){
	if (nextDir == 0){
		moveCell2('N');
	}
	else if(nextDir == 1){
		moveCell2('E');
	}
	else if(nextDir == 2){
		moveCell2('S');
	}
	else if(nextDir == 3){
		moveCell2('W');
	}
	else{
		cout << "(naviSyst::moveCell) Invalid direction" << endl; 
	}
}

int naviSyst::moveCell2(char nextDir){
	unitCell* curr = activeMap->getCurr();
	if(curr->getWall(nextDir) == true){
		cout << "(naviSyst::moveCell) Cannot move in that direction, wall exists " << endl;
		return 1;
	}
	else if (currFace == 'N'){
		 if(nextDir == 'N'){
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			back();
			curr = curr->getAdj('S');
			currFace = 'N';
		}	
		else if(nextDir == 'W'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}			
	}
	else if (currFace == 'E'){
		if(nextDir == 'N'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			back();
			curr = curr->getAdj('W');
			currFace = 'E';
		}				
	}
	else if (currFace == 'S'){
		if(nextDir == 'N'){
			back();
			curr = curr->getAdj('N');
			currFace = 'S';
		}
		else if(nextDir == 'E'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}				
	}
	else if (currFace == 'W'){
		if(nextDir == 'N'){
			faceRight();
			rightRotates++;
			forward();
			curr = curr->getAdj('N');
			currFace = 'N';
		}
		else if(nextDir == 'E'){
			back();
			curr = curr->getAdj('E');
			currFace = 'E';
		}	
		else if(nextDir == 'S'){
			faceLeft();
			leftRotates++;
			forward();
			curr = curr->getAdj('S');
			currFace = 'S';
		}	
		else if(nextDir == 'W'){
			forward();
			curr = curr->getAdj('W');
			currFace = 'W';
		}				
	}
	pushTravHist(curr->getNum()); // Record the new cell travelled to
	activeMap->setCurr(curr); // Set the current position on the active map
	//callWallSensorsSim(); // Must remove this later, this is only for debugging
	return 0;	
}

void naviSyst::revisitCell(){
		
	queue<int> revisitRoute; // Queue used to travel to the cell that must be revisited
	list<int> routeToOpt; // Queue that must be optimized
	stack<int> temp; // Stack used to restore the travelHist stack
	unitCell* curr = activeMap->getCurr();
	int dest = popRevisit(); // Indicate the cell that must be revisited

	temp.push(popTravHist()); // Top of the travHist stack is the current node, so we don't want to include it in the revisitRoute queue
	// cout << "(naviSyst::revisitCell) Travel history: "; 
	// printTravHist();
	int travHistTop = peekTravHist();
	int count = 0;
	while((!travHist.empty())&&(travHistTop != dest)){ // Push the cells that lead to the destination onto the routeToOpt list
		// cout << "Count: " << count++ << endl;
		// cout << "q1: " << (!travHist.empty()) << endl;
		// cout << "q2: " << (travHistTop != dest) << endl;
		// cout << "top: " << travHistTop << " " << dest << endl;
		routeToOpt.push_back(travHist.top());
		temp.push(travHist.top());
		travHist.pop();
		travHistTop = travHist.top();
	}	
	routeToOpt.push_back(peekTravHist()); // Last value to enqueue onto the revisitRoute queue should be the destination
	temp.push(popTravHist());
	
	while(!temp.empty()){ //Restore the travel history stack
		pushTravHist(temp.top());
		temp.pop();
	}
	
	//Begin Looping Route Elimination Algorithm (LREA) on the routeToOpt list
	queue<int> routeToOptRev; //aka RTOR, used to check for loops against the top value of the CFL. I want to iterate through the routeToOpt list in reverse because I want the longest loop. (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then the longest loop is 16, 17, 18, 17, 16, not 17, 18, 17).
	stack<int> checkForLoop; // aka CFL. This stack contains values that must be checked for loops (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then 16 is a cell where a loop begins)
	queue<int> restoreRTOR; // Used to restore the RTOR so we can iterate through it again through each value of the CFL stack that needs to be checked.
	int cfl; // Current top value of the CFL that is being checked to see if it is the start of a redundant loop.
	
	//LREA Step 1: Initialize RTOR and CFL 
	while(routeToOpt.empty() == false){ // The RTOR list must contain the contents of routeToOpt in reverse. I know I could have just accessed the last element of routeToOpt to accomplish the same thing, but it's easier for me to handle this algorithm this way
		routeToOptRev.push(routeToOpt.back());
		checkForLoop.push(routeToOpt.back());
		routeToOpt.pop_back();
	}
	checkForLoop.push(getCurrNum()); // Need to have the current occupied cell in the CFL stack because we want to check loops that may return to the current cell
	
	// Continue the LREA until the checkForLoop stack is empty, i.e., all possible values loops have been checked
	while(checkForLoop.empty() == false){
		cfl = checkForLoop.top();
		//LREA Step 2: Check front value of CFL against all values of RTOR
		while(routeToOptRev.empty() == false){
			//LREA Step 3: If the current front value in the RTOR does not match cfl, then store the non-matching value in the restoreRTOR stack 
			if(cfl != routeToOptRev.front()){
					restoreRTOR.push(routeToOptRev.front());
					routeToOptRev.pop();
			}
			//LREA Step 4a: If the current front value in the RTOR matches cfl, then save the value onto the revisitRoute
			else if(cfl == routeToOptRev.front()){
				revisitRoute.push(cfl);
				//LREA Step 4b: If the RTOR is empty after the matching cfl value, then no loop actually exists. Must verify that a loop actually exists to get rid of it.
				routeToOptRev.pop();
				if(routeToOptRev.empty() == false){
					//LREA Step 4c: If the RTOR is not empty after the matching cfl value, then discard the rest of the contents, since the remaining values in the RTOR are the redundant loop.
					while(routeToOptRev.empty() == false){
						routeToOptRev.pop();
					}
					//LREA Step 4d: Since a loop is found, the CFL must be popped to omit iterating through the loop. Pop the CFL stack until the top value is repeated. Pop once more so the value right after the repeated one will be checked during the next iteration of the LREA.
					checkForLoop.pop();
					while(checkForLoop.top() != cfl){
						checkForLoop.pop();
					}
				}
			}
		}
		//LREA Step 4e: Restore the RTOR
		while(restoreRTOR.empty() == false){
			routeToOptRev.push(restoreRTOR.front());
			restoreRTOR.pop();
		}
		//LREA Step 5: Pop the checkForLoop stack to ready the next value to be checked
		checkForLoop.pop();	
	}
	
	while(!revisitRoute.empty()){ // Begin algorithm to travel back to cell
		// cout << "(naviSyst::revisitCell) Travelling to cell " << revisitRoute.front() << " from cell " << getCurrNum() << endl;
		if((curr->getNum() == revisitRoute.front())){ // If the next cell to travel to is already the current cell, get the next cell in the revisit route
			revisitRoute.pop();
		}
		else{
			for(int i = 0; i < 4; i++){
				if(curr->getAdj(i) == NULL){ //If the adjacent cell is non-existent (i.e., the current cell being reviewed is on the outer part of the map)
					continue;
				}
				else if((curr->getAdj(i))->getNum() == revisitRoute.front()){ // If the adjacent cell to the current cell matches the next cell on the revisitRoute queue, travel to it.
					moveCell2(i);
					cout << "(revisitCell) Current cell: " <<getCurrNum() << endl;
					revisitRoute.pop();
					curr = curr->getAdj(i); // Update the new current cell
					break;
				}
			}
		}
	}
}

int naviSyst::moveToFin(){
		
	queue<int> revisitRoute; // Queue used to travel to the cell that must be revisited
	list<int> routeToOpt; // Queue that must be optimized
	stack<int> temp; // Stack used to restore the travelHist stack
	unitCell* curr = activeMap->getCurr();
	
	//If the current cell is already the ending cell, then exit this function.
	if(curr->getNum() == activeMap->getFin()->getNum()){
		return 1;
	}
	int dest = activeMap->getFin()->getNum(); // Indicate the cell that must be revisited

	temp.push(popTravHist()); // Top of the travHist stack is the current node, so we don't want to include it in the revisitRoute queue
	// cout << "(naviSyst::revisitCell) Travel history: " << endl; 
	// printTravHist();
	int travHistTop = peekTravHist();

	while((!travHist.empty())&&(travHistTop != dest)){ // Push the cells that lead to the destination onto the routeToOpt list
		routeToOpt.push_back(travHist.top());
		temp.push(travHist.top());
		travHist.pop();
		travHistTop = travHist.top();
	}	
	routeToOpt.push_back(peekTravHist()); // Last value to enqueue onto the revisitRoute queue should be the destination
	temp.push(popTravHist());
	
	while(!temp.empty()){ //Restore the travel history stack
		pushTravHist(temp.top());
		temp.pop();
	}
	
	//Begin Looping Route Elimination Algorithm (LREA) on the routeToOpt list
	queue<int> routeToOptRev; //aka RTOR, used to check for loops against the top value of the CFL. I want to iterate through the routeToOpt list in reverse because I want the longest loop. (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then the longest loop is 16, 17, 18, 17, 16, not 16, 17, 16).
	stack<int> checkForLoop; // aka CFL. This stack contains values that must be checked for loops (e.g., if the CFL contains 9 (top), 16, 17, 18, 17, 16 (bottom), then 16 is a cell where a loop begins)
	queue<int> restoreRTOR; // Used to restore the RTOR so we can iterate through it again through each value of the CFL stack that needs to be checked.
	int cfl; // Current top value of the CFL that is being checked to see if it is the start of a redundant loop.
	
	//LREA Step 1: Initialize RTOR and CFL 
	while(routeToOpt.empty() == false){ // The RTOR list must contain the contents of routeToOpt in reverse. I know I could have just accessed the last element of routeToOpt to accomplish the same thing, but it's easier for me to handle this algorithm this way
		routeToOptRev.push(routeToOpt.back());
		checkForLoop.push(routeToOpt.back());
		routeToOpt.pop_back();
	}
	checkForLoop.push(getCurrNum()); // Need to have the current occupied cell in the CFL stack because we want to check loops that may return to the current cell
	
	// Continue the LREA until the checkForLoop stack is empty, i.e., all possible values loops have been checked
	while(checkForLoop.empty() == false){
		cfl = checkForLoop.top();
		//LREA Step 2: Check front value of CFL against all values of RTOR
		while(routeToOptRev.empty() == false){
			//LREA Step 3: If the current front value in the RTOR does not match cfl, then store the non-matching value in the restoreRTOR stack 
			if(cfl != routeToOptRev.front()){
					restoreRTOR.push(routeToOptRev.front());
					routeToOptRev.pop();
			}
			//LREA Step 4a: If the current front value in the RTOR matches cfl, then save the value onto the revisitRoute
			else if(cfl == routeToOptRev.front()){
				revisitRoute.push(cfl);
				//LREA Step 4b: If the RTOR is empty after the matching cfl value, then no loop actually exists. Must verify that a loop actually exists to get rid of it.
				routeToOptRev.pop();
				if(routeToOptRev.empty() == false){
					//LREA Step 4c: If the RTOR is not empty after the matching cfl value, then discard the rest of the contents, since the remaining values in the RTOR are the redundant loop.
					while(routeToOptRev.empty() == false){
						routeToOptRev.pop();
					}
					//LREA Step 4d: Since a loop is found, the CFL must be popped to omit iterating through the loop. Pop the CFL stack until the top value is repeated. Pop once more so the value right after the repeated one will be checked during the next iteration of the LREA.
					checkForLoop.pop();
					while(checkForLoop.top() != cfl){
						checkForLoop.pop();
					}
				}
			}
		}
		//LREA Step 4e: Restore the RTOR
		while(restoreRTOR.empty() == false){
			routeToOptRev.push(restoreRTOR.front());
			restoreRTOR.pop();
		}
		//LREA Step 5: Pop the checkForLoop stack to ready the next value to be checked
		checkForLoop.pop();	
	}
	
	while(!revisitRoute.empty()){ // Begin algorithm to travel back to cell
		// cout << "(naviSyst::revisitCell) Travelling to cell " << revisitRoute.front() << " from cell " << getCurrNum() << endl;
		if((curr->getNum() == revisitRoute.front())){ // If the next cell to travel to is already the current cell, get the next cell in the revisit route
			revisitRoute.pop();
		}
		else{
			for(int i = 0; i < 4; i++){
				if(curr->getAdj(i) == NULL){ //If the adjacent cell is non-existent (i.e., the current cell being reviewed is on the outer part of the map)
					continue;
				}
				else if((curr->getAdj(i))->getNum() == revisitRoute.front()){ // If the adjacent cell to the current cell matches the next cell on the revisitRoute queue, travel to it.
					moveCell(i);
					cout << "(moveToFin) Current cell: " <<getCurrNum() << endl;
					revisitRoute.pop();
					curr = curr->getAdj(i); // Update the new current cell
					break;
				}
			}
		}
	}
	return 0;
}

void naviSyst::moveToCell(int returnCell){
	int endCellNum = returnCell; //Get the cell number of the ending cell
	stack<int> travHistCopy = travHist;
	deque<int> routeToFin; // Will contain the route to the ending cell from whatever cell the robot is currently in
	unitCell* currCopy = getCurrCell(); // unitCell pointer used to plan out the direct route to the ending cell
	unitCell* curr = activeMap->getCurr();
	
	travHistCopy.pop(); // Remove the cell that the robot is currently in
	
	while((travHistCopy.top() != endCellNum)&&(!travHistCopy.empty())){
		unitCell* t = activeMap->getCell(travHistCopy.top());	// Get the address of the cell that is next on the top of the travHistCopy stack
		//cout << "(naviSyst::moveToFin) Checking if cell " << t->getNum() << " is a dead end" << endl;
		if(t->getNumWalls() >= 3){ // If that cell has more than 3 walls, then it is a dead end and the robot should not travel there on the way back to the cell
			travHistCopy.pop();
			continue;
		}
		else if((!routeToFin.empty())&&(t->getNum() == routeToFin.back())){ // If the cell being considered is the most recent cell added to the routeToFin queue, do not add it again
			travHistCopy.pop();
			continue;
		}
		else{
			routeToFin.push_back(t->getNum());
			travHistCopy.pop();
		}
	}
	
	//Determine quickest path to destination
	int rtf[100]; // Route to fin array for checking the shortest path to the destination
	deque<int> rtfCopy = routeToFin;
	int routeSteps = 0;
	for(int i = 0; rtfCopy.empty() == false; i++){
		rtf[i] = rtfCopy.front();
		rtfCopy.pop_front();
		routeSteps++;
	}
	
	rtfCopy.clear();
	if(routeSteps > 1){
		for(int k = routeSteps; k > 0; k--){
			for(int i = 0; i < routeSteps; i++){
				if(rtf[k] == rtf[i]){
					for(int j = 0; j < i; j++){
						rtfCopy.push_back(rtf[j]);
					}
					break;
				}
			}
		}
	}
	routeToFin = rtfCopy;
	
	routeToFin.push_back(endCellNum); // Push the ending cell onto the routeToFin queue because the previous while loop does not do that
	
	
	
	// cout << "(naviSyst::moveToFin) travHist: ";
	// printTravHist();
	// queue<int> rtf = routeToFin;
	// cout << "(naviSyst::moveToFin) routeToFin front: ";
	// while(!rtf.empty()){
		// cout << rtf.front() << " ";
		// rtf.pop();
	// }
	// cout << endl;
	
	while(!routeToFin.empty()){ // Travel to the cell that must be revisited
		// cout << "(naviSyst::moveToFin) Travelling to cell " << routeToFin.front() << " from cell " << getCurrNum() << endl;
		for(int i = 0; i < 4; i++){
			if((curr->getAdj(i))->getNum() == routeToFin.front()){ // If the adjacent cell to the current cell matches the next cell on the revisitRoute queue, travel to it.
				moveCell(i);
				routeToFin.pop_front();
				curr = curr->getAdj(i); // Update the new current cell
				break;
			}
		}
	}
}

void naviSyst::travCritPath(){

	queue<int> temp;
	stack<int> path;
	unitCell* curr = activeMap->getCurr();
	
	while(!critPath.empty()){ //Create path stack and temp queue to preserve stack
		temp.push(critPath.front());
		path.push(critPath.front());
		critPath.pop();
	}
	
	int nextCell;
	while(!path.empty()){
		if(curr->getNum() == path.top()){ // If the next cell to travel to is already the current cell, ignore it and get the next cell in the critical path
			path.pop();
		}
		else{
			for(int i = 0; i < 4; i++){
				if(curr->getAdj(i) == NULL){ // If the adjacent cell is non-existent, ignore it
					continue;
				}
				else if((curr->getAdj(i))->getNum() == path.top()){
					moveCell(i);
					cout << "(travCritPath) Current cell: " << getCurrNum() << endl;
					path.pop();
					curr = curr->getAdj(i);
					break;
				}
				
			}
		}
	}
}

int naviSyst::forward(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::back(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::strafeLeft(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::strafeRight(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::faceLeft(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::faceRight(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

int naviSyst::aboutFace(){
	#ifdef ACK
	waitForAck();
	#endif
	return 0;
}

void naviSyst::waitForAck(){
	char response;
	while(1){
		scanf(" %c", &response); //Don't use getchar(), the 'a' gets left in cin and is counted again for some reason so it skips some waitForAck commands
		if(response = 'a'){
			break;
		}
	}
}

bool naviSyst::callUltras(){ //Calls the ultrasonic distance sensors and modifies the wallReadings array to reflect the readings. Left, midddle, and right correspond to indexes 0, 1, and 2
	return true;
}

void naviSyst::callWallSensorsSim(){ //Simulates sensor readings for a given map defined by the boolean array "maze"

	bool maze[] = {true,true,false,true,true,true,false,true,true,false,false,true,true,false,true,false,true,true,true,false,true,false,false,true,true,true,false,false,false,false,true,true,false,false,true,false,false,true,false,false,true,true,false,true,true,false,false,true,false,true,false,false,false,true,true,true,true,true,false,true,true,false,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,false,true,true,true,true,false,false,false,true,false,true,false,true,false,true,false,true,false,true,true,false,false,true,true,false,false,false,true,false,true,false,false,true,false,false,false,false,true,true,false,true,false,false,false,true,false,true,false,true,false,true,false,true,false,true,true,true,false,true,false,true,false,true,true,true,false,true,false,true,false,true,false,false,true,true,false,true,true,false,false,true,false,true,false,true,false,true,false,true,false,true,false,false,true,true,false,false,true,false,true,false,true,false,true,true,true,false,false,false,true,true,false,true,true,false,false,true,true,true};

	unitCell* c = activeMap->getCurr();
	int n = c->getNum();
	c->setWall('N',maze[((n-1)*4) + 0]);	
	c->setWall('E',maze[((n-1)*4) + 1]);
	c->setWall('S',maze[((n-1)*4) + 2]);
	c->setWall('W',maze[((n-1)*4) + 3]);

	//Make sure that adjacent cells have matching wall readings
	unitCell* northAdj = activeMap->getCell('N');
	unitCell* southAdj = activeMap->getCell('S');
	unitCell* eastAdj = activeMap->getCell('E');
	unitCell* westAdj = activeMap->getCell('W');
	if(northAdj != NULL){
		northAdj->setWall('S', c->getWall('N'));
	}
	if(southAdj != NULL){
		southAdj->setWall('N', c->getWall('S'));
	}
	if(eastAdj != NULL){
		eastAdj->setWall('W', c->getWall('E'));
	}
	if(westAdj != NULL){
		westAdj->setWall('E', c->getWall('W'));
	}
	
	c->setWallScan(true); //This line should not be included in the actual callWallSensors function, it should be executed by scanWalls instead.
}

int naviSyst::firstCellProtocol(){
	unitCell* c = activeMap->getCurr();
	callWallSensorsSim();
	c->setWall('N', wallReadings[1]);
	c->setWall('E', wallReadings[2]);
	c->setWall('W', wallReadings[0]);	
	c->setWall('S', true);
	cellsVisited++; // Count the initial cell as a unique cell
}

void naviSyst::scanWalls(){
	callUltras(); 
	unitCell* c = activeMap->getCurr();
	//Update the current cell's wall array using the wall sensor readings relative to the current direction the robot is facing. 
	//This is function assumes that the robot has entered the cell from the direction of the robot that does not have any distance sensors, i.e., the robot's "south" side.
	if(currFace == 'N'){ //If the robot is facing north:
		c->setWall('N', wallReadings[1]); // Map front sensor reading to the north wall of the current cell.
		c->setWall('E', wallReadings[2]); // Map right sensor reading to the east wall of the current cell.
		c->setWall('W', wallReadings[0]); // Map left sensor reading to the west wall of the current cell.
		c->setWall('S', false);
	}
	else if(currFace == 'E'){
		c->setWall('E', wallReadings[1]);
		c->setWall('S', wallReadings[2]);
		c->setWall('N', wallReadings[0]);
		c->setWall('W', false);		
	}
	else if(currFace == 'S'){
		c->setWall('S', wallReadings[1]);
		c->setWall('W', wallReadings[2]);
		c->setWall('E', wallReadings[0]);
		c->setWall('N', false);
	}
	else if(currFace == 'W'){
		c->setWall('W', wallReadings[1]);
		c->setWall('N', wallReadings[2]);
		c->setWall('S', wallReadings[0]);
		c->setWall('E', false);
	}
	else{
		cout << "(naviSyst) Current face of the robot is undefined" << endl;
	}
	c->setWallScan(true);
}

void naviSyst::checkFreePaths(){
	unitCell* c = activeMap->getCurr();

	int totalFreePaths = 0;
	// The order that the walls are checked matter: I want to keep the E>N>W>S priority of choosing the next cell to visit. Also want to ensure that an adjacent cell exists
	if((c->getAdj('S') != NULL)&&(c->getWall('S') ==  false)) {
		//cout << "(cFR) S" << endl;
		if((c->getAdj('S'))->getWallScan() == false){ //I don't want the robot to revisit a cell it has already processed unless it's currently backtracking to an unvisited cell.
			c->pushUnexplPaths('S');
			totalFreePaths++;
		}
	}
	if((c->getAdj('W') != NULL)&&(c->getWall('W') ==  false)) {
		//cout << "(cFR) W" << endl;
		if((c->getAdj('W'))->getWallScan() == false){ 
			c->pushUnexplPaths('W');
			totalFreePaths++;
		}
	}
	
	if((c->getAdj('N') != NULL)&&(c->getWall('N') ==  false)) {
		//cout << "(cFR) N" << endl;
		if((c->getAdj('N'))->getWallScan() == false){ 
			c->pushUnexplPaths('N');
			totalFreePaths++;
		}
	}
	
	if((c->getAdj('E') != NULL)&&(c->getWall('E') ==  false)) {
		//cout << "(cFR) E" << endl;
		if((c->getAdj('E'))->getWallScan() == false){ 
			c->pushUnexplPaths('E');
			totalFreePaths++;
		}
	}
	if(totalFreePaths > 1){ // If there is more than one free path from the current cell, push the cell onto the revisit stack because the robot must revisit the cell later
		pushRevisit(c->getNum());
	}
	// cout << "(naviSyst::checkFreePaths) Total free paths from cell " << getCurrNum() << ": " << totalFreePaths << endl;
	// cout << "(cFR) ";
	// printRevisit();
	c->setPathScan(true);
}

char naviSyst::getNextPath(){
	unitCell* c = activeMap->getCurr();
	char nextPath = 'X';
	if(!(c->isUnexplPathsEmpty())){ // If there are unexplored paths, take the next unexplored path.
		nextPath = c->popUnexplPaths();
	}
	return nextPath;
}


void naviSyst::logMapInfo(ofstream & f){
	#ifdef LOG
	
	string NSwall = "--";
	string EWwall = "|";
	string NSfree = "  ";	
	string EWfree = " ";	
	string NSunkn = "??";
	string EWunkn = "?";
	queue<string> topRow, leftRow, rightRow, botRow;
	
	int l = 7; // max length of map 
	int aml = activeMap->getLength(); // length of active map

	f << "Current cell: " << getCurrNum() << " / Current face:" << getCurrFace() << endl;
	
	for(int i = 0; i <= l-1; i++){ // Row
		while(!topRow.empty()){
			topRow.pop();
		}
		while(!leftRow.empty()){
			leftRow.pop();
		}
		while(!rightRow.empty()){
			rightRow.pop();
		}
		while(!botRow.empty()){
			botRow.pop();
		}
		for(int j = 1; j <= l; j++){ // Column		
			unitCell* c = activeMap->getCell((i*l)+ j);
			if(c->getWallScan() == false){ //Cell has not been scanned for walls
				topRow.push(NSunkn);
				leftRow.push(EWunkn);
				rightRow.push(EWunkn);
				botRow.push(NSunkn);
			}
			else{
				if(c->getWall('N') == false){
					topRow.push(NSfree);	
				}
				else {
					topRow.push(NSwall);
				}
				if(c->getWall('E') == false){
					rightRow.push(EWfree);	
				}
				else {
					rightRow.push(EWwall);
				}
				if(c->getWall('S') == false){
					botRow.push(NSfree);	
				}
				else {
					botRow.push(NSwall);
				}
				if(c->getWall('W') == false){
					leftRow.push(EWfree);	
				}
				else {
					leftRow.push(EWwall);
				}
			}
		}
		f << " ";
		//Begin printing out column data for the row
		for(int a = 1; a <= l; a++){
			f << topRow.front() << "   ";
			topRow.pop();
		}
		f << endl;
		for(int b = 1; b <= l; b++){
			if((i*l)+ b  < 10){
				f << leftRow.front() << " " << (i*l)+ b << rightRow.front() << " ";
				leftRow.pop();
				rightRow.pop();
			}
			else{
				f << leftRow.front() << (i*l)+ b << rightRow.front() << " ";
				leftRow.pop();
				rightRow.pop();
			}
		}
		f << endl;
		f << " ";
		for(int c = 1; c <= l; c++){
			f << botRow.front() << "   ";
			botRow.pop();
		}
		f << endl;
	}
	f << endl;
	#endif
}

void naviSyst::logTravHist(ofstream & f){
	#ifdef LOG
	stack<int> temp = travHist;
	stack<int> tempFlip;
	f << "top of stack -> ";
	while(!temp.empty()){ //Flip the travel stack so the first cell printed is the starting cell
		tempFlip.push(temp.top());
		temp.pop();
	}
	
	f << "Travel history: " << endl;	
	while(!tempFlip.empty()){

		f << tempFlip.top() << " -> ";
		tempFlip.pop();
	}
	f << " bottom of stack" << endl;
	#endif
}

void naviSyst::logCritPath(ofstream & f){ // Use after the critical path has been found
	#ifdef LOG
	queue<int> temp = critPath;
	
	f << endl;
	f << "Critical path: " << endl;	
	f << "front of queue -> ";
	while(!temp.empty()){

		f << temp.front() << " -> ";
		temp.pop();
	}
	f << " end of queue";
	#endif
}