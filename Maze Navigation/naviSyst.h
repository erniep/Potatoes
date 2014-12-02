#include "virtualMap.h"
#include <queue>
#include <stack>

class naviSyst{

private:
	
	//[Internal variables]
	virtualMap map5x5, map6x6, map7x7;
	virtualMap* activeMap; // Points at the map that the robot is using, since the map size changes for each round of the competition
	int totalCells; // Total number of cells to visit for the current active map
	int cellsVisited; // Number of unique cells currently visited
	char currFace; // Current cardinal direction of front face
	stack<int> critPath; // Cell# order of critical path
	stack<int> travHist; // Travel history of robot. Used to retrace steps
	stack<int> revisit; // Stack of cells that need to be revisited because there are still paths to explore there
	queue<int> eggLocs; // Cell numbers of cells that contain easter eggs

	//[Sensor variables]
	bool wallReadings[4]; // Values returned by the wall sensors for the current cell. [0], [1], and [2] correspond to left, front, and right sensors. There is a fourth index in case a fourth sensor is added to the south of the robot. Since the wall sensors do not return values relative to the NESW convention used in this code, this function must use the robot's current face to translate these sensor readings into the NESW convention.
	
public: 

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
	void pushCritPath(int cellNum);
	void pushTravHist(int cellNum);
	void pushRevisit(int cellNum);
	void pushEggLocs(int eggLoc);
	void changeMapSize(); // Gets the map matching the next round of the competition. Also changes the totalCells variable and resets the cellsVisited variable.
	
	//[Movement functions]
	int traverseMap(); // This is the main function that actually navigates throughout the map.
	int traverseMapSim(); // This function virtually solves a maze, and does not use the scanWalls function.
	int moveCell(int nextDir);
	int moveCell(char nextDir); // Travel to an adjacent cell indicated by the cardinal direction passed as an argument. Checks against current face of robot, then updates the new face when the robot reaches the new cell. This movement is basic and does not use strafing.
	void moveCellFast(char nextDir); // Move to an adjacent cell using strafing.
	void revisitCell(); // Travel to a cell that must be revisited. Intended to be called after the getNextPath function returns 'X'
	void moveToFin(); // Travel to the end of the maze after all the cells have been visited.
	int forward(); // Move forward until the tivaC returns a flag saying that the robot is in a new cell.
	int back();
	int strafeLeft();
	int strafeRight();
	int faceLeft();
	int faceRight();
	int aboutFace();
	
	//[Unit cell check functions]
	void callWallSensors(); //Function to call the wall sensors. Updates the wallReadings array. This function assumes that the direction not covered by the wall sensors has no wall.
	void callWallSensorsSim(); //Dummy function to call the wall sensors. Directly updates the current cell's walls
	void callRPiCam(); // Dummy function to call the raspberry pi camera.
	int firstCellProtocol(); // Information retrieval function for the first cell, which has a few differences from other cells in the maze. This function must be called first to initialize the maze navigation algorithm. Assuming the robot is pointed north at the initial cell, this function ensures that the wall without a distance sensor is recorded as a wall in the virtual map. The wallScan() function assumes that the wall without a distance sensor is an open path, since it's assumed the robot entered the cell from there.
	void scanWalls(); // Checks for walls in the current cell and update the virtual map to reflect the new information.
	void checkFreePaths(); // Intended to be called after the scanWalls function has been used to detect and record walls in the current cell. This function will push onto the current cell's unexplPaths stack the free paths that the robot can travel to.
	char getNextPath(); // Intended to be called after the checkFreePaths function. Determines the next direction to travel to, based on the unitCell's unexploredPaths stack. Returns a capital 'X' if there is no more unexplored paths to travel.
	
};

naviSyst::naviSyst(){ 
	leftRotates = 0;
	rightRotates = 0;
	aboutFaceRotates = 0;
	map6x6 = virtualMap("6x6");
	map7x7 = virtualMap("7x7");
	activeMap = &map5x5;
	totalCells = 26; // Initially assume the first round. Total = 5x5 cells from map + starting cell
	cellsVisited = 0; // Number of unique cells visited
	currFace = 'N'; // Default face of the robot is north
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

int naviSyst::peekTravHist(){
	int p = travHist.top();
	return p;
}

int naviSyst::popTravHist(){
	int t = travHist.top();
	travHist.pop();
	return t;
}

int naviSyst::printTravHist(){
	stack<int> temp = travHist;
	cout << "(printTravHist) Stack top: " << endl;
	while(!temp.empty()){
		cout << temp.top() << " ";
		temp.pop();
	}
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

void naviSyst::pushCritPath(int cellNum){
	critPath.push(cellNum);
}

void naviSyst::pushTravHist(int cellNum){
	travHist.push(cellNum);
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
		//Hi Ernie
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
	callWallSensorsSim(); // Must remove this later, this is only for debugging
	return 0;
}

void naviSyst::revisitCell(){
	queue<int> revisitRoute; // Queue used to travel to the cell that must be revisited
	stack<int> temp; // Stack used to restore the travelHist stack
	unitCell* curr = activeMap->getCurr();
	int dest = popRevisit();

	temp.push(popTravHist()); // Top of the travHist stack is the current node, so we don't want to include it in the revisitRoute queue
	// cout << "(naviSyst::revisitCell) Travel history: " << endl; 
	// printTravHist();
	int travHistTop = peekTravHist();


	while((!travHist.empty())&&(travHistTop != dest)){ // Push the cells that lead to the destination onto the revisitRoute stack
		revisitRoute.push(peekTravHist());
		temp.push(popTravHist());
		travHistTop = peekTravHist();
	}
	revisitRoute.push(peekTravHist()); // Last value to enqueue onto the revisitRoute queue should be the destination
	temp.push(popTravHist());
	
	while(!temp.empty()){ //Restore the travel history stack
		pushTravHist(temp.top());
		temp.pop();
	}
	
	// queue<int> rr = revisitRoute;
	// cout << "(naviSyst::revisitCell) revisitRoute front: ";
	// while(!rr.empty()){
		// cout << rr.front() << " ";
		// rr.pop();
	// }
	// cout << endl;
	
	while(!revisitRoute.empty()){ //Travel to the cell that must be revisited
		// cout << "(naviSyst::revisitCell) Travelling to cell " << revisitRoute.front() << " from cell " << getCurrNum() << endl;
		for(int i = 0; i < 4; i++){
			if((curr->getAdj(i))->getNum() == revisitRoute.front()){ // If the adjacent cell to the current cell matches the next cell on the revisitRoute queue, travel to it.
				moveCell(i);
				revisitRoute.pop();
				curr = curr->getAdj(i); // Update the new current cell
				break;
			}
		}
	}
}

void naviSyst::moveToFin(){
	
	int endCellNum = activeMap->getFin()->getNum(); //Get the cell number of the ending cell
	stack<int> travHistCopy = travHist;
	queue<int> routeToFin; // Will contain the route to the ending cell from whatever cell the robot is currently in
	unitCell* currCopy = getCurrCell(); // unitCell pointer used to plan out the direct route to the ending cell
	unitCell* curr = activeMap->getCurr();
	
	travHistCopy.pop(); // Remove the cell that the robot is currently in
	
	while((travHistCopy.top() != endCellNum)&&(!travHistCopy.empty())){
		unitCell* t = activeMap->getCell(travHistCopy.top());	// Get the address of the cell that is next on the top of the travHistCopy stack
		//cout << "(naviSyst::moveToFin) Checking if cell " << t->getNum() << " is a dead end" << endl;
		if(t->getNumWalls() >= 3){ // If that cell has more than 3 walls, then it is a dead end at the robot should not travel there on the way back to the ending cell
			travHistCopy.pop();
			continue;
		}
		else if((!routeToFin.empty())&&(t->getNum() == routeToFin.back())){ // If the cell being considered is the most recent cell added to the routeToFin queue, do not add it again
			travHistCopy.pop();
			continue;
		}
		else{
			routeToFin.push(t->getNum());
			travHistCopy.pop();
		}
	}
	routeToFin.push(endCellNum); // Push the ending cell onto the routeToFin queue because the previous while loop does not do that
	
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
				routeToFin.pop();
				curr = curr->getAdj(i); // Update the new current cell
				break;
			}
		}
	}
	
}

int naviSyst::forward(){
	return 0;
}

int naviSyst::back(){
	return 0;
}

int naviSyst::strafeLeft(){
	return 0;
}

int naviSyst::strafeRight(){
	return 0;
}

int naviSyst::faceLeft(){
	return 0;
}

int naviSyst::faceRight(){
	return 0;
}

int naviSyst::aboutFace(){
	return 0;
}


int naviSyst::traverseMap(){	
	return 0;
}

int naviSyst::traverseMapSim(){
	// char nextPath;
	// callWallSensorsSim(); // In the actual traverseMap function, this would be a call to the firstCellProtocol function.
	// checkFreePaths();
	// nextPath = getNextPath();
	// if(nextPath == 'X'){
		// revisitCell();
	// }
	// else{
		// moveCell(nextPath);
	// }
	// return 0;
}

void naviSyst::callWallSensors(){ 
}

void naviSyst::callWallSensorsSim(){ //Dummy function that returns wall values according to the sample 5x5 maze given by the IEEE competition
	unitCell* c = activeMap->getCurr();
	int n = c->getNum(); 
	if(n == 48){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 41){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 40){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 39){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);	
	}
	else if(n == 38){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 37){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 34){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 33){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 32){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 31){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 30){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 27){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 26){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 25){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 24){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 23){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 20){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 19){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',true);
		c->setWall('W',false);
	}
	else if(n == 18){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 17){
		c->setWall('N',false);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 16){
		c->setWall('N',false);
		c->setWall('E',false);
		c->setWall('S',true);
		c->setWall('W',true);
	}
	else if(n == 13){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',false);
	}
	else if(n == 12){
		c->setWall('N',true);
		c->setWall('E',false);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 11){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 10){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
	}
	else if(n == 9){
		c->setWall('N',true);
		c->setWall('E',true);
		c->setWall('S',false);
		c->setWall('W',true);
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
	callWallSensors(); 
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
	// The order that the walls are checked matter: I want to keep the E>N>W>S priority of choosing the next cell to visit
	if(c->getWall('S') == false){
		if((c->getAdj('S'))->getWallScan() == false){ //I don't want the robot to revisit a cell it has already processed unless it's currently backtracking to an unvisited cell.
			c->pushUnexplPaths('S');
			totalFreePaths++;
		}
	}
	
	if(c->getWall('W') == false){
		if((c->getAdj('W'))->getWallScan() == false){ 
			c->pushUnexplPaths('W');
			totalFreePaths++;
		}
	}
	
	if(c->getWall('N') == false){
		if((c->getAdj('N'))->getWallScan() == false){ 
			c->pushUnexplPaths('N');
			totalFreePaths++;
		}
	}
	
	if(c->getWall('E') == false){
		if((c->getAdj('E'))->getWallScan() == false){ 
			c->pushUnexplPaths('E');
			totalFreePaths++;
		}
	}
	if(totalFreePaths > 1){ // If there is more than one free path from the current cell, push the cell onto the revisit stack because the robot must revisit the cell later
		pushRevisit(c->getNum());
	}
	// cout << "(naviSyst::checkFreePaths) Total free paths from cell " << getCurrNum() << ": " << totalFreePaths << endl;
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