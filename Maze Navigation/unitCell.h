#include <stdio.h>
#include <cstdlib>
#include <string>
#include <stack>

using namespace std;

class unitCell{

private:
	int num;
	bool wall[4]; // 0123 -> NESW True of there is a wall.
	char egg[4]; // 0123 -> NESW
	bool spBump[4]; 
	bool wallScan; // True if walls have been registered for this cell.
	bool pathScan; // True if unexplored paths have been registered for this cell.
	bool cellScan; // True if walls, easter eggs, speed bumps, and unexplored paths have been registered for this cell
	bool complete; // False if there are still unexplored paths that branch out from this cell
	stack<char> unexplPaths; // Stack of paths that still must be explored from this cell.
	unitCell* adj[4]; // 0123 -> NESW. Need to know relative cardinal directions when traversing through unit cells
						
	
public:

	unitCell();
	unitCell(int n);
	unitCell(int n, unitCell* north, unitCell* east, unitCell* south, unitCell* west);

	int getNum();
	bool getWallScan();
	bool getPathScan();
	bool getCellScan();
	bool getWall(int crdnlDir);
	bool getWall(char crdnlDir); //Overloaded get instruction to make code more readable
	int	getNumWalls(); // Returns the number of walls that surround this unit cell. Used for detecting dead ends.
	char getEgg(int crdnlDir);
	char getEgg(char crdnlDir);
	bool getSpBump(int crdnlDir);
	bool getComplete();
	char peekUnexplPaths(); // Return 'X' if empty
	char popUnexplPaths();	
	bool isUnexplPathsEmpty();
	unitCell* getAdj(int crdnlDir);
	unitCell* getAdj(char crdnlDir);
	int getAdjNum(int crdnlDir);
	
	void setNum(int n);
	void setWallScan(bool check);
	void setPathScan(bool check);
	void setCellScan(bool check);
	void setWall(int crdnlDir, bool w);
	void setWall(char crdnlDir, bool w);
	void setEgg(int crdnlDir, char e);
	void setSpBump(int crdnlDir, bool spB);
	void setComplete(bool f);
	char pushUnexplPaths(char path);
	void setAdj(int crdnlDir, unitCell* n);
};

unitCell::unitCell(){
	num = 100;
	wallScan = false;
	pathScan = false;
	cellScan = false;
	cellScan = false;
	for(int i = 0; i < 4; i++){
		wall[i] = true;
		egg[i] = false;
		spBump[i] = false;
		adj[i] = NULL;
	}
	complete = false;
}

unitCell::unitCell(int n){
	num = n;
	wallScan = false;
	pathScan = false;
	cellScan = false;	
	for(int i = 0; i < 4; i++){
		wall[i] = true;
		egg[i] = false;
		spBump[i] = false;
		adj[i] = NULL;
	}	
	complete = false;
}

unitCell::unitCell(int n, unitCell* north, unitCell* east, unitCell* south, unitCell* west){
	num = n;
	wallScan = false;
	pathScan = false;
	cellScan = false;	
	unitCell* initAdj[4] = {north, east, south, west};
	for(int i = 0; i < 4; i++){
		wall[i] = true;
		egg[i] = false;
		spBump[i] = false;
		adj[i] = initAdj[i];
	}	
	
	complete = false;
}

int unitCell::getNum(){
	return num;
}

bool unitCell::getWallScan(){
	return wallScan;
}

bool unitCell::getPathScan(){
	return pathScan;
}

bool unitCell::getCellScan(){
	return cellScan;
}

bool unitCell::getWall(int crdnlDir){
	return wall[crdnlDir];
}

bool unitCell::getWall(char crdnlDir){
	if(crdnlDir == 'N'){
		return getWall(0);
	}
	else if(crdnlDir == 'E'){
		return getWall(1);
	}
	else if(crdnlDir == 'S'){
		return getWall(2);
	}
	else if(crdnlDir == 'W'){
		return getWall(3);
	}
	else{
		cout << "(unitCell " << num << ")Error retrieving wall" << endl;
		exit(0);
	}
}

int	unitCell::getNumWalls(){
	int numWalls = 0;
	for(int i = 0; i < 4; i++){
		if(wall[i] == true){
			numWalls++;
		}
	}
	//cout << "(unitCell " << num << ") has " << numWalls << " walls "<< endl;
	return numWalls;
}

char unitCell::getEgg(int crdnlDir){
	return egg[crdnlDir];
}

char unitCell::getEgg(char crdnlDir){
	if(crdnlDir == 'N'){
		return getEgg(0);
	}
	else if(crdnlDir == 'E'){
		return getEgg(1);
	}
	else if(crdnlDir == 'S'){
		return getEgg(2);
	}
	else if(crdnlDir == 'W'){
		return getEgg(3);
	}
	else{
		cout << "(unitCell " << num << ")Error retrieving eggs" << endl;
		exit(0);
	}
}

bool unitCell::getSpBump(int crdnlDir){
	return spBump[crdnlDir];
}

bool unitCell::getComplete(){
	return complete;
}

char unitCell::peekUnexplPaths(){
	if(unexplPaths.empty()){
		return 'X';
	}
	return unexplPaths.top();
}

char unitCell::popUnexplPaths(){
	int n = unexplPaths.top();
	unexplPaths.pop();
	return n;
}

bool unitCell::isUnexplPathsEmpty(){
	return unexplPaths.empty();
}

unitCell* unitCell::getAdj(int crdnlDir){
	return adj[crdnlDir];
}

unitCell* unitCell::getAdj(char crdnlDir){
	if(crdnlDir == 'N'){
		return getAdj(0);
	}
	else if(crdnlDir == 'E'){
		return getAdj(1);
	}
	else if(crdnlDir == 'S'){
		return getAdj(2);
	}
	else if(crdnlDir == 'W'){
		return getAdj(3);
	}
	else{
		cout << "(unitCell " << num << ")Error retrieving adjacent cell" << endl;
		exit(0);
	}
}

int unitCell::getAdjNum(int crdnlDir){
	return adj[crdnlDir]->getNum();
}

void unitCell::setNum(int n){
	num = n;
}

void unitCell::setWallScan(bool check){	
	wallScan = check;
}

void unitCell::setPathScan(bool check){
	pathScan = check;
}

void unitCell::setCellScan(bool check){
	cellScan = check;
}

void unitCell::setWall(int crdnlDir, bool w){
	wall[crdnlDir] = w;
}

void unitCell::setWall(char crdnlDir, bool w){
	if(crdnlDir == 'N'){
		setWall(0, w);
	}
	else if(crdnlDir == 'E'){
		setWall(1, w);
	}
	else if(crdnlDir == 'S'){
		setWall(2, w);
	}
	else if(crdnlDir == 'W'){
		setWall(3, w);
	}
	else{
		cout << "(unitCell " << num << ")Error passing arguments to setWall function" << endl;
		exit(0);
	}
}

void unitCell::setEgg(int crdnlDir, char e){
	egg[crdnlDir] = e;
}

void unitCell::setSpBump(int crdnlDir, bool spB){
	spBump[crdnlDir] = spB;
}

void unitCell::setComplete(bool f){
	complete = f;
}

char unitCell::pushUnexplPaths(char path){
	unexplPaths.push(path);
}

void unitCell::setAdj(int crdnlDir, unitCell* n){
	adj[crdnlDir] = n;
}
