#include <time.h>
#include <queue>
#include <stack>
#include <deque>
#include <list>
#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "serialComm2.h"

using namespace std;

/*
March 26, 2015
-Started camera system class to coordinate the three cameras
*/

class camSyst{
private:
	int serialHandle; 
	int uniquePathOrder[70]; // External program must keep track of the order of unique cells visited
	int pathIterator;
	stack<char> eggs; // Will hold the eggs that this raspberry pi's camera sees. A '.' pushed onto the stack means that there was either no wall to take a picture of or no character recognized.
	char finalEggs[50][3]; // Will hold the easter egg values found at the cell location denoted by the first index after the robot has traveled through the maze
	char temp[2]; //Temporary args parameter used in sendMessage()
	char PIC[3], DNE[3]; //Command parameter for sendMessage()
	
public:
	camSyst();
	void addUniquePath(int cell); 
	int takePics(char currFace, bool NWall, bool EWall, bool SWall, bool WWall); // Function using serial IO to command the auxilliary Pis to take pictures based on which sides have walls
	int consolEggs(int mapSize); // Function to consolidate each pi's easter egg data to write to the flash drive. Needs to be passed the active map's size
	char rpiCam(); // Place holder function for the main pi to take a  picture, returns the character read
};

camSyst::camSyst(){
	serialHandle = serialOpen("/dev/ttyAMA0", 9600); // String argument found in dev folder. Opens serial comm using the only TX and RX pins on the board.
	pathIterator = 0;
	temp[0] = '~'; temp[1] = 0;
	PIC[0] = 'P'; PIC[1] = 'I'; PIC[2] = 'C';
	DNE[0] = 'D'; DNE[1] = 'N'; DNE[2] = 'E';

}

void camSyst::addUniquePath(int cell){
	uniquePathOrder[pathIterator] = cell;
	pathIterator++;
}

int camSyst::takePics(char currFace, bool NWall, bool EWall, bool SWall, bool WWall){
	// sendMessage(serialHandle, 'B', "PIC", '~'); // Tell the other two pis to take a picture
	// waitAck(serialHandle, "L"); //Wait for pi 1 to finish analyzing pic
	// waitAck(serialHandle, "R"); // Wait for pi 2 to finish analyzing pic 
	// rpiCam(); //Main pi takes pi
	char eggRead;
	if (currFace == 'N'){
		cout << "(camSyst.takePics) North" << endl;
		if(NWall == true){
			cout << "(camSyst.takePics) North true" << endl;
			eggRead = rpiCam();
			eggs.push(eggRead);
		}
		else{
			eggs.push(' ');
		}
		if(EWall == true){
			cout << "(camSyst.takePics) East true" << endl;
			sendMessage(serialHandle, 'L', PIC, temp);
		}
		else{
			sendMessage(serialHandle, 'L', DNE, temp);
		}
		if(WWall == true){
			cout << "(camSyst.takePics) West true" << endl;
			sendMessage(serialHandle, 'R', PIC, temp);		
		}		
		else{
			sendMessage(serialHandle, 'R', DNE, temp);
		}
	}
	else if (currFace == 'E'){
		cout << "(camSyst.takePics) East" << endl;
		if(NWall == true){
			sendMessage(serialHandle, 'R', PIC, temp);		
		}
		else{
			sendMessage(serialHandle, 'R', DNE, temp);
		}
		if(EWall == true){
			eggRead = rpiCam();
			eggs.push(eggRead);
		}
		else{
			eggs.push(' ');
		}
		if(SWall == true){
			sendMessage(serialHandle, 'L', PIC, temp);	
		}
		else{
			sendMessage(serialHandle, 'L', DNE, temp);
		}
	}
	else if (currFace == 'S'){
		cout << "(camSyst.takePics) South" << endl;
		if(EWall == true){
			sendMessage(serialHandle, 'L', PIC, temp);	
		}
		else{
			sendMessage(serialHandle, 'L', DNE, temp);
		}
		if(SWall == true){
			eggRead = rpiCam();
			eggs.push(eggRead);
		}
		else{
			eggs.push(' ');
		}
		if(WWall == true){
			sendMessage(serialHandle, 'R', PIC, temp);
		}		
		else{
			sendMessage(serialHandle, 'R', DNE, temp);
		}
	}
	else if (currFace == 'W'){
		cout << "(camSyst.takePics) West" << endl;
		if(NWall == true){
			sendMessage(serialHandle, 'R', PIC, temp);			
		}
		else{
			sendMessage(serialHandle, 'R', DNE, temp);
		}
		if(SWall == true){
			sendMessage(serialHandle, 'L', PIC, temp);
		}
		else{
			sendMessage(serialHandle, 'L', DNE, temp);
		}
		if(WWall == true){
		    eggRead = rpiCam();
			eggs.push(eggRead);
		}		
		else{
			eggs.push(' ');
		}
	}
	else{
		cout << "(cs.takePics) Invalid face passed" << endl;
	}
	return 0;
}

int camSyst::consolEggs(int mapSize){
	char* leftEggs; // Stores the left pi's easter eggs
	char* rightEggs; // Stores the right pi's easter eggs
	char* leftParse;
	char* rightParse;
	
	getMessage(serialHandle, 'L', leftEggs); 
	getMessage(serialHandle, 'R', rightEggs);
	leftParse = strtok(leftEggs, ",");
	for(int i = 0; i < mapSize; i++){
		finalEggs[uniquePathOrder[i]][0] = *leftParse;
		finalEggs[uniquePathOrder[i]][1] = eggs.top();
		leftParse = strtok(NULL, ",");
		eggs.pop();
	}
	rightParse = strtok(rightEggs, ",");
	for(int i = 0; i < mapSize; i++){ // use another loop to parse the right pi's egg queue because strtok can't be used alternatingly between two strings with NULL as the first argument
		finalEggs[uniquePathOrder[i]][2] = *rightParse;
		rightParse = strtok(NULL, ",");		
	}
}

char camSyst::rpiCam(){
	//placeholder function
	return 'A';
}