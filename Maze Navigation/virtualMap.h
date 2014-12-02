#include <stdio.h>
#include <cstdlib>
#include <string>
#include <stack>
#include "unitCell.h" 

using namespace std;

class virtualMap{

private:
	unitCell* start;
	unitCell* fin;
	unitCell* curr;
	int currNum;
	unitCell c1, c2, c3, c4, c5, c6, c7, //Declare each cell
			 c8, c9, c10, c11, c12, c13, c14,
			 c15, c16, c17, c18, c19, c20, c21,
			 c22, c23, c24, c25, c26, c27, c28, 
			 c29, c30, c31, c32, c33, c34, c35,
			 c36, c37, c38, c39, c40, c41, c42,
			 c43, c44, c45, c46, c47, c48, c49;
	
public:

	virtualMap(); // Default map assumes the map size is 5x5
	virtualMap(string mapSize); //Given a map size, set the starting location and ending locations of the robot. [INCOMPLETE]
	
	unitCell* getStart();
	unitCell* getFin();
	unitCell* getCurr();
	unitCell* getCell(int n); // Given a number, return the address of the unitCell that has that number
	int getCurrNum();
	// bool getCurrCellScan();
	// bool getCurrWall();
	// bool getCurrEgg();
	// bool getCurrSpBump();
	// bool getCurrComplete();
	// int peekCurrUnexplPaths();
	// int popCurrUnexplPaths();

	void setStart(unitCell* ns);
	void setFin(unitCell* nf);
	void setCurr(unitCell* nc); //Set new current cell and update currNum as well
	void move(int crdnlDir); //Move a certain direction
	void move(char crdnlDir);
	void initMap(); // Create the 7x7 map using the default unitCell
	
};

virtualMap::virtualMap(){
	initMap();
	start = &c48;
	curr = &c48;
	fin = &c9;
}

virtualMap::virtualMap(string mapSize){ 
	if(strcmp(&mapSize[0], "6x6") == 0){
		cout << "(virtualMap::virtualMap) Created 6x6" << endl;
		initMap();
		start = &c48;
		curr = &c48;
		fin = &c1;
	}
	else if (strcmp(&mapSize[0], "7x7") == 0){
		cout << "(virtualMap::virtualMap) Created 7x7" << endl;
		initMap();
		start = &c49;
		curr = &c49;
		fin = &c1;
	}
	else{
		cout << "(virtualMap::virtualMap) Created 5x5" << endl;
		initMap();
		start = &c48;
		curr = &c48;
		fin = &c9;
	}
}

unitCell* virtualMap::getStart(){
	return start;
}

unitCell* virtualMap::getFin(){
	return fin;
}

unitCell* virtualMap::getCurr(){
	return curr;
}

unitCell* virtualMap::getCell(int n){	
	if( n == 1){
		return &c1;
	}
	if( n == 2){
		return &c2;
	}
	if( n == 3){
		return &c3;
	}
	if( n == 4){
		return &c4;
	}
	if( n == 5){
		return &c5;
	}
	if( n == 6){
		return &c6;
	}
	if( n == 7){
		return &c7;
	}
	if( n == 8){
		return &c8;
	}
	if( n == 9){
		return &c9;
	}
	if( n == 10){
		return &c10;
	}
	if( n == 11){
		return &c11;
	}
	if( n == 12){
		return &c12;
	}
	if( n == 13){
		return &c13;
	}
	if( n == 14){
		return &c14;
	}
	if( n == 15){
		return &c15;
	}
	if( n == 16){
		return &c16;
	}
	if( n == 17){
		return &c17;
	}
	if( n == 18){
		return &c18;
	}
	if( n == 19){
		return &c19;
	}
	if( n == 20){
		return &c20;
	}
	if( n == 21){
		return &c21;
	}
	if( n == 22){
		return &c22;
	}
	if( n == 23){
		return &c23;
	}
	if( n == 24){
		return &c24;
	}
	if( n == 25){
		return &c25;
	}
	if( n == 26){
		return &c26;
	}
	if( n == 27){
		return &c27;
	}
	if( n == 28){
		return &c28;
	}
	if( n == 29){
		return &c29;
	}
	if( n == 30){
		return &c30;
	}
	if( n == 31){
		return &c31;
	}
	if( n == 32){
		return &c32;
	}
	if( n == 33){
		return &c33;
	}
	if( n == 34){
		return &c34;
	}
	if( n == 35){
		return &c35;
	}
	if( n == 36){
		return &c36;
	}
	if( n == 37){
		return &c37;
	}
	if( n == 38){
		return &c38;
	}
	if( n == 39){
		return &c39;
	}
	if( n == 40){
		return &c40;
	}
	if( n == 41){
		return &c41;
	}
	if( n == 42){
		return &c42;
	}
	if( n == 43){
		return &c43;
	}
	if( n == 44){
		return &c44;
	}
	if( n == 45){
		return &c45;
	}
	if( n == 46){
		return &c46;
	}
	if( n == 47){
		return &c47;
	}
	if( n == 48){
		return &c48;
	}
	if( n == 49){
		return &c49;
	}
	else{
		cout << "(virtualMap::getCell) Invalid request" << endl;
		return NULL;
	}

}

int virtualMap::getCurrNum(){
	return curr->getNum();
}

void virtualMap::setStart(unitCell* ns){
	start = ns;
}

void virtualMap::setFin(unitCell* nf){
	fin = nf;
}

void virtualMap::setCurr(unitCell* nc){
	curr = nc;
	currNum = curr->getNum();
}

void virtualMap::move(int crdnlDir){
	unitCell* prev = curr; 
	curr = curr->getAdj(crdnlDir);
	if (curr == NULL){
		cout << "(virtualMap) No cell exists in that direction" << endl;
		curr = prev;
	}
	currNum = curr->getNum();
}

void virtualMap::move(char crdnlDir){
	unitCell* prev = curr;
	curr = curr->getAdj(crdnlDir);
	if (curr == NULL){
		cout << "(virtualMap) No cell exists in that direction" << endl;
		curr = prev;
	}
	currNum = curr->getNum();
}

void virtualMap::initMap(){

	c1 = unitCell(1, NULL, &c2, &c8, NULL);
	c2 = unitCell(2, NULL, &c3, &c9, &c1);
	c3 = unitCell(3, NULL, &c4, &c10, &c2);
	c4 = unitCell(4, NULL, &c5, &c11, &c3);
	c5 = unitCell(5, NULL, &c6, &c12, &c4);
	c6 = unitCell(6, NULL, &c7, &c13, &c5);
	c7 = unitCell(7, NULL, NULL, &c14, &c6);
	c8 = unitCell(8, &c1, &c9, &c15, NULL);
	c9 = unitCell(9, &c2, &c10, &c16, &c8);
	c10 = unitCell(10, &c3, &c11, &c17, &c9);
	c11 = unitCell(11, &c4, &c12, &c18, &c10);
	c12 = unitCell(12, &c5, &c13, &c19, &c11);
	c13 = unitCell(13, &c6, &c14, &c20, &c12);
	c14 = unitCell(14, &c7, NULL, &c21, &c13);
	c15 = unitCell(15, &c8, &c16, &c22, NULL);
	c16 = unitCell(16, &c9, &c17, &c23, &c15);
	c17 = unitCell(17, &c10, &c18, &c24, &c16);
	c18 = unitCell(18, &c11, &c19, &c25, &c17);
	c19 = unitCell(19, &c12, &c20, &c26, &c18);
	c20 = unitCell(20, &c13, &c21, &c27, &c19);
	c21 = unitCell(21, &c14, NULL, &c28, &c20);
	c22 = unitCell(22, &c15, &c23, &c29, NULL);
	c23 = unitCell(23, &c16, &c24, &c30, &c22);
	c24 = unitCell(24, &c17, &c25, &c31, &c23);
	c25 = unitCell(25, &c18, &c26, &c32, &c24);
	c26 = unitCell(26, &c19, &c27, &c33, &c25);
	c27 = unitCell(27, &c20, &c28, &c34, &c26);
	c28 = unitCell(28, &c21, NULL, &c35, &c27);
	c29 = unitCell(29, &c22, &c30, &c36, NULL);
	c30 = unitCell(30, &c23, &c31, &c37, &c29);
	c31 = unitCell(31, &c24, &c32, &c38, &c30);
	c32 = unitCell(32, &c25, &c33, &c39, &c31);
	c33 = unitCell(33, &c26, &c34, &c40, &c32);
	c34 = unitCell(34, &c27, &c35, &c41, &c33);
	c35 = unitCell(35, &c28, NULL, &c42, &c34);
	c36 = unitCell(36, &c29, &c37, &c43, NULL);
	c37 = unitCell(37, &c30, &c38, &c44, &c36);
	c38 = unitCell(38, &c31, &c39, &c45, &c37);
	c39 = unitCell(39, &c32, &c40, &c46, &c38);
	c40 = unitCell(40, &c33, &c41, &c47, &c39);
	c41 = unitCell(41, &c34, &c42, &c48, &c40);
	c42 = unitCell(42, &c35, NULL, &c49, &c41);
	c43 = unitCell(43, &c36, &c44, NULL, NULL);
	c44 = unitCell(44, &c37, &c45, NULL, &c43);
	c45 = unitCell(45, &c38, &c46, NULL, &c44);
	c46 = unitCell(46, &c39, &c47, NULL, &c45);
	c47 = unitCell(47, &c40, &c48, NULL, &c46);
	c48 = unitCell(48, &c41, &c49, NULL, &c47);
	c49 = unitCell(49, &c42, NULL, NULL, &c48);

}