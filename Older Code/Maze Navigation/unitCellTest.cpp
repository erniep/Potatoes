#include <stdio.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>
#include "unitCell.h"

using namespace std;

int main(){

	// unitCell start = unitCell();
	// unitCell* startPtr = &start;
	// int length;
	// int width;

	//Pointers used to create and initialize the length x width array
	// unitCell* currLength = &start;
	// unitCell* currWidth = &start;
	// unitCell* prevLength = &start;
	// unitCell* prevWidth = &start;
	// unitCell* northLink = &start;	
	
	unitCell* ucPtr;
	unitCell c1, c2, c3, c4, c5, c6, c7, //Declare each cell
			 c8, c9, c10, c11, c12, c13, c14,
			 c15, c16, c17, c18, c19, c20, c21,
			 c22, c23, c24, c25, c26, c27, c28, 
			 c29, c30, c31, c32, c33, c34, c35,
			 c36, c37, c38, c39, c40, c41, c42,
			 c43, c44, c45, c46, c47, c48, c49;
	
	//Set the number and adjacent cells of each cell
	//(Cell number, north adjacent, east adjacent, south adjacent, west adjacent)
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
	
	//Move around the maze and print out the cells visited	
	ucPtr = &c1;
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c2
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c3
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c10
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('W'); //c9
	cout << ucPtr->getNum() << endl; 
	ucPtr = ucPtr->getAdj('W'); //c8
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c15
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c16
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c17
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c18
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c19
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c20
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('E'); //c21
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c28
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c35
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c42
	cout << ucPtr->getNum() << endl;
	ucPtr = ucPtr->getAdj('S'); //c49
	cout << ucPtr->getNum() << endl;
}