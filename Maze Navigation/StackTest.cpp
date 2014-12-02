#include <stdio.h>
#include <cstdlib>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <stack>

using namespace std;

int main(){
	stack<int> s;
	s.push(1);
	s.push(2);
	s.push(3);
	int t = s.top();
	cout << t << endl;
	cout << "Hello" << endl;
}