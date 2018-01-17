//============================================================================
// Name        : TestInstanceProj.cpp
// Author      : roberthilton
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ThisInstance.h"
using namespace std;

int main() {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	ThisInstance *t = ThisInstance::getInstance();
	cout << t << endl;
	return 0;
}
