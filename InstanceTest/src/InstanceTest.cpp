//============================================================================
// Name        : InstanceTest.cpp
// Author      : Chris Bonomi
//============================================================================

#include <iostream>
#include "TestClass.h"

using namespace std;

int main() {
	TestClass* a = TestClass::GetInstance();
	TestClass* b = TestClass::GetInstance();

	a->print();
	b->print();

	return 0;
}
