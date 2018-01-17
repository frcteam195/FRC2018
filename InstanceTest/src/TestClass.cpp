/*
 * TestClass.cpp
 *
 *  Created on: Jan 17, 2018
 *      Author: chris
 */

#include "TestClass.h"

TestClass* TestClass::instance = nullptr;

TestClass::TestClass() {;}

TestClass* TestClass::GetInstance() {
	if(instance == nullptr)
		instance = new TestClass();
	return instance;
}

void TestClass::print() {
	cout << &instance << endl;
}
