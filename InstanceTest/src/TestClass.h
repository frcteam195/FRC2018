/*
 * TestClass.h
 *
 *  Created on: Jan 17, 2018
 *      Author: chris
 */

#ifndef TESTCLASS_H_
#define TESTCLASS_H_

#include <iostream>

using namespace std;

class TestClass {
private:
	static TestClass* instance;
public:
	TestClass();

	static TestClass* GetInstance();

	void print();
};

#endif /* TESTCLASS_H_ */
