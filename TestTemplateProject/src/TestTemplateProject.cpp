//============================================================================
// Name        : TestTemplateProject.cpp
// Author      : roberthilton
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "TestTemplate.h"
#include "TestClass.h"
using namespace std;

int main() {
	CKAutoBuilder<TalonEx> ckAuto = new CKAutoBuilder<TalonEx>(new TalonEx());
	return 0;
}
