/*
 * ThisInstance.h
 *
 *  Created on: Jan 17, 2018
 *      Author: roberthilton
 */

#include <iostream>

#ifndef THISINSTANCE_H_
#define THISINSTANCE_H_

class ThisInstance {
public:
	ThisInstance();
	~ThisInstance() {};

	static ThisInstance *getInstance() {
		if(!instance)
			instance = new ThisInstance();

		return instance;
	};

private:
	static ThisInstance *instance;
};

#endif /* THISINSTANCE_H_ */
