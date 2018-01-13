/*
 * TestTemplate.h
 *
 *  Created on: Jan 12, 2018
 *      Author: roberthilton
 */

#ifndef TESTTEMPLATE_H_
#define TESTTEMPLATE_H_


template<class T> class CKAutoBuilder {
public:
	CKAutoBuilder(T *t);

private:
	T *t;
};

template <class T>
CKAutoBuilder<T>::CKAutoBuilder(T *t) {
	this->t = t;
}
#endif /* TESTTEMPLATE_H_ */
