/*
 * Point.h
 *
 *  Created on: Jan 12, 2018
 *      Author: chris bonomi
 */

#ifndef POINT_H_
#define POINT_H_

struct Point {
	double x;
	double y;
	Point(double x, double y) {
		this->x = x;
		this->y = y;
	}
};

#endif /* POINT_H_ */
