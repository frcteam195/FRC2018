/*
 * MathUtils.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_MATHUTILS_H_
#define SRC_UTILITIES_PATH_MATHUTILS_H_

#include <math.h>

class MathUtils {
public:
	static double hypot(double x_, double y_) {
		return sqrt(pow(x_, 2) + pow(y_, 2));
	};

	static double toRadians(double angdeg) {
		return angdeg / 180.0 * M_PI;
	};

	static bool epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

    static double cross(double x1, double y1, double x2, double y2) {
        return x1 * y2 - y1 * x2;
    }
};


#endif /* SRC_UTILITIES_PATH_MATHUTILS_H_ */
