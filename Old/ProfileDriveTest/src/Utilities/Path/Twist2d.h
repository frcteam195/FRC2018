/*
 * Twist2d.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_TWIST2D_H_
#define SRC_UTILITIES_TWIST2D_H_

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 *
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
class Twist2d {
public:

    double dx;
    double dy;
    double dtheta; // Radians!

    Twist2d(double dx, double dy, double dtheta) {
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
    }

    Twist2d *scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }
};
#endif /* SRC_UTILITIES_TWIST2D_H_ */
