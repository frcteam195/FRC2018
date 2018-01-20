/*
 * Translation2d.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_TRANSLATION2D_H_
#define SRC_UTILITIES_PATH_TRANSLATION2D_H_

#include <math.h>
#include "Rotation2d.h"
#include "MathUtils.h"

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
class Translation2d {
protected:
   //static const Translation2d *kIdentity = new Translation2d();
   double x_;
   double y_;

public:
//    static const Translation2d identity() {
//        return kIdentity;
//    }



    Translation2d() {
        x_ = 0;
        y_ = 0;
    }

    Translation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    Translation2d(Translation2d *other) {
        x_ = other->x_;
        y_ = other->y_;
    }

    Translation2d(Translation2d *start, Translation2d *end) {
        x_ = end->x_ - start->x_;
        y_ = end->y_ - start->y_;
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x^2 + y^2)
     */
    double norm() {
    		return MathUtils::hypot(x_, y_);
    }

    double norm2() {
        return x_ * x_ + y_ * y_;
    }

    double x() {
        return x_;
    }

    double y() {
        return y_;
    }

    void setX(double x) {
        x_ = x;
    }

    void setY(double y) {
        y_ = y;
    }

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other
     *            The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    Translation2d *translateBy(Translation2d *other) {
        return new Translation2d(x_ + other->x_, y_ + other->y_);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation
     *            The rotation to apply.
     * @return This translation rotated by rotation.
     */
    Translation2d *rotateBy(Rotation2d *rotation) {
        return new Translation2d(x_ * rotation->rcos() - y_ * rotation->rsin(), x_ * rotation->rsin() + y_ * rotation->rcos());
    }

    Rotation2d *direction() {
        return new Rotation2d(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    Translation2d *inverse() {
        return new Translation2d(-x_, -y_);
    }

    Translation2d *interpolate(Translation2d *other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    Translation2d *extrapolate(Translation2d *other, double x) {
        return new Translation2d(x * (other->x_ - x_) + x_, x * (other->y_ - y_) + y_);
    }

    Translation2d *scale(double s) {
        return new Translation2d(x_ * s, y_ * s);
    }

    static double dot(Translation2d *a, Translation2d *b) {
        return a->x_ * b->x_ + a->y_ * b->y_;
    }

    static Rotation2d *getAngle(Translation2d *a, Translation2d *b) {
        double cos_angle = dot(a, b) / (a->norm() * b->norm());
        if (isnan(cos_angle)) {
            return new Rotation2d();
        }
        return Rotation2d::fromRadians(acos(min(1.0, max(cos_angle, -1.0))));
    }

	static double cross(Translation2d *a, Translation2d *b) {
		return a->x_ * b->y_ - a->y_ * b->x_;
	}

};



#endif /* SRC_UTILITIES_PATH_TRANSLATION2D_H_ */
