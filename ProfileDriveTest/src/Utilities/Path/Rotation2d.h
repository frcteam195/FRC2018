/*
 * Rotation2d.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_ROTATION2D_H_
#define SRC_UTILITIES_PATH_ROTATION2D_H_

#include <math.h>
#include "MathUtils.h"
#include <limits>
#include "Utilities/Constants.h"

using namespace std;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle (cosine and sine).
 *
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
class Rotation2d {
protected:
    //static const Rotation2d *kIdentity = new Rotation2d();

    double cos_angle_;
    double sin_angle_;

public:

//    static const Rotation2d identity() {
//        return kIdentity;
//    }

    Rotation2d() { // @suppress("Class members should be properly initialized")
    		Rotation2d(1, 0, false);
    }

    Rotation2d(double x, double y, bool normalize) {
        cos_angle_ = x;
        sin_angle_ = y;
        if (normalize) {
            this->normalize();
        }
    }

    Rotation2d(Rotation2d *other) {
        cos_angle_ = other->cos_angle_;
        sin_angle_ = other->sin_angle_;
    }

    static Rotation2d *fromRadians(double angle_radians) {
        return new Rotation2d(cos(angle_radians), sin(angle_radians), false);
    }

    static Rotation2d *fromDegrees(double angle_degrees) {
        return fromRadians(MathUtils::toRadians(angle_degrees));
    }

    /**
     * From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
     * Normalizing forces us to re-scale the sin and cos to reset rounding errors.
     */
    void normalize() {
        double magnitude = MathUtils::hypot(cos_angle_, sin_angle_);
        if (magnitude > kEpsilon) {
            sin_angle_ /= magnitude;
            cos_angle_ /= magnitude;
        } else {
            sin_angle_ = 0;
            cos_angle_ = 1;
        }
    }

    double rcos() {
        return cos_angle_;
    }

    double rsin() {
        return sin_angle_;
    }

    double rtan() {
        if (abs(cos_angle_) < kEpsilon) {
            if (sin_angle_ >= 0.0) {
                return numeric_limits<double>::infinity();
            } else {
                return -numeric_limits<double>::infinity();
            }
        }
        return sin_angle_ / cos_angle_;
    }

    double getRadians() {
        return atan2(sin_angle_, cos_angle_);
    }

    double getDegrees() {
    		return getRadians() * 180.0 / M_PI;
    }

    /**
     * We can rotate this Rotation2d by adding together the effects of it and another rotation.
     *
     * @param other
     *            The other rotation. See: https://en.wikipedia.org/wiki/Rotation_matrix
     * @return This rotation rotated by other.
     */
    Rotation2d *rotateBy(Rotation2d *other) {
        return new Rotation2d(cos_angle_ * other->cos_angle_ - sin_angle_ * other->sin_angle_,
                cos_angle_ * other->sin_angle_ + sin_angle_ * other->cos_angle_, true);
    }

    Rotation2d *normal() {
        return new Rotation2d(-sin_angle_, cos_angle_, false);
    }

    /**
     * The inverse of a Rotation2d "undoes" the effect of this rotation.
     *
     * @return The opposite of this rotation.
     */
    Rotation2d *inverse() {
        return new Rotation2d(cos_angle_, -sin_angle_, false);
    }

    bool isParallel(Rotation2d *other) {
        return MathUtils::epsilonEquals(MathUtils::cross(cos_angle_, sin_angle_, other->cos_angle_, other->sin_angle_), 0.0, kEpsilon);
    }

    Rotation2d *interpolate(Rotation2d *other, double x) {
        if (x <= 0) {
            return new Rotation2d(this);
        } else if (x >= 1) {
            return new Rotation2d(other);
        }
        double angle_diff = inverse()->rotateBy(other)->getRadians();
        return this->rotateBy(Rotation2d::fromRadians(angle_diff * x));
    }

};
#endif /* SRC_UTILITIES_PATH_ROTATION2D_H_ */
