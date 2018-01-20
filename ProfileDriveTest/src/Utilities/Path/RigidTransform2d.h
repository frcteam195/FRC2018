/*
 * RigidTransform2d.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_RIGIDTRANSFORM2D_H_
#define SRC_UTILITIES_PATH_RIGIDTRANSFORM2D_H_

#include "Utilities/Constants.h"
#include "Translation2d.h"
#include "Rotation2d.h"
#include "MathUtils.h"
#include "Twist2d.h"
#include <math.h>
#include <limits>

using namespace std;

/**
 * Represents a 2d pose (rigid transform) containing translational and rotational elements->
 *
 * Inspired by Sophus (https://github->com/strasdat/Sophus/tree/master/sophus)
 */
class RigidTransform2d {
protected:
    Translation2d *translation_;
    Rotation2d *rotation_;

public:
    RigidTransform2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
    }

    RigidTransform2d(Translation2d *translation, Rotation2d *rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    RigidTransform2d(RigidTransform2d *other) {
        translation_ = new Translation2d(other->translation_);
        rotation_ = new Rotation2d(other->rotation_);
    }

    static RigidTransform2d *fromTranslation(Translation2d *translation) {
        return new RigidTransform2d(translation, new Rotation2d());
    }

    static RigidTransform2d *fromRotation(Rotation2d *rotation) {
        return new RigidTransform2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new RigidTransform2d from a (constant curvature) velocity-> See:
     * https://github->com/strasdat/Sophus/blob/master/sophus/se2->hpp
     */
    static RigidTransform2d *exp(Twist2d *delta) {
        double sin_theta = sin(delta->dtheta);
        double cos_theta = cos(delta->dtheta);
        double s, c;
        if (abs(delta->dtheta) < kEpsilon) {
            s = 1.0 - 1.0 / 6.0 * delta->dtheta * delta->dtheta;
            c = 0.5 * delta->dtheta;
        } else {
            s = sin_theta / delta->dtheta;
            c = (1.0 - cos_theta) / delta->dtheta;
        }
        return new RigidTransform2d(new Translation2d(delta->dx * s - delta->dy * c, delta->dx * c + delta->dy * s),
                new Rotation2d(cos_theta, sin_theta, false));
    }

    /**
     * Logical inverse of the above->
     */
    static Twist2d *log(RigidTransform2d *transform) {
        double dtheta = transform->getRotation()->getRadians();
        double half_dtheta = 0.5 * dtheta;
        double cos_minus_one = transform->getRotation()->rcos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (abs(cos_minus_one) < kEpsilon) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform->getRotation()->rsin()) / cos_minus_one;
        }
        Translation2d *translation_part = transform->getTranslation()
                ->rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part->x(), translation_part->y(), dtheta);
    }

    Translation2d *getTranslation() {
        return translation_;
    }

    void setTranslation(Translation2d *translation) {
        translation_ = translation;
    }

    Rotation2d *getRotation() {
        return rotation_;
    }

    void setRotation(Rotation2d *rotation) {
        rotation_ = rotation;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other->translation and then rotating by
     * other->rotation
     *
     * @param other
     *            The other transform->
     * @return This transform * other
     */
    RigidTransform2d *transformBy(RigidTransform2d *other) {
        return new RigidTransform2d(translation_->translateBy(other->translation_->rotateBy(rotation_)),
                rotation_->rotateBy(other->rotation_));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform->
     *
     * @return The opposite of this transform->
     */
    RigidTransform2d *inverse() {
        Rotation2d *rotation_inverted = rotation_->inverse();
        return new RigidTransform2d(translation_->inverse()->rotateBy(rotation_inverted), rotation_inverted);
    }

    RigidTransform2d *normal() {
        return new RigidTransform2d(translation_, rotation_->normal());
    }

    /**
     * Finds the point where the heading of this transform intersects the heading of another-> Returns (+INF, +INF) if
     * parallel->
     */
    Translation2d *intersection(RigidTransform2d *other) {
        Rotation2d *other_rotation = other->getRotation();
        if (rotation_->isParallel(other_rotation)) {
            // Lines are parallel->
            return new Translation2d(numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
        }
        if (abs(rotation_->rcos()) < abs(other_rotation->rcos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if the heading of this transform is colinear with the heading of another->
     */
    bool isColinear(RigidTransform2d *other) {
        const Twist2d *twist = log(inverse()->transformBy(other));
        return (MathUtils::epsilonEquals(twist->dy, 0.0, kEpsilon) && MathUtils::epsilonEquals(twist->dtheta, 0.0, kEpsilon));
    }

    /**
     * Do twist interpolation of this transform assuming constant curvature->
     */
    RigidTransform2d *interpolate(RigidTransform2d *other, double x) {
        if (x <= 0) {
            return new RigidTransform2d(this);
        } else if (x >= 1) {
            return new RigidTransform2d(other);
        }
        Twist2d *twist = log(inverse()->transformBy(other));
        return transformBy(exp(twist->scaled(x)));
    }

private:
    static Translation2d *intersectionInternal(RigidTransform2d *a, RigidTransform2d *b) {
        Rotation2d *a_r = a->getRotation();
        Rotation2d *b_r = b->getRotation();
        Translation2d *a_t = a->getTranslation();
        Translation2d *b_t = b->getTranslation();

        double tan_b = b_r->rtan();
        double t = ((a_t->x() - b_t->x()) * tan_b + b_t->y() - a_t->y()) / (a_r->rsin() - a_r->rcos() * tan_b);
        return a_t->translateBy((new Translation2d(a_r->rcos(), a_r->rsin()))->scale(t));
    }

};


#endif /* SRC_UTILITIES_PATH_RIGIDTRANSFORM2D_H_ */
