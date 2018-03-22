/*
 * Arc->h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_ARC_H_
#define SRC_UTILITIES_PATH_ARC_H_

#include "Waypoint.h"
#include "Line.h"
#include "RigidTransform2d.h"
#include "Rotation2d.h"
#include "Translation2d.h"
#include "PathSegment.h"
#include "Utilities/Constants.h"

    /**
     * An Arc object is formed by two Lines that share a common Waypoint-> Contains a center position, radius, and speed->
     */
class Arc {
public:
	Line *a;
	Line *b;
	Translation2d *center;
	double radius;
	double speed;

	Arc(Waypoint *a, Waypoint *b, Waypoint *c) {
		Arc(new Line(a, b), new Line(b, c));
	}

	Arc(Line *a, Line *b) {
		this->a = a;
		this->b = b;
		this->speed = (a->speed + b->speed) / 2;
		this->center = intersect(a, b);
		this->radius = (new Translation2d(center, a->end))->norm();
	}

private:
	void addToPath(Path *p) {
		a->addToPath(p, speed);
		if (radius > kEpsilon && radius < kReallyBigNumber) {
			p->addSegment(new PathSegment(a->end->x(), a->end->y(), b->start->x(), b->start->y(), center->x(), center->y(),
					speed, b->speed));
		}
	}

	static Translation2d *intersect(Line *l1, Line *l2) {
		RigidTransform2d *lineA = new RigidTransform2d(l1->end, (new Rotation2d(l1->slope->x(), l1->slope->y(), true))->normal());
		RigidTransform2d *lineB = new RigidTransform2d(l2->start, (new Rotation2d(l2->slope->x(), l2->slope->y(), true))->normal());
		return lineA->intersection(lineB);
	}
};

#endif /* SRC_UTILITIES_PATH_ARC_H_ */
