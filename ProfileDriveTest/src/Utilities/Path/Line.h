/*
 * Line->h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_LINE_H_
#define SRC_UTILITIES_PATH_LINE_H_

#include "Path.h"
#include "Waypoint.h"
#include "Translation2d.h"

    /**
     * A Line object is formed by two Waypoints-> Contains a start and end position, slope, and speed->
     */
class Line {
public:
        Waypoint *a;
        Waypoint *b;
        Translation2d *start;
        Translation2d *end;
        Translation2d *slope;
        double speed;

        Line(Waypoint *a, Waypoint *b) {
            this->a = a;
            this->b = b;
            slope = new Translation2d(a->position, b->position);
            speed = b->speed;
            start = a->position->translateBy(slope->scale(a->radius / slope->norm()));
            end = b->position->translateBy(slope->scale(-b->radius / slope->norm()));
        }

		void addToPath(Path *p, double endSpeed) {
			double pathLength = (new Translation2d(end, start))->norm();
			if (pathLength > kEpsilon) {
				if (b->marker != "") {
					p->addSegment(new PathSegment(start->x(), start->y(), end->x(), end->y(), b->speed, endSpeed, b->marker));
				} else {
					p->addSegment(new PathSegment(start->x(), start->y(), end->x(), end->y(), b->speed, endSpeed));
				}
			}

		}
};



#endif /* SRC_UTILITIES_PATH_LINE_H_ */
