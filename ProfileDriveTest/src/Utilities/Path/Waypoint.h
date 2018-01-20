/*
 * Waypoint.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_WAYPOINT_H_
#define SRC_UTILITIES_PATH_WAYPOINT_H_

#include "Translation2d.h"
#include <string>

/**
 * A waypoint along a path. Contains a position, radius (for creating curved paths), and speed. The information from
 * these waypoints is used by the PathBuilder class to generate Paths. Waypoints also contain an optional marker
 * that is used by the WaitForPathMarkerAction.
 *
 * @see PathBuilder
 * @see WaitForPathMarkerAction
 */
class Waypoint {
public:
    Translation2d *position;
    double radius;
    double speed;
    string marker;

    Waypoint(double x, double y, double r, double s) {
        position = new Translation2d(x, y);
        radius = r;
        speed = s;
    };

    Waypoint(Translation2d *pos, double r, double s) {
        position = pos;
        radius = r;
        speed = s;
    };

    Waypoint(double x, double y, double r, double s, string m) {
        position = new Translation2d(x, y);
        radius = r;
        speed = s;
        marker = m;
    };

    Waypoint(Waypoint *other) {
        Waypoint(other->position->x(), other->position->y(), other->radius, other->speed, other->marker);
    };
};

#endif /* SRC_UTILITIES_PATH_WAYPOINT_H_ */
