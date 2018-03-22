/*
 * PathBuilder.h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_PATHBUILDER_H_
#define SRC_UTILITIES_PATH_PATHBUILDER_H_

#include "WPILib.h"
#include "Path.h"
#include "Translation2d.h"
#include "Waypoint.h"
#include <string>

using namespace std;
using namespace frc;

/**
 * Class used to convert a list of Waypoints into a Path object consisting of arc and line PathSegments
 *
 * @see Waypoint
 * @see Path
 * @see PathSegment
 */
class PathBuilder {
private:
    static const double kEpsilon = 1E-9;
    static const double kReallyBigNumber = 1E9;

	static Waypoint *getPoint(vector<Waypoint *> *w, int i) {
		if (i > w->size())
			return w->at(w->size() - 1);
		return w->at(i);
	};

public:
    static Path *buildPathFromWaypoints(vector<Waypoint *> *w) {
        Path *p = new Path();
        if (w->size() < 2)
            throw new Error("Path must contain at least 2 waypoints");
        int i = 0;
        if (w->size() > 2) {
            do {
                (new Arc(getPoint(w, i), getPoint(w, i + 1), getPoint(w, i + 2)))->addToPath(p);
                i++;
            } while (i < w->size() - 2);
        }
        (new Line(w->at(w->size() - 2), w->at(w->size() - 1)))->addToPath(p, 0);
        // System.out.println(p);
        return p;
    }

};



#endif /* SRC_UTILITIES_PATH_PATHBUILDER_H_ */
