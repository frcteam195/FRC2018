/*
 * PathSegment->h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_PATHSEGMENT_H_
#define SRC_UTILITIES_PATH_PATHSEGMENT_H_

#include "Translation2d.h"
#include "Utilities/Constants.h"
#include <string>

/**
 * Class representing a segment of the robot's autonomous path->
 */

class PathSegment {
private:
    Translation2d *start;
    Translation2d *end;
    Translation2d *center;
    Translation2d *deltaStart;
    Translation2d *deltaEnd;
    double maxSpeed;
    bool isLine;
    bool extrapolateLookahead;
    string marker;

//    double getDistanceTravelled(Translation2d robotPosition) {
//        Translation2d pathPosition = getClosestPoint(robotPosition);
//        double remainingDist = getRemainingDistance(pathPosition);
//        return getLength() - remainingDist;
//
//    }

public:

    /**
     * Constructor for a linear segment
     *
     * @param x1
     *            start x
     * @param y1
     *            start y
     * @param x2
     *            end x
     * @param y2
     *            end y
     * @param maxSpeed
     *            maximum speed allowed on the segment
     */
    PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, double endSpeed) {
        this->start = new Translation2d(x1, y1);
        this->end = new Translation2d(x2, y2);

        this->deltaStart = new Translation2d(start, end);

        this->maxSpeed = maxSpeed;
        isLine = true;
    }

    PathSegment(double x1, double y1, double x2, double y2, double maxSpeed, double endSpeed, string marker) {
        this->start = new Translation2d(x1, y1);
        this->end = new Translation2d(x2, y2);

        this->deltaStart = new Translation2d(start, end);

        this->maxSpeed = maxSpeed;
        isLine = true;
        this->marker = marker;
    }

    /**
     * Constructor for an arc segment
     *
     * @param x1
     *            start x
     * @param y1
     *            start y
     * @param x2
     *            end x
     * @param y2
     *            end y
     * @param cx
     *            center x
     * @param cy
     *            center y
     * @param maxSpeed
     *            maximum speed allowed on the segment
     */
    PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed , double endSpeed) {
        this->start = new Translation2d(x1, y1);
        this->end = new Translation2d(x2, y2);
        this->center = new Translation2d(cx, cy);

        this->deltaStart = new Translation2d(center, start);
        this->deltaEnd = new Translation2d(center, end);

        this->maxSpeed = maxSpeed;
        isLine = false;
    }

    PathSegment(double x1, double y1, double x2, double y2, double cx, double cy, double maxSpeed, double endSpeed, string marker) {
        this->start = new Translation2d(x1, y1);
        this->end = new Translation2d(x2, y2);
        this->center = new Translation2d(cx, cy);

        this->deltaStart = new Translation2d(center, start);
        this->deltaEnd = new Translation2d(center, end);

        this->maxSpeed = maxSpeed;
        isLine = false;
        this->marker = marker;
    }

    /**
     * @return max speed of the segment
     */
    double getMaxSpeed() {
        return maxSpeed;
    }

    /**
     * @return starting point of the segment
     */
    Translation2d *getStart() {
        return start;
    }

    /**
     * @return end point of the segment
     */
    Translation2d *getEnd() {
        return end;
    }

    /**
     * @return the total length of the segment
     */
    double getLength() {
        if (isLine) {
            return deltaStart->norm();
        } else {
            return deltaStart->norm() * Translation2d::getAngle(deltaStart, deltaEnd)->getRadians();
        }
    }


    /**
     * Gets the point on the segment closest to the robot
     *
     * @param position
     *            the current position of the robot
     * @return the point on the segment closest to the robot
     */
    Translation2d *getClosestPoint(Translation2d *position) {
        if (isLine) {
            Translation2d *delta = new Translation2d(start, end);
            double u = ((position->x() - start->x()) * delta->x() + (position->y() - start->y()) * delta->y())
                    / (delta->x() * delta->x() + delta->y() * delta->y());
            if (u >= 0 && u <= 1)
                return new Translation2d(start->x() + u * delta->x(), start->y() + u * delta->y());
            return (u < 0) ? start : end;
        } else {
            Translation2d *deltaPosition = new Translation2d(center, position);
            deltaPosition = deltaPosition->scale(deltaStart->norm() / deltaPosition->norm());
            if (Translation2d::cross(deltaPosition, deltaStart) * Translation2d::cross(deltaPosition, deltaEnd) < 0) {
                return center->translateBy(deltaPosition);
            } else {
                Translation2d *startDist = new Translation2d(position, start);
                Translation2d *endDist = new Translation2d(position, end);
                return (endDist->norm() < startDist->norm()) ? end : start;
            }
        }
    }

    /**
     * Calculates the point on the segment <code>dist</code> distance from the starting point along the segment->
     *
     * @param dist
     *            distance from the starting point
     * @return point on the segment <code>dist</code> distance from the starting point
     */
    Translation2d *getPointByDistance(double dist) {
        double length = getLength();
        if (dist > length) {
            dist = length;
        }
        if (isLine) {
            return start->translateBy(deltaStart->scale(dist / length));
        } else {
            double deltaAngle = Translation2d::getAngle(deltaStart, deltaEnd)->getRadians()
                    * ((Translation2d::cross(deltaStart, deltaEnd) >= 0) ? 1 : -1);
            deltaAngle *= dist / length;
            Translation2d *t = deltaStart->rotateBy(Rotation2d::fromRadians(deltaAngle));
            return center->translateBy(t);
        }
    }

    /**
     * Gets the remaining distance left on the segment from point <code>point</code>
     *
     * @param point
     *            result of <code>getClosestPoint()</code>
     * @return distance remaining
     */
    double getRemainingDistance(Translation2d *position) {
        if (isLine) {
            return (new Translation2d(end, position))->norm();
        } else {
            Translation2d *deltaPosition = new Translation2d(center, position);
            double angle = Translation2d::getAngle(deltaEnd, deltaPosition)->getRadians();
            double totalAngle = Translation2d::getAngle(deltaStart, deltaEnd)->getRadians();
            return angle / totalAngle * getLength();
        }
    }

    string getMarker() {
        return marker;
    }

};

#endif /* SRC_UTILITIES_PATH_PATHSEGMENT_H_ */
