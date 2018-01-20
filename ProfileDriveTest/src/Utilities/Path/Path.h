/*
 * Path->h
 *
 *  Created on: Jan 19, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_PATH_PATH_H_
#define SRC_UTILITIES_PATH_PATH_H_

#include <string>
#include <vector>
#include <math.h>
#include "Translation2d.h"
#include "PathSegment.h"

using namespace std;
/**
 * Class representing the robot's autonomous path->
 *
 * Field Coordinate System: Uses a right hand coordinate system-> Positive x is right, positive y is up, and the origin
 * is at the bottom left corner of the field-> For angles, 0 degrees is facing right (1, 0) and angles increase as you
 * turn counter clockwise->
 */

class Path {
public:
    vector<PathSegment *> *segments;
    PathSegment *prevSegment;
    vector<string> *mMarkersCrossed = new vector<string>();

//    void extrapolateLast() {
//        PathSegment last = segments->get(segments->size() - 1);
//        last->extrapolateLookahead(true);
//    }

    Translation2d *getEndPosition() {
        return segments->at(segments->size() - 1)->getEnd();
    }

    Path() {
        segments = new vector<PathSegment *>();
    }

    /**
     * add a segment to the Path
     *
     * @param segment
     *            the segment to add
     */
    void addSegment(PathSegment *segment) {
        segments->push_back(segment);
    }


    /**
     * @return the length of the current segment
     */
    double getSegmentLength() {
        PathSegment *currentSegment = segments->at(0);
        return currentSegment->getLength();
    }

//    static class TargetPointReport {
//        Translation2d closest_point;
//        double closest_point_distance;
//        double closest_point_speed;
//        Translation2d lookahead_point;
//        double max_speed;
//        double lookahead_point_speed;
//        double remaining_segment_distance;
//        double remaining_path_distance;
//
//        TargetPointReport() {
//        }
//    };
//
//    /**
//     * Gives the position of the lookahead point (and removes any segments prior to this point)->
//     *
//     * @param robot
//     *            Translation of the current robot pose->
//     * @return report containing everything we might want to know about the target point->
//     */
//    TargetPointReport getTargetPoint(Translation2d robot, Lookahead lookahead) {
//        TargetPointReport rv = new TargetPointReport();
//        PathSegment currentSegment = segments->get(0);
//        rv->closest_point = currentSegment->getClosestPoint(robot);
//        rv->closest_point_distance = new Translation2d(robot, rv->closest_point)->norm();
//        /*
//         * if (segments->size() > 1) { // Check next segment to see if it is closer-> final Translation2d
//         * next_segment_closest_point = segments->get(1)->getClosestPoint(robot); final double
//         * next_segment_closest_point_distance = new Translation2d(robot, next_segment_closest_point) ->norm(); if
//         * (next_segment_closest_point_distance < rv->closest_point_distance) { rv->closest_point =
//         * next_segment_closest_point; rv->closest_point_distance = next_segment_closest_point_distance;
//         * removeCurrentSegment(); currentSegment = segments->get(0); } }
//         */
//        rv->remaining_segment_distance = currentSegment->getRemainingDistance(rv->closest_point);
//        rv->remaining_path_distance = rv->remaining_segment_distance;
//        for (int i = 1; i < segments->size(); ++i) {
//            rv->remaining_path_distance += segments->get(i)->getLength();
//        }
//        rv->closest_point_speed = currentSegment
//                ->getSpeedByDistance(currentSegment->getLength() - rv->remaining_segment_distance);
//        double lookahead_distance = lookahead->getLookaheadForSpeed(rv->closest_point_speed) + rv->closest_point_distance;
//        if (rv->remaining_segment_distance < lookahead_distance && segments->size() > 1) {
//            lookahead_distance -= rv->remaining_segment_distance;
//            for (int i = 1; i < segments->size(); ++i) {
//                currentSegment = segments->get(i);
//                final double length = currentSegment->getLength();
//                if (length < lookahead_distance && i < segments->size() - 1) {
//                    lookahead_distance -= length;
//                } else {
//                    break;
//                }
//            }
//        } else {
//            lookahead_distance += (currentSegment->getLength() - rv->remaining_segment_distance);
//        }
//        rv->max_speed = currentSegment->getMaxSpeed();
//        rv->lookahead_point = currentSegment->getPointByDistance(lookahead_distance);
//        rv->lookahead_point_speed = currentSegment->getSpeedByDistance(lookahead_distance);
//        checkSegmentDone(rv->closest_point);
//        return rv;
//    }

//    /**
//     * Gives the speed the robot should be traveling at the given position
//     *
//     * @param robotPos
//     *            position of the robot
//     * @return speed robot should be traveling
//     */
//    double getSpeed(Translation2d *robotPos) {
//        PathSegment *currentSegment = segments->at(0);
//        return currentSegment->getSpeedByClosestPoint(robotPos);
//    }

    /**
     * Checks if the robot has finished traveling along the current segment then removes it from the path if it has
     *
     * @param robotPos
     *            robot position
     */
    void checkSegmentDone(Translation2d *robotPos) {
        PathSegment *currentSegment = segments->at(0);
        double remainingDist = currentSegment->getRemainingDistance(currentSegment->getClosestPoint(robotPos));
        if (remainingDist < kSegmentCompletionTolerance) {
            removeCurrentSegment();
        }
    }

    void removeCurrentSegment() {
    		prevSegment = segments->at(0);
        segments->erase(segments->begin());
        string marker = prevSegment->getMarker();
        if (marker != "")
            mMarkersCrossed->push_back(marker);
    }

//    /**
//     * Ensures that all speeds in the path are attainable and robot can slow down in time
//     */
//    void verifySpeeds() {
//        double maxStartSpeed = 0.0;
//        double startSpeeds[segments->size() + 1];
//        startSpeeds[segments->size()] = 0.0;
//        for (int i = segments->size() - 1; i >= 0; i--) {
//            PathSegment *segment = segments->at(i);
//            maxStartSpeed += sqrt(maxStartSpeed * maxStartSpeed + 2 * kPathFollowingMaxAccel * segment->getLength());
//            startSpeeds[i] = segment->getStartState()->vel();
//            // System->out->println(maxStartSpeed + ", " + startSpeeds[i]);
//            if (startSpeeds[i] > maxStartSpeed) {
//                startSpeeds[i] = maxStartSpeed;
//                // System->out->println("Segment starting speed is too high!");
//            }
//            maxStartSpeed = startSpeeds[i];
//        }
//        for (int i = 0; i < segments->size(); i++) {
//            PathSegment segment = segments->at(i);
//            double endSpeed = startSpeeds[i + 1];
//            MotionState startState = (i > 0) ? segments->get(i - 1)->getEndState() : new MotionState(0, 0, 0, 0);
//            startState = new MotionState(0, 0, startState->vel(), startState->vel());
//            segment->createMotionProfiler(startState, endSpeed);
//        }
//    }

    bool hasPassedMarker(string marker) {
    		bool retVal = false;
    		for (unsigned int i = 0; i < mMarkersCrossed->size(); i++) {
    			retVal |= mMarkersCrossed->at(i).find(marker) != std::string::npos;
    		}
    		return retVal;
    }

};
#endif /* SRC_UTILITIES_PATH_PATH_H_ */
