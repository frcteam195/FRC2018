/*
 * MPTrajectory.h
 *
 *  Created on: Jan 17, 2018
 *      Author: roberthilton
 */

#ifndef SRC_UTILITIES_MPTRAJECTORYHELPER_H_
#define SRC_UTILITIES_MPTRAJECTORYHELPER_H_

#include <vector>

using namespace std;

class MPTrajectoryHelper {
private:

public:
	static vector<vector <double> *> *getVectorFromProfile(const double kMotionProfile[][3], int kMotionProfileSz) {
		vector<vector <double> *> *retVal = new vector<vector <double> *> ();
		for (int i = 0; i < kMotionProfileSz; i++) {
			retVal->push_back(new vector<double>({kMotionProfile[i][0], kMotionProfile[i][1], kMotionProfile[i][2]}));
		}
		return retVal;
	}
};


#endif /* SRC_UTILITIES_MPTRAJECTORYHELPER_H_ */
