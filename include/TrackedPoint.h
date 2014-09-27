/*
 * TrackedPoint.h
 *
 *  Created on: Sep 6, 2012
 *      Author: richard
 */

#ifndef TRACKEDPOINT_H_
#define TRACKEDPOINT_H_

#include <math.h>

#include "opencv2/core/core.hpp"

using namespace cv;



class TrackedPoint {
public:
	TrackedPoint() {
	}

	TrackedPoint(Point2f firstVelocity) {
		histIdx = 1;
		velHistory[0] = firstVelocity;
		histFill = 1;
		objectId = -1;
		averageVel = Point2f(0,0);
	}

	bool IsMoving() const {
		if(histFill < NUM_VELOC_FRAMES) {
			return false;
		}

		return (abs(averageVel.x) > MVTHRESH) || (abs(averageVel.y) > MVTHRESH);
	}

	bool HasValidAverage() {
		return histFill >= NUM_VELOC_FRAMES;
	}

	void UpdateAverageVelocity() {
		if(histFill >= NUM_VELOC_FRAMES) {

			averageVel = Point2f(0.,0.);
			for( unsigned int vi=0; vi<NUM_VELOC_FRAMES; vi++) {
				averageVel.x += velHistory[vi].x / NUM_VELOC_FRAMES;
				averageVel.y += velHistory[vi].y / NUM_VELOC_FRAMES;
			}
		}
	}

	float CalcVelDiff(const TrackedPoint &other) {
		return sqrt((averageVel.x - other.averageVel.x) * (averageVel.x - other.averageVel.x)
				+ (averageVel.y - other.averageVel.y) * (averageVel.y - other.averageVel.y));
	}

	Point2f pos;
	Point2f velHistory[NUM_VELOC_FRAMES];
	unsigned int histIdx;
	unsigned int histFill;
	Point2f averageVel;
	int objectId;
};


#endif /* TRACKEDPOINT_H_ */
