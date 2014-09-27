#ifndef TRACKEDGROUP_H_
#define TRACKEDGROUP_H_

#include "opencv2/core/core.hpp"

using namespace cv;



class TrackedGroup {
public:
	TrackedGroup() {
		sum = Point2f(0,0);
		min = Point2f(INFINITY, INFINITY);
		max = Point2f(-INFINITY, -INFINITY);
		nPoints = 0;
	}

	Point2f minMaxMean() {
		return Point2f((max.x+min.x)/2, (max.y+min.y)/2);
	}

	CvSize groupSize() {
		return Size((int)(max.x-min.x), (int)(max.y-min.y));
	}


	Point2f sum;
	Point2f centre;
	Point2f min;
	Point2f max;
	unsigned int nPoints;
};

#endif
