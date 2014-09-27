/*
 * Tracker.h
 *
 *  Created on: Sep 23, 2012
 *      Author: richard
 */

#ifndef TRACKER_H_
#define TRACKER_H_

using namespace cv;
using namespace std;

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "ITracker.h"



class Tracker : public ITracker {
public:

	Tracker(int trackNum);
	virtual ~Tracker();

	Result trackNewImage(const cv::Mat &image, Timings &timings);

private:
	int State;
	cv::Mat PrevGrey;
	map<unsigned int, TrackedGroup> TrackedGroups;
    int LargestTrackedGroup;
    vector<Point2f> TrackPoints[2];
    vector<uchar> TrackStatus;
	map<unsigned int, TrackedPoint> TrackedPoints;


    static const int MAX_COUNT = 500;

};

#endif /* TRACKER_H_ */
