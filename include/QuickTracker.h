/*
 * QuickTracker.h
 *
 *  Created on: Sep 23, 2012
 *      Author: richard
 */

#ifndef QUICKTRACKER_H_
#define QUICKTRACKER_H_

using namespace cv;
using namespace std;

#include "ITracker.h"

// #define NUMSAMPLES 8

class QuickTracker : public ITracker {
public:

	QuickTracker(int trackNum, int numSamples, int limit, int edge, int termCount );
	virtual ~QuickTracker();

	Result trackNewImage(const cv::Mat &image, Timings &timings);

private:
	Mat PrevGrey;
	int NumSamples;
	int Limit;
	int Edge;
	int TermCount;
	Mat *DiffBuffer;
	int DiffBufferIndex;
	bool DiffBufferComplete;
	cv::Point2f Centre;
};

#endif /* QUICKTRACKER_H_ */
