/*
 * QuickTracker.cpp
 *
 *  Created on: Sep 23, 2012
 *      Author: richard
 */



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "TrackParams.h"
#include "Timings.h"
#include "TrackedGroup.h"
#include "QuickTracker.h"
#include "TrackedPoint.h"
#include "FindAreas.h"


using namespace cv;
using namespace std;

QuickTracker::QuickTracker(int trackNum, int numSamples, int limit, int edge, int termCount ) {

	TrackNum = trackNum;
	DiffBufferIndex = 0;
	NumSamples = numSamples;
	DiffBuffer = new cv::Mat[NumSamples];
	DiffBufferComplete = false;
	Limit = limit;
	Edge = edge;
	TermCount = termCount;
}

QuickTracker::~QuickTracker() {
	delete [] DiffBuffer;
}

QuickTracker::Result QuickTracker::trackNewImage(const cv::Mat &image, Timings &timings)  {
	fprintf(stderr, "Track start\n");

	double startTime = Timings::getTimeUsecs();

	// Convert to grey
	cv::Mat grey;
	cvtColor(image, grey, CV_BGR2GRAY);
    if(PrevGrey.empty())
        grey.copyTo(PrevGrey);

	// Diff with PrevGrey
	cv::Mat instDiff;
	cv::subtract(grey, PrevGrey, instDiff);
    std::swap(PrevGrey, grey);

	// Put difference in averaging array
    DiffBuffer[DiffBufferIndex] = instDiff;
    DiffBufferIndex = (DiffBufferIndex+1) % NumSamples;
    DiffBufferComplete = DiffBufferComplete || (DiffBufferIndex==0);

    Result res;
    res.pObjectPos = NULL;
    res.State = ACQUIRE;
    if(!DiffBufferComplete) {
    	timings.TrackTime += Timings::getTimeUsecs() - startTime;

    	fprintf(stderr, "Track has not all buffers yet, DiffBufferIndex=%d, NumSamples=%d\n",
    			DiffBufferIndex, NumSamples);
    	return res;
    }

	// Calculate average
	cv::Mat total(grey.rows, grey.cols, grey.type());
	for(int n=0; n<NumSamples; n++) {
		cv::Mat temp;
		cv::add(total, DiffBuffer[n], temp);
		total = temp;
	}

	Mat avg = total.mul(1.0/NumSamples);
	Mat avgBytes;
	avg.convertTo(avgBytes, CV_8U);

	double matTime = Timings::getTimeUsecs() - startTime;
	fprintf(stderr, "Starting FindLargest, matTime=%0.3f\n", matTime);

	Rect objRect = FindAreas::FindLargest(Limit, Edge, avgBytes, TermCount);

	if(objRect.height==0) {
		fprintf(stderr, "No target found\n");

		timings.TrackTime += Timings::getTimeUsecs() - startTime;
		return res;
	}

	Centre = Point2f(objRect.x + objRect.width/2, objRect.y + objRect.height/2);

    res.pObjectPos = &Centre;
    res.State = TRACK;

	double thisTime = Timings::getTimeUsecs() - startTime;
	timings.TrackTime += thisTime;

	fprintf(stderr, "Track done %0.3f\n", thisTime);
    return res;
}
