/*
 * Tracker.cpp
 *
 *  Created on: Sep 23, 2012
 *      Author: richard
 */



#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "TrackParams.h"
#include "Timings.h"
#include "TrackedGroup.h"
#include "Tracker.h"
#include "TrackedPoint.h"


using namespace cv;
using namespace std;

Tracker::Tracker(int trackNum) {

	TrackNum = trackNum;
    State = FIND;
    LargestTrackedGroup = -1;
}

Tracker::~Tracker() {
	// TODO Auto-generated destructor stub
}

Tracker::Result Tracker::trackNewImage(const cv::Mat &image, Timings &timings)  {

	cv::Mat grey;
	cvtColor(image, grey, CV_BGR2GRAY);

    static TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    static Size subPixWinSize(10,10), winSize(31,31);

	// Image processing here
	if(State==FIND) {
		// printf("%d: State=FIND, clear tracked groups\n", TrackNum);
		// No groups tracked
		TrackedGroups.clear();
		LargestTrackedGroup = -1;

		// Find points to track
		double tFindFeatureStart = Timings::getTimeUsecs();

		try {
			goodFeaturesToTrack(grey, TrackPoints[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);

			cornerSubPix(grey, TrackPoints[1], subPixWinSize, Size(-1,-1), termcrit);
		}
		catch(cv::Exception &e) {
			fprintf(stderr, "%d: Find threw %s\n", TrackNum, e.what());
			TrackPoints[1].clear();
			State = FIND;
		}

        timings.FindFeatureTime += Timings::getTimeUsecs() - tFindFeatureStart;


        if(TrackPoints[1].size() > 0) {
        	// printf("%d: Tracked %d points, set state to ACQUIRE\n", TrackNum, (int)TrackPoints[1].size());
        	State = ACQUIRE;
        }
	} else if(State==ACQUIRE || State==TRACK) {
		// Got some points, do a Lukas-Kanade
		// printf("%d: State=%d, Lukas-Kanade\n", TrackNum, State);

        vector<float> err;
        if(PrevGrey.empty())
            grey.copyTo(PrevGrey);

        double tLkStart = Timings::getTimeUsecs();

        calcOpticalFlowPyrLK(PrevGrey, grey, TrackPoints[0], TrackPoints[1], TrackStatus, err, winSize,
                             3, termcrit, 0, 0.001);

        timings.TrackTime += Timings::getTimeUsecs() - tLkStart;

        double tFeatureProcessStart = Timings::getTimeUsecs();

        // Calculate velocities for tracked points
        for(unsigned int n=0; n<TrackStatus.size(); n++) {
        	if(TrackStatus[n]) {
        		Point2f dp = Point2f(TrackPoints[1][n].x - TrackPoints[0][n].x, TrackPoints[1][n].y - TrackPoints[0][n].y);

        		if(TrackedPoints.count(n)) {
					TrackedPoint &tp = TrackedPoints[n];

					tp.pos = TrackPoints[1][n];
        			tp.velHistory[tp.histIdx++] = dp;
        			if(tp.histIdx >= NUM_VELOC_FRAMES) {
        				tp.histFill = NUM_VELOC_FRAMES;
        				tp.histIdx = 0;
        			} else if(tp.histFill < NUM_VELOC_FRAMES) {
        				tp.histFill = tp.histIdx;
        			}

        			tp.UpdateAverageVelocity();
        		}
        		else {
        			// no point yet
        			TrackedPoint newtp(dp);
					newtp.pos = TrackPoints[1][n];

        			TrackedPoints[n] = newtp;
        		}

//        		TrackedPoint &dtp = trackedPoints[n];
//	            		printf("Tracked point %d: (obj %d, moving %d [%f,%f], hf=%d) (%f,%f)\n",
//	            				n, dtp.objectId, dtp.IsMoving(), dtp.averageVel.x, dtp.averageVel.y,
//	            				dtp.histFill,
//	            				TrackPoints[1][n].x, TrackPoints[1][n].y);

        	}
        	else {
        		// Point n untracked
        		TrackedPoints.erase(n);
        		// printf("Erase TrackedPoints %d (status)\n", n);
        	}
        }

        // Identify (new) objects
        bool averageDone = false;
        for(map<unsigned int, TrackedPoint>::iterator it=TrackedPoints.begin(); it != TrackedPoints.end(); it++) {

        	TrackedPoint &tp = (*it).second;

        	averageDone |= tp.HasValidAverage();

        	if(tp.objectId < 0 && tp.IsMoving()) {
        		// Not bound to an object and is moving

        		for(map<unsigned int, TrackedPoint>::iterator iit=TrackedPoints.begin(); iit != TrackedPoints.end(); iit++) {
        			TrackedPoint &itp = (*iit).second;
        			if(itp.objectId >= 0 && itp.IsMoving()) {
        				if(itp.CalcVelDiff(tp) < GROUP_THRESH) {
        					// We are in the same object
        					tp.objectId = itp.objectId;
	            			//printf("%d: Found object %d for point %d (%f,%f) vel (%f,%f)\n",
	            			//		idx, tp.objectId, (*it).first, tp.pos.x, tp.pos.y, tp.averageVel.x, tp.averageVel.y);
        					break;
        				}
        			}
        		}

        		if(tp.objectId < 0) {
        			// No connected object found
        			static int nextObjectId = 0;
        			tp.objectId = nextObjectId++;
        			//printf("%d: Added new object %d for point %d (%f,%f) vel (%f,%f)\n\n",
        			//		idx, tp.objectId, (*it).first, tp.pos.x, tp.pos.y, tp.averageVel.x, tp.averageVel.y);
        		}
        	}
        	else if(tp.objectId >= 0 && !tp.IsMoving()) {
        		// Object stopped moving
        		//printf("%d: point %d, object %d stopped\n", idx, (*it).first, tp.objectId);
        		tp.objectId = -1;	// remove from object
        	}
        }

        // zero point counters and sum
        for(map<unsigned int, TrackedGroup>::iterator ig= TrackedGroups.begin(); ig != TrackedGroups.end(); ig++) {
        	TrackedGroup &tg = (*ig).second;
        	tg.nPoints = 0;
        	tg.sum = Point2f(0,0);
        	tg.max = Point2f(-INFINITY,-INFINITY);
        	tg.min = Point2f(INFINITY,INFINITY);
        }

        for(map<unsigned int, TrackedPoint>::iterator it=TrackedPoints.begin(); it != TrackedPoints.end(); it++) {
			TrackedPoint &tp = (*it).second;
			if(tp.objectId >= 0 && tp.IsMoving()) {
				if(TrackedGroups.count(tp.objectId)) {
					TrackedGroup &tg = TrackedGroups[tp.objectId];
					tg.nPoints++;
					tg.sum.x += tp.pos.x;
					tg.sum.y += tp.pos.y;
					if(tg.max.x < tp.pos.x) tg.max.x = tp.pos.x;
					if(tg.max.y < tp.pos.y) tg.max.y = tp.pos.y;
					if(tg.min.x > tp.pos.x) tg.min.x = tp.pos.x;
					if(tg.min.y > tp.pos.y) tg.min.y = tp.pos.y;
				}
				else {
					TrackedGroup newtg;
					newtg.nPoints = 1;
					newtg.sum = tp.pos;
					newtg.max = tp.pos;
					newtg.min = tp.pos;
					TrackedGroups[tp.objectId] = newtg;
				}
			}
        }

        unsigned int npMax = 0;
        for(map<unsigned int, TrackedGroup>::iterator ig= TrackedGroups.begin(); ig != TrackedGroups.end();) {
        	TrackedGroup &tg = (*ig).second;

        	if(tg.nPoints==0) {
        		// printf("%d: Object %d has no points\n", TrackNum, (*ig).first);
        		TrackedGroups.erase(ig++);
        		// printf("%d: Erased object\n", TrackNum);
        	}
        	else {
				tg.centre.x = tg.sum.x / tg.nPoints;
				tg.centre.y = tg.sum.y / tg.nPoints;

				if(tg.nPoints > npMax) {
					npMax = tg.nPoints;
					LargestTrackedGroup = (*ig).first;
				}
			// printf("  Tracked group %d: np=%d, c=%f,%f\n", (*ig).first, tg.nPoints, tg.centre.x, tg.centre.y);
			ig++;
        	}
        }


        if(TrackedGroups.size() > 0) {
        	if(State != TRACK) {
        		printf("%d: %d tracked groups; State=>TRACK\n", TrackNum, (int)TrackedGroups.size());
        	}
        	State = TRACK;
        }
        else if(averageDone) {
        	if(State != FIND) {
        		// printf("%d: Zero tracked groups after average, State=>FIND\n", TrackNum);
        	}
        	State = FIND;	// reset the state if we have averaged points but nothing tracked
        }
        else {
        	printf("%d: Averaging, state=%d\n", TrackNum, State);
        }

        timings.FeatureProcessTime += Timings::getTimeUsecs() - tFeatureProcessStart;
	}

    std::swap(TrackPoints[1], TrackPoints[0]);
    std::swap(PrevGrey, grey);

    TrackedGroup *pLargestTracked;

    if(LargestTrackedGroup < 0) {
    	pLargestTracked = NULL;
    }
    else {
    	pLargestTracked = &(TrackedGroups[LargestTrackedGroup]);
    }

    Result res;
    res.pObjectPos = (pLargestTracked==NULL) ? NULL : &(pLargestTracked->centre);
    res.State = State;
    res.pAllPoints = &(TrackPoints[1]);
    res.pPointStatus=  &TrackStatus;
    res.pTrackedPoints=	&TrackedPoints;
   // res.ObjectGroup = new TrackedGroup(*pLargestTracked);

    return res;
}
