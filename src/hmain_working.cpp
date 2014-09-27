#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <queue>
#include <ctype.h>
#include <stdio.h>
#include <sys/time.h>

const float PIXELS_SECOND_THRESH = 10.;
const float PIXELS_SECOND_GROUP_THRESH = 30.;
const float FRAMES_SECOND=30.;
const float GROUP_THRESH = PIXELS_SECOND_GROUP_THRESH / FRAMES_SECOND;
const unsigned int NUM_VELOC_FRAMES = 15;
const float MVTHRESH = PIXELS_SECOND_THRESH / FRAMES_SECOND;

#include "TrackedPoint.h"
#include "TrackedGroup.h"

using namespace cv;
using namespace std;


VideoCapture cap;
const char *WindowName = "CV Harness";
char * IoFiles[2] = {NULL, NULL};
VideoWriter Vwrite;
CvSize LastImageSize;
bool IsCam;

void help()
{
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a test harness for OpenCV,\n"
    		"Using OpenCV version %s\n" << CV_VERSION << "\n"
    		"Options:\n"
    		" -f1 <file> define io file 1\n"
    		" -f2 <file> define io file 2\n"
    		<< endl;

    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tc - capture from cam 1\n"
            "\td - capture from cam 2\n"
            "\tf - process file 1\n"
            "\tg - process file 2\n"
            "\ts - save current cam to file 1\n"
            "\tt - save current cam to file 2\n"
    		<< endl;
}


void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{

}

void Error(const char * msg) {
	fprintf(stderr, "%s", msg);
}

bool ProcessKeys() {

    char c = (char)waitKey(30);
    if( c == 27 ) {

        return false;
    }

    switch( c )
    {
    case 'c':
    case 'd':
    	// capture from cam
		{
			int camNum = (c=='c') ? 0 : 1;
			bool res = cap.open(camNum);
			if(res) {
				printf("Init capture from cam %d - ok\n", camNum);
			}
			else {
				printf("Init capture from cam %d - failed %s\n", camNum, cvErrorStr(cvGetErrStatus()));
			}

			cap.set(CV_CAP_PROP_CONVERT_RGB, 1.);
			IsCam = true;
		}
        break;
    case 'f':
    case 'g':
		{
			const char *fname = IoFiles[c-'f'];

			bool res = cap.open(fname);
			if(res) {
				printf("Init read from file %s - ok\n", fname);
			}
			else {
				printf("Init read from file %s - failed %s\n", fname, cvErrorStr(cvGetErrStatus()));
			}
			IsCam = false;
		}
        break;
    case 's':
    case 't':
		{
			if(IoFiles[c-'s'] == NULL) {
				Error("No file defined\n");
				break;
			}

			if(!Vwrite.open(IoFiles[c-'s'], CV_FOURCC('I', 'Y', 'U', 'V'),
					FRAMES_SECOND, LastImageSize, true)) {
				Error("Could not capture to file\n");
				break;
			}
		}
        break;
    default:
        break;
    }

    return true;
}



void ProcessOptions(int argc, char** argv ) {
	for(int i=1; i<argc; i++) {
		if(argv[i] == strstr(argv[i], "-f") && (i+1) < argc) {
			IoFiles[argv[i][2] - '1'] = argv[i+1];
			i++;
		}
	}
}

void ShowMsg(const char *msg) {
	  IplImage *img = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 1 );
	  CvFont font;
	  double hScale = 1.0;
	  double vScale = 1.0;
	  int lineWidth = 1;
	  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,
	              hScale, vScale, 0, lineWidth );
	  cvPutText( img, msg, cvPoint( 200, 400 ), &font,
	             cvScalar( 255, 255, 0 ) );
	  cvShowImage( WindowName, img );
}

double getTimeUsecs() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}


int main( int argc, char** argv )
{
	ProcessOptions(argc, argv);

	cvRedirectError(cvStdErrReport);

    namedWindow( WindowName, 1 );
//    setMouseCallback( "LK Demo", onMouse, 0 );

    Mat image;
    Mat rsImage;
    ShowMsg("Waiting...");
    
    const int FIND=0;
	const int ACQUIRE=1;
	const int TRACK=2;

	const int fontFace = FONT_HERSHEY_PLAIN;
	const double fontScale = 1.5;
	const int thickness=1;

    const int MAX_COUNT = 500;
    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    int state = FIND;
    vector<Point2f> trackPoints[2];
    vector<uchar> trackStatus;

    Mat prevGrey;
	Mat grey;
	map<unsigned int, TrackedPoint> trackedPoints;

	map<unsigned int, TrackedGroup> trackedGroups;
    int largestTrackedGroup = -1;

    double totalTime=0;
    double findFeatureTime=0;
    double lkTime=0;
    double featureProcessTime=0;
    int nframes = 0;
    for(;;)
    {
    	if(cap.isOpened()) {

    		nframes++;

			Mat frame;
			cap >> frame;
			if( frame.empty() )
				break;	// TODO: handle properly

			if(IsCam) {
				cv::resize(frame, rsImage, Size(), 0.25, 0.25);
			}
			else {
				frame.copyTo(rsImage);
			}

			cvtColor(rsImage, grey, CV_BGR2GRAY);


			// Image processing here
			if(state==FIND) {
				printf("State=FIND, clear tracked groups\n");
				// No groups tracked
				trackedGroups.clear();
				largestTrackedGroup = -1;

				// Find points to track
				double tFindFeatureStart = getTimeUsecs();

				goodFeaturesToTrack(grey, trackPoints[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
	            cornerSubPix(grey, trackPoints[1], subPixWinSize, Size(-1,-1), termcrit);

	            findFeatureTime += getTimeUsecs() - tFindFeatureStart;


	            if(trackPoints[1].size() > 0) {
	            	state = ACQUIRE;
	            }
			} else if(state==ACQUIRE || state==TRACK) {
				// Got some points, do a Lukas-Kanade
				printf("State=%d, Lukas-Kanade\n", state);

	            vector<float> err;
	            if(prevGrey.empty())
	                grey.copyTo(prevGrey);

	            double tLkStart = getTimeUsecs();

	            calcOpticalFlowPyrLK(prevGrey, grey, trackPoints[0], trackPoints[1], trackStatus, err, winSize,
	                                 3, termcrit, 0, 0.001);

	            lkTime += getTimeUsecs() - tLkStart;

	            double tFeatureProcessStart = getTimeUsecs();

	            // Calculate velocities for tracked points
	            for(unsigned int n=0; n<trackStatus.size(); n++) {
	            	if(trackStatus[n]) {
	            		Point2f dp = Point2f(trackPoints[1][n].x - trackPoints[0][n].x, trackPoints[1][n].y - trackPoints[0][n].y);

	            		if(trackedPoints.count(n)) {
        					TrackedPoint &tp = trackedPoints[n];

        					tp.pos = trackPoints[1][n];
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
        					newtp.pos = trackPoints[1][n];

	            			trackedPoints[n] = newtp;
	            		}

	            		TrackedPoint &dtp = trackedPoints[n];
//	            		printf("Tracked point %d: (obj %d, moving %d [%f,%f], hf=%d) (%f,%f)\n",
//	            				n, dtp.objectId, dtp.IsMoving(), dtp.averageVel.x, dtp.averageVel.y,
//	            				dtp.histFill,
//	            				trackPoints[1][n].x, trackPoints[1][n].y);

	            	}
	            }

	            // Identify (new) objects
	            bool averageDone = false;
	            for(map<unsigned int, TrackedPoint>::iterator it=trackedPoints.begin(); it != trackedPoints.end(); it++) {

	            	TrackedPoint &tp = (*it).second;

	            	averageDone |= tp.HasValidAverage();

	            	if(tp.objectId < 0 && tp.IsMoving()) {
	            		// Not bound to an object and is moving

	            		for(map<unsigned int, TrackedPoint>::iterator iit=trackedPoints.begin(); iit != trackedPoints.end(); iit++) {
	            			TrackedPoint &itp = (*iit).second;
	            			if(itp.objectId >= 0 && itp.IsMoving()) {
	            				if(itp.CalcVelDiff(tp) < GROUP_THRESH) {
	            					// We are in the same object
	            					tp.objectId = itp.objectId;
	    	            			printf("Found object %d for point %d (%f,%f) vel (%f,%f)\n",
	    	            					tp.objectId, (*it).first, tp.pos.x, tp.pos.y, tp.averageVel.x, tp.averageVel.y);
	            					break;
	            				}
	            			}
	            		}

	            		if(tp.objectId < 0) {
	            			// No connected object found
	            			static int nextObjectId = 0;
	            			tp.objectId = nextObjectId++;
	            			printf("Added new object %d for point %d (%f,%f) vel (%f,%f)\n\n",
	            					tp.objectId, (*it).first, tp.pos.x, tp.pos.y, tp.averageVel.x, tp.averageVel.y);
	            		}
	            	}
	            }

	            // zero point counters and sum
	            for(map<unsigned int, TrackedGroup>::iterator ig= trackedGroups.begin(); ig != trackedGroups.end(); ig++) {
	            	TrackedGroup &tg = (*ig).second;
	            	tg.nPoints = 0;
	            	tg.sum = Point2f(0,0);
	            	tg.max = Point2f(-INFINITY,-INFINITY);
	            	tg.min = Point2f(INFINITY,INFINITY);
	            }

	            for(map<unsigned int, TrackedPoint>::iterator it=trackedPoints.begin(); it != trackedPoints.end(); it++) {
					TrackedPoint &tp = (*it).second;
					if(tp.objectId >= 0 && tp.IsMoving()) {
						if(trackedGroups.count(tp.objectId)) {
							TrackedGroup &tg = trackedGroups[tp.objectId];
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
							trackedGroups[tp.objectId] = newtg;
						}
					}
	            }

	            unsigned int npMax = 0;
	            for(map<unsigned int, TrackedGroup>::iterator ig= trackedGroups.begin(); ig != trackedGroups.end(); ig++) {
	            	TrackedGroup &tg = (*ig).second;
	            	tg.centre.x = tg.sum.x / tg.nPoints;
	            	tg.centre.y = tg.sum.y / tg.nPoints;

	            	if(tg.nPoints > npMax) {
	            		npMax = tg.nPoints;
	            		largestTrackedGroup = (*ig).first;
	            	}
	            	// printf("  Tracked group %d: np=%d, c=%f,%f\n", (*ig).first, tg.nPoints, tg.centre.x, tg.centre.y);
	            }


	            if(trackedGroups.size() > 0) {
	            	printf("%d tracked groups; state=>TRACK\n", (int)trackedGroups.size());
	            	state = TRACK;
	            }
	            else if(averageDone) {
	            	printf("Zero tracked groups after average, state=>FIND\n");
	            	state = FIND;	// reset the state if we have averaged points but nothing tracked
	            }
	            else {
	            	printf("Averaging, state=%d\n", state);
	            }

	            featureProcessTime += getTimeUsecs() - tFeatureProcessStart;
			}

			// debug
			putText(rsImage, state==0 ? "Find" : (state==1 ? "Acquire" : "Track"),
					Point(16,32), fontFace, fontScale,
			        Scalar::all(255), thickness, 8);

			if(largestTrackedGroup >= 0) {
				TrackedGroup &tg = trackedGroups[largestTrackedGroup];
	            printf("Largest tracked group %d, np=%d, c=%f,%f, mm=%f,%f, gs=%d,%d\n", largestTrackedGroup, tg.nPoints,
	            		tg.centre.x, tg.centre.y, tg.minMaxMean().x, tg.minMaxMean().y, tg.groupSize().width, tg.groupSize().height);
				ellipse(rsImage, tg.centre, Size(40,40), 0., 0., M_PI, Scalar(255,200,200), 4, 8, 0);
			}

            for(unsigned int i = 0; i < trackPoints[1].size(); i++ ) {
            	Scalar pointColour;
            	if((state==ACQUIRE || state==TRACK) && trackStatus.size() > i && trackStatus[i]) {
            		if(trackedPoints.count(i)) {
            			TrackedPoint &tp = trackedPoints[i];
            			if(tp.IsMoving()) {
                			pointColour = Scalar(0,255,0);
                        	// circle( rsImage, trackPoints[1][i], 3, pointColour, -1, 8);
            			}
            			else {
            				pointColour = Scalar(255,0,0);
            			}
            		}
            		else {
            			pointColour = Scalar(128,0,0);
            		}
				}
            	else {
            		 pointColour = Scalar(0, 0, 255);
            	}

            }

            std::swap(trackPoints[1], trackPoints[0]);
            std::swap(prevGrey, grey);

			imshow( WindowName, rsImage);

			// LastImageSize = cvSize((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
			LastImageSize = rsImage.size();

			if(Vwrite.isOpened()) {
				 Vwrite << rsImage;
			}

    	}


        if(!ProcessKeys())
        	break;
        
    }

    totalTime = findFeatureTime + lkTime + featureProcessTime;

	double avgSecs = totalTime / nframes;
	double avgFindFeatureSecs = findFeatureTime / nframes;
	double avgLkSecs = lkTime / nframes;
	double avgFeatureProcessTime = featureProcessTime / nframes;

	printf("%d frames, total time=%f, find feature time=%f, lk time=%f, feature process time=%f\n",
			nframes, totalTime, findFeatureTime, lkTime, featureProcessTime);
	printf("average (frame) time=%f, avg find feature time=%f, avg lk time=%f, avg feature process time=%f\n",
			avgSecs, avgFindFeatureSecs, avgLkSecs, avgFeatureProcessTime);

    return 0;
}
