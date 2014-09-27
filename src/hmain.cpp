//
// Youflower / OcvHarness main app
// Reads 1 or 2 cameras and drives N flower servos over USB

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <fstream>
#include <queue>
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/time.h>
#include <boost/program_options.hpp>

#include "TrackParams.h"
#include "TrackedPoint.h"
#include "TrackedGroup.h"
#include "FlowerDef.h"
#include "FlowerMove.h"

// QuickTracker enables a faster tracking algorithm that doesn't really work...
#ifdef QUICKTRACKER
#include "QuickTracker.h"
#else
#include "Tracker.h"
#endif

#include "Timings.h"
#include "TwoCamSolver.h"
#include "lop.h"

using namespace cv;
using namespace std;
namespace po = boost::program_options;

#ifdef MAC
CvCapture* cvCreateCameraCapture_QT( int index );
#else
CvCapture* cvCreateCameraCapture_V4L_Cust( int index );
#endif

CvCapture * cap[2] = {NULL, NULL};
const char *WindowName = "CV Harness";
string IoFiles[2];
VideoWriter Vwrite[2];
VideoCapture Reader[2];
int FrameRate=0;

int Samples=8;
int Limit=40;
int Edge=20;
int TermCount = 10;

//CvSize LastImageSize;
bool IsCam;
bool paused = false;
bool stereo = false;
bool processing = true;
bool display = true;
bool angletrack = false;
int channel = 0;
int startupMode = 0;	// Startup command - controlled by keys and initialize
const int START_PLAY = 1;
const int START_CAM = 2;
const int START_RECORD = 4;

int DefaultWidths[2] = {0,0};
int DefaultHeights[2] = {0,0};

bool finished = false;
Mat capturedImages[2];
int imageCounts[2] = {0,0};
pthread_mutex_t imageMutex[2] = {PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER};

Timings timings[2];

// Frame rate, overridable
int ProcessFrameRate = 10;

// Define the arbitrary workspace (used for the display)
// set in geometry
float AreaWidth = 5000;
float AreaDepth = 5000;

// The LOP solver
TwoCamSolver Solver; // (TwoCamSolver::CamDef(0.0f, 1790.0f,1720.0f, 30.0f, 0.0f, 48.0f/640.0f, 640,480),
		// TwoCamSolver::CamDef(1510.0f, 1720.0f+1270.0f, 1000.0f, -10.0f, 270.0f, 51.0f/640.0f, 640, 480));

// The moving flowers
vector<FlowerDef> Flowers;
FlowerMove *pFlowerMove;

// Flag an error
void Error(const char * msg) {
	fprintf(stderr, "%s", msg);
}

// Initialize write to a file for a video channel 'chan'
bool InitWrite(int chan) {
	if(IoFiles[chan].empty()) {
		Error("No file defined\n");
		return false;
	}

	if(!Vwrite[chan].open(IoFiles[chan], CV_FOURCC(0,0,0,0),
			FRAMES_SECOND, Size(640,480), true)) {
		Error("Could not capture to file\n");
		return false;
	}

	fprintf(stdout, "Capture started to file %s\n", IoFiles[chan].c_str());
	return true;
}

void FindCameras() {
	int camSelect = 0;
	int camCount = 0;

	for(int camNum=0; camNum<2; camNum++) {
		CvCapture *newCap;

		while(true) {
			printf("Try camera (%d)\n", camSelect);
			newCap = cvCreateCameraCapture_QT(camSelect);
			if(newCap==NULL) {
				printf("camera (%d) not found\n", camSelect);
				break;
			}

			int width = cvGetCaptureProperty(newCap, CV_CAP_PROP_FRAME_WIDTH);
			int height = cvGetCaptureProperty(newCap, CV_CAP_PROP_FRAME_HEIGHT);
			printf("Camera index %d, width %d, height %d\n", camSelect, width, height);

			if((DefaultWidths[camCount]==0 || DefaultWidths[camCount]==width) ||
					(DefaultHeights[camCount]==0 || DefaultHeights[camCount]==height)) {

				if(camCount>=camNum) {
					printf("%d matching selected for %d\n", camCount, camNum);
					cap[camNum] = newCap;

					// Frame size and rate is hardcoded here ??
					cvSetCaptureProperty(newCap, CV_CAP_PROP_FRAME_WIDTH, 320.);
					cvSetCaptureProperty(newCap, CV_CAP_PROP_FRAME_HEIGHT, 240.);
					cvSetCaptureProperty(newCap, CV_CAP_PROP_FPS, 15.);
					camSelect++;
					camCount++;
					break;
				}
				printf("Skipped camera %d (%d)\n", camCount, camSelect);
				camCount++;
			}
			else {
				printf("Ignored camera %d (%d), wrong size\n", camCount, camSelect);
				newCap = NULL;
			}

			camSelect++;
		}
	}
}

// Initialize camera 'camNum'
bool InitCam(int camNum) {
	IsCam = cap[camNum] != NULL;
	return IsCam;
}

// Open a file for read
bool OpenRead(int chan, const char * fname) {
	bool res = Reader[chan].open(fname);
	if(res) {
		printf("Init read from file %s - ok\n", fname);
	}
	else {
		printf("Init read from file %s - failed %s\n", fname, cvErrorStr(cvGetErrStatus()));

		return false;
	}

	IsCam = false;

	int fileFrameRate = Reader[chan].get(CV_CAP_PROP_FPS);
	if(fileFrameRate==0) {
		fprintf(stderr, "File %s has no frome rate\n", fname);
	}

	if(FrameRate!=0) {
		if(FrameRate != fileFrameRate) {
			fprintf(stderr, "File %s has incompatible frome rate (%d) with other channel %d\n", fname, fileFrameRate, FrameRate);
			return false;
		}
	}


	FrameRate = fileFrameRate;

	printf("Read from file %s frame rate %d\n", fname, FrameRate);

	return res;
}

// Startup playback/record/capture as configured
bool Startup() {

	if(startupMode & START_PLAY) {
		if(stereo) {
			if(!OpenRead(0, IoFiles[0].c_str())) {
				return false;
			}
			if(!OpenRead(1, IoFiles[1].c_str())) {
				return false;
			}
		}
		else {
			if(OpenRead(channel, IoFiles[channel].c_str())) {
				return false;
			}
		}
	}


	if(startupMode & START_CAM) {
		if(stereo) {
			printf("stereo camera start on boot\n");
			if(!InitCam(0)) {
				return false;
			}
			if(!InitCam(1)) {
				return false;
			}
		}
		else {
			printf("Camera start on boot, channel %d\n", channel);
			if(!InitCam(channel)) {
				return false;
			}
		}
	}

	if(startupMode & START_RECORD) {
		if(stereo) {
			printf("stereo record start on boot\n");
			if(!InitWrite(0)) {
				return false;
			}
			if(!InitWrite(1)) {
				return false;
			}
		}
		else {
			printf("Record start on boot, channel %d\n", channel);
			if(!InitWrite(channel)) {
				return false;
			}
		}
	}

	return true;
}

// process command keys
// c,d start cameras (both on either key if stereo is enabled)
bool ProcessKeys() {

	int waitDelay = IsCam ? (1000/ProcessFrameRate) : (1000/((FrameRate==0) ? 10 : FrameRate));

	// fprintf(stderr, "Wait up to %d ms for keys\n", waitDelay);
    char c = (char)waitKey(waitDelay);
    if( c == 27 ) {

        return false;
    }

    switch( c )
    {
    case 'c':
    case 'd':
    	// capture from cam
		{
			if(stereo) {
				InitCam(1);
				InitCam(0);
			}
			else {
				InitCam((c=='c') ? 0 : 1);
			}
			paused = false;
		}
        break;

    case 'f':
    case 'g':
		{
			if(stereo) {
				OpenRead(0, IoFiles[0].c_str());
				OpenRead(1, IoFiles[1].c_str());
			}
			else {
				const char *fname = IoFiles[c-'f'].c_str();
				OpenRead(c-'f', fname);
			}
			IsCam = false;
			paused = false;
		}
        break;

    case 's':
    case 't':
		{
			if(stereo) {
				if(!InitWrite(0)) break;
				if(!InitWrite(1)) break;
			}
			else {
				if(!InitWrite(c-'s')) break;
			}
		}
        break;
    case ' ':
    	paused = !paused;
    	break;
    default:
        break;
    }

    return true;
}

// Process command line and file options
bool ProcessOptions(int argc, char** argv ) {
	// Using boost, declare the supported options.
	po::options_description command_line_only_options;
	command_line_only_options.add_options()
	    ("help,h", "produce help message")
	    ("config,@", po::value<string>(), "specify config file");

	po::options_description file_only_options;

	po::options_description generic_options;
	generic_options.add_options()
	    	    ("stereo,s",  "enable stereo cameras")
	    	    ("nostereo",  "disable stereo cameras")
	    	    ("processing,p",  "enable image tracker processing")
	    	    ("noprocessing,n",  "disable image tracker processing")
	    	    ("display,d",  "enable image tracker processing")
	    	    ("nodisplay",  "disable image tracker processing")
	    	    ("file1,f", po::value<string>(), "specify file 1")
	    	    ("file2,g", po::value<string>(), "specify file 2")
	    	    ("channel,c", po::value<int>(), "specify channel 0/1")
	    	    ("play,l", "play files on startup")
	    	    ("record,r", "record files on startup")
	    	    ("sample,e", "sample on startup")
	    	    ("angletrack,a", "track on angles with single camera");

	file_only_options.add_options()
			("process.framerate", po::value<int>(), "frame rate when processing from camera")
			("geometry.width", po::value<int>(), "width of the tracking area")
			("geometry.depth", po::value<int>(), "depth of the tracking area")
			("cam.0.width", po::value<int>(), "default width of camera 0")
			("cam.0.height", po::value<int>(), "default height of camera 0")
			("cam.1.width", po::value<int>(), "default width of camera 1")
			("cam.1.height", po::value<int>(), "default height of camera 1")
			("geometry.cam.0.x", po::value<float>(), "Camera X position")
			("geometry.cam.0.y", po::value<float>(), "Camera Y position")
			("geometry.cam.0.z", po::value<float>(), "Camera Z position")
			("geometry.cam.0.dd", po::value<float>(), "Camera declination (degrees)")
			("geometry.cam.0.ad", po::value<float>(), "Camera azimuth (degrees)")
			("geometry.cam.0.dp", po::value<float>(), "Camera degrees per pixel")
			("geometry.cam.0.wp", po::value<float>(), "Camera frame width")
			("geometry.cam.0.hp", po::value<float>(), "Camera frame height")
			("geometry.cam.1.x", po::value<float>(), "Camera X position")
			("geometry.cam.1.y", po::value<float>(), "Camera Y position")
			("geometry.cam.1.z", po::value<float>(), "Camera Z position")
			("geometry.cam.1.dd", po::value<float>(), "Camera declination (degrees)")
			("geometry.cam.1.ad", po::value<float>(), "Camera azimuth (degrees)")
			("geometry.cam.1.dp", po::value<float>(), "Camera degrees per pixel")
			("geometry.cam.1.wp", po::value<float>(), "Camera frame width")
			("geometry.cam.1.hp", po::value<float>(), "Camera frame height")
			("quicktracker.samples", po::value<int>(), "Number of samples to average")
			("quicktracker.limit", po::value<int>(), "Threshold limit")
			("quicktracker.termcount", po::value<int>(), "Hill find termination count")
			("quicktracker.edge", po::value<int>(), "Edge limit")
			("flower.0.x", po::value<int>(), "X pos of flower 0")
			("flower.0.y", po::value<int>(), "Y pos of flower 0")
			("flower.0.az", po::value<int>(), "Flower 0 azimuth (degrees)")
			("flower.0.range", po::value<int>(), "Flower 0 range (degrees)")
			("flower.1.x", po::value<int>(), "X pos of flower 1")
			("flower.1.y", po::value<int>(), "Y pos of flower 1")
			("flower.1.az", po::value<int>(), "Flower 1 azimuth (degrees)")
			("flower.1.range", po::value<int>(), "Flower 1 range (degrees)")
			("flower.2.x", po::value<int>(), "X pos of flower 2")
			("flower.2.y", po::value<int>(), "Y pos of flower 2")
			("flower.2.az", po::value<int>(), "Flower 2 azimuth (degrees)")
			("flower.2.range", po::value<int>(), "Flower 2 range (degrees)")
			("flower.3.x", po::value<int>(), "X pos of flower 3")
			("flower.3.y", po::value<int>(), "Y pos of flower 3")
			("flower.3.az", po::value<int>(), "Flower 3 azimuth (degrees)")
			("flower.3.range", po::value<int>(), "Flower 3 range (degrees)")
			("flower.4.x", po::value<int>(), "X pos of flower 4")
			("flower.4.y", po::value<int>(), "Y pos of flower 4")
			("flower.4.az", po::value<int>(), "Flower 4 azimuth (degrees)")
			("flower.4.range", po::value<int>(), "Flower 4 range (degrees)")
			;


	po::options_description command_line_options("Command line options");
	command_line_options.add(command_line_only_options);
	command_line_options.add(generic_options);

	po::options_description file_options("Config file options");
	file_options.add(file_only_options);
	file_options.add(generic_options);


	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, command_line_options), vm);
	po::notify(vm);

	if(vm.count("config")) {
		const char * config_file = vm["config"].as<string>().c_str();

        ifstream ifs(config_file);
        if (!ifs)
        {
            cout << "can not open config file: " << config_file << "\n";
            return 0;
        }
        else
        {
            store(parse_config_file(ifs, file_only_options, true), vm);
            notify(vm);
        }
	}

	if (vm.count("help")) {
	    cout << command_line_options << "\n";
	    return false;
	}

	if (vm.count("stereo")) {
		stereo = true;
	}
	if (vm.count("nostereo")) {
		stereo = false;
	}

	if (vm.count("processing")) {
		processing = true;
	}
	if (vm.count("noprocessing")) {
		processing = false;
	}

	if (vm.count("display")) {
		display = true;
	}
	if (vm.count("nodisplay")) {
		display = false;
	}

	if(vm.count("angletrack")) {
		angletrack = true;
	}

	if (vm.count("file1")) {
		IoFiles[0] = vm["file1"].as<string>();
	}
	if (vm.count("file2")) {
		IoFiles[1] = vm["file2"].as<string>();
	}

	if (vm.count("channel")) {
		if(stereo) {
			fprintf(stderr, "Error: channel specified in stereo\n");
			return false;
		}
		channel = vm["channel"].as<int>();
	}

	if (vm.count("play")) {
		if (vm.count("sample")) {
			fprintf(stderr, "Error: both play and capture specified\n");
			return false;
		}
		startupMode |= START_PLAY;
	}
	if (vm.count("record")) {
		if (vm.count("play")) {
			fprintf(stderr, "Error: both play and record specified\n");
			return false;
		}
		startupMode |= START_RECORD;
	}
	if (vm.count("sample")) {
		startupMode |= START_CAM;
	}

	if (vm.count("process.framerate")) {
		ProcessFrameRate = vm["process.framerate"].as<int>();
	}

	if (vm.count("geometry.width")) {
		AreaWidth = vm["geometry.width"].as<int>();
	}
	if (vm.count("geometry.depth")) {
		AreaDepth = vm["geometry.depth"].as<int>();
	}

	if (vm.count("quicktracker.samples")) {
		Samples = vm["quicktracker.samples"].as<int>();
	}
	if (vm.count("quicktracker.limit")) {
		Limit = vm["quicktracker.limit"].as<int>();
	}
	if (vm.count("quicktracker.termcount")) {
		TermCount = vm["quicktracker.termcount"].as<int>();
	}
	if (vm.count("quicktracker.edge")) {
		Edge = vm["quicktracker.edge"].as<int>();
	}

	// Configure the cameras
	for(int camIdx=0; camIdx<2; camIdx++) {
		char name[64];
		snprintf(name, 63, "geometry.cam.%d.x", camIdx);
		if (vm.count(string(name))) {
			float x = vm[name].as<float>();

			float y=0.,z=0.,dd=0.,ad=0.,dp=0.;
			snprintf(name, 63, "geometry.cam.%d.y", camIdx);
			if (vm.count(name)) {
				y = vm[name].as<float>();
			}
			else {
				fprintf(stderr, "Camera %d has x specified but no %s\n", camIdx, name);
			}
			snprintf(name, 63, "geometry.cam.%d.z", camIdx);
			if (vm.count(name)) {
				z = vm[name].as<float>();
			}
			else {
				fprintf(stderr, "Camera %d has x specified but no %s\n", camIdx, name);
			}
			snprintf(name, 63, "geometry.cam.%d.dd", camIdx);
			if (vm.count(name)) {
				dd = vm[name].as<float>();
			}
			else {
				dd = 0;
			}
			snprintf(name, 63, "geometry.cam.%d.ad", camIdx);
			if (vm.count(name)) {
				ad = vm[name].as<float>();
			}
			else {
				ad = 0;
			}
			snprintf(name, 63, "geometry.cam.%d.dp", camIdx);
			if (vm.count(name)) {
				dp = vm[name].as<float>();
			}
			else {
				fprintf(stderr, "Camera %d has x specified but no %s\n", camIdx, name);
			}

			snprintf(name, 63, "cam.%d.width", camIdx);
			if (vm.count(name)) {
				DefaultWidths[camIdx] = vm[name].as<int>();
			}
			snprintf(name, 63, "cam.%d.height", camIdx);
			if (vm.count(name)) {
				DefaultHeights[camIdx] = vm[name].as<int>();
			}

			Solver.cams[camIdx] = TwoCamSolver::CamDef(x,y,z,dd,ad,dp);
		}
	}


	// and read flowers
	for(int floIdx=0; floIdx<5; floIdx++) {
		FlowerDef flower;
		char name[64];
		snprintf(name, 63, "flower.%d.x", floIdx);

		if (vm.count(name)) {
			flower.x = vm[name].as<int>();
		}
		else {
			break;
		}

		snprintf(name, 63, "flower.%d.y", floIdx);

		if (vm.count(name)) {
			flower.y = vm[name].as<int>();
		}
		else {
			fprintf(stderr, "Flower %d has x specified but no %s\n", floIdx, name);
		}

		snprintf(name, 63, "flower.%d.az", floIdx);

		if (vm.count(name)) {
			flower.az = vm[name].as<int>();
		}
		else {
			fprintf(stderr, "Flower %d has x specified but no %s\n", floIdx, name);
		}

		snprintf(name, 63, "flower.%d.range", floIdx);

		if (vm.count(name)) {
			flower.range = vm[name].as<int>();
		}
		else {
			fprintf(stderr, "Flower %d has x specified but no %s\n", floIdx, name);
		}

		fprintf(stderr, "Flower %d x=%d, y=%d", floIdx, flower.x, flower.y);

		Flowers.push_back(flower);
	}

	return true;

}

// Show a message on the screen
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

// Show the debug screen
void DebugShowImage(Mat *pImage1, Mat *pImage2, ITracker::Result *pRes1, ITracker::Result *pRes2, Point2f *pTrackPt, vector<Lop> *pLops) {
	const int fontFace = FONT_HERSHEY_PLAIN;
	const double fontScale = 1.5;
	const int thickness=1;

	if(!display) {
		return;
	}

	Mat *images[] = {pImage1, pImage2};
	ITracker::Result *results[] = {pRes1, pRes2};

	// Work out size for target based on max width and height
	int imgtype;
	int width = 0;
	int height = 0;
	for(int n=0; n<2; n++) {
		if(images[n]==NULL) {
			continue;
		}

		imgtype = images[n]->type();

		if(images[n]->cols > width) {
			width = images[n]->cols;
		}
		if(images[n]->rows > height) {
			height = images[n]->rows;
		}
	}

	Mat target(Size(width*3, height), imgtype);

	// process each capturing image
	for(int n=0; n<2; n++) {
		if(images[n]==NULL) {
			continue;
		}

		// If we are tracking an object draw it as a blob
		if(results[n] != NULL && results[n]->pObjectPos != NULL) {
//			printf("%d: Object group np=%d, c=%f,%f, mm=%f,%f, gs=%d,%d\n", n, results[n]->ObjectGroup.nPoints,
//					results[n]->ObjectGroup.centre.x, results[n]->ObjectGroup.centre.y,
//					results[n]->ObjectGroup.minMaxMean().x, results[n]->ObjectGroup.minMaxMean().y,
//					results[n]->ObjectGroup.groupSize().width, results[n]->ObjectGroup.groupSize().height);
			cv::ellipse(*(images[n]), *(results[n]->pObjectPos), Size(40,40), 0., 0., M_PI, Scalar(50,200,200), 20, 8, 0);
		}

		// Display the tracking status
		if(results[n] != NULL ) {
			putText(*(images[n]), results[n]->State==ITracker::FIND ? "Find" : (results[n]->State==ITracker::ACQUIRE ? "Acquire" : "Track"),
					Point(16,32), fontFace, fontScale,
					Scalar::all(255), thickness, 8);
		}

		// Enable the below to show the point swarm output by Lucas-Kanade
#ifdef SHOWPOINTSWARM
		if(results[n] != NULL && NULL != results[n]->pAllPoints) {

			for(unsigned int i = 0; i < results[n]->pAllPoints->size(); i++ ) {
				Scalar pointColour;
				if((results[n]->State==ITracker::ACQUIRE || results[n]->State==ITracker::TRACK) && results[n]->pPointStatus->size() > i && (*(results[n]->pPointStatus))[i]) {
					if(results[n]->pTrackedPoints->count(i)) {
						TrackedPoint &tp = (*(results[n]->pTrackedPoints))[i];
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
		}
#endif

		// Display the image with annotations from above
//		fprintf(stderr, "Debug display image %d, size %d,%d\n", n, images[n]->cols, images[n]->rows);
		Rect targetRect = Rect((n==0 ? 0 : 2*width), 0,width,height);
		Mat targetRoi = target(targetRect);
		images[n]->copyTo(targetRoi);

	}

	// Display the tracked point and LOPs
//	static bool newIm = true;
//	if(newIm) {
	float dispWidth = width - 20;
	float dispHeight = height - 20;
		cv::rectangle( target, Point(width,0), Point(width*2,height), Scalar(255,255,255), CV_FILLED);
//		newIm = false;
//	}
	if(pTrackPt != NULL) {
		Point2f plotPoint( width + 10 + pTrackPt->x * dispWidth / AreaWidth,
				dispHeight - (10 + pTrackPt->y * dispHeight / AreaDepth));
		cv::circle( target, plotPoint, 5, Scalar(128,0,128), -1);
	}
	if(pLops != NULL) {
		int lopNum=0;
		for(vector<Lop>::iterator lopit=pLops->begin(); lopit != pLops->end(); lopit++,lopNum++) {
			Lop::LineEnds ends = lopit->GetEnds(AreaWidth, AreaDepth);

			// printf("Draw lop %d: %f,%f => %f,%f\n",
			//		lopNum, ends.sx, ends.sy,
			//		width + 10 + ends.sx * dispWidth / AreaWidth,
			//		dispHeight - (10 + ends.sy * dispHeight / AreaDepth));
			Point2f p1(width + 10 + ends.sx * dispWidth / AreaWidth,
					dispHeight - (10 + ends.sy * dispHeight / AreaDepth));
			Point2f p2(width + 10 + ends.ex * dispWidth / AreaWidth,
					dispHeight - (10 + ends.ey * dispHeight / AreaDepth));
			cv::line(target, p1, p2, Scalar(0,0,0));
			char buf[10];
			snprintf(buf, 10, "%d", lopNum);
			putText(target, buf, p1, fontFace, fontScale,
					Scalar::all(0), thickness, 8);
		}
	}

	pFlowerMove->debugToImage(target, width+10, dispHeight-10, dispWidth / AreaWidth, -dispHeight / AreaDepth);

	imshow( WindowName, target);
}

// Image capture thread
// Captures images into buffer
void *captureWorker(void *threadid) {
	while(!finished) {
		if(cap[0]==NULL && cap[1]==NULL) {
			usleep(66000);
		}

		for(int camNum=0; camNum<2; camNum++) {
			if(cap[camNum] == NULL) continue;

			timings[camNum].NFrames++;

			// fputs("G\n", stderr);
			double gStart = Timings::getTimeUsecs();
			if(!cvGrabFrame(cap[camNum])) {
				fprintf(stderr, "cvGrabFrame failed\n");
			}
			timings[camNum].GrabTime += Timings::getTimeUsecs() - gStart;
		}

		for(int camNum=0; camNum<2; camNum++) {
			if(cap[camNum] == NULL) continue;

			// fputs("R\n", stderr); fflush(stderr);
			pthread_mutex_lock(&imageMutex[camNum]);
			double rStart = Timings::getTimeUsecs();

			IplImage *newImage = cvRetrieveFrame(cap[camNum], 0);
			capturedImages[camNum] = Mat(newImage, true);

			imageCounts[camNum]++;
			timings[camNum].RetrieveTime = Timings::getTimeUsecs() - rStart;
			// fputs("F\n", stderr); fflush(stderr);
			pthread_mutex_unlock(&imageMutex[camNum]);

			if(Vwrite[camNum].isOpened()) {
				 Vwrite[camNum] << capturedImages[camNum];
			}

		}
	}

	pthread_exit(NULL);
	return NULL;
}

// Return true if image n has been captured
bool IsImageValid(int n) {
	if(IsCam) {
		return (cap[n] != NULL) && (imageCounts[n] > 0);
	}
	else {
		return Reader[n].isOpened();
	}
}

// Main command processor
int main( int argc, char** argv )
{
	if(!ProcessOptions(argc, argv)) {
		return 0;
	}

	cvRedirectError(cvStdErrReport);

	FindCameras();

	if(stereo) {
		printf("Stereo is enabled\n");
	}
	if(processing) {
		printf("Image processing is enabled\n");
	}

	// Control moving the flowers by servo
	pFlowerMove = new FlowerMove(Flowers);

    namedWindow( WindowName, 1 );

    Mat image;
    Mat rsImage[2];
    ShowMsg("Waiting...");

#ifdef QUICKTRACKER
    QuickTracker tr0(0, Samples, Limit, Edge, TermCount);
    QuickTracker tr1(1, Samples, Limit, Edge, TermCount);
#else
    Tracker tr0(0);
    Tracker tr1(1);
#endif

	ITracker *pTracker[2] = {&tr0, &tr1};
	ITracker::Result trackres[2];

	// Create the worker thread
	fprintf(stderr, "Create thread\n");
	finished = false;

	pthread_t worker;
	pthread_attr_t attr;
	pthread_attr_init(&attr);

	pthread_attr_setstacksize (&attr, 100000L);

	int res = pthread_create(&worker, &attr, captureWorker, NULL);
	if(res) {
		fprintf(stderr, "pthread_create returned %d\n", res);
	}
	else {
		fprintf(stderr, "pthread_create ok\n");
	}

	// The main foreground loop
	double runStart=0;
	bool started = false;

	bool fileFinished = false;
    while(!fileFinished)
    {
    	if(!started) {
    		Startup();
    		started = true;
    	}

    	if(!paused) {
			for(int camNum=0; camNum<2; camNum++) {
				if(IsCam) {
					// Capture from a camera
					if(cap[camNum] == NULL) {
						continue;
					}

					if(runStart==0) {
						runStart = Timings::getTimeUsecs();
					}

					// fprintf(stderr, "Wait for lock on %d\n", camNum);
					pthread_mutex_lock(&imageMutex[camNum]);
					// fputs("L\n",stderr); fflush(stderr);
					if(imageCounts[camNum]==0) {
						pthread_mutex_unlock(&imageMutex[camNum]);
						// fputs("Z\n", stderr); fflush(stderr);
						continue;
					}

					//fprintf(stderr, "Got image %d from %d\n", imageCounts[camNum], camNum); fflush(stderr);

					capturedImages[camNum].copyTo(rsImage[camNum]);
					pthread_mutex_unlock(&imageMutex[camNum]);
				}
				else {
					// From a file
					if(Reader[camNum].isOpened()) {
						Reader[camNum] >> rsImage[camNum];
						if(rsImage[camNum].empty()) {
							fileFinished = true;
							break;
						}

						imageCounts[camNum]++;
					}
				}

				// Track an image and collect the result
				if(processing && IsImageValid(camNum)) {
					trackres[camNum] = pTracker[camNum]->trackNewImage(rsImage[camNum], timings[camNum]);
				}

				timings[camNum].NFrames++;

			}


			double dStart = Timings::getTimeUsecs();
			if(IsImageValid(0) || IsImageValid(1)) {
				if(trackres[0].pObjectPos != NULL || trackres[1].pObjectPos != NULL) {
					if(angletrack) {
						// Just track by angle off a single camera (zero)
						int vci = (trackres[0].pObjectPos != NULL) ? 0 : 1;

						float angle = Solver.cams[vci].degreesPerPixel
								* (rsImage[vci].cols/2 - trackres[vci].pObjectPos->x)
							+ Solver.cams[vci].azimuthDegrees;

						// fprintf(stderr, "Rotate pixels=%f, angle=%f\n", trackres[vci].pObjectPos->x, angle);

						pFlowerMove->MoveAngle((int)angle);
					}
					else {
						// Solve and show lops
						Solve slv = Solver.solve(trackres[0].pObjectPos != NULL,
								trackres[0].pObjectPos != NULL ? trackres[0].pObjectPos->x : 0.0f,
								trackres[0].pObjectPos != NULL ? trackres[0].pObjectPos->y : 0.0f,
								IsImageValid(0) ? rsImage[0].cols : 0,
								IsImageValid(0) ? rsImage[0].rows : 0,
								trackres[1].pObjectPos != NULL,
								trackres[1].pObjectPos != NULL ? trackres[1].pObjectPos->x : 0.0f,
								trackres[1].pObjectPos != NULL ? trackres[1].pObjectPos->y : 0.0f,
								IsImageValid(1) ? rsImage[1].cols : 0,
								IsImageValid(1) ? rsImage[1].rows : 0);

						// Move the flowers
						pFlowerMove->Move(slv.x, slv.y);

						// Show the image
						Point2f trackPt(slv.x,slv.y);
						DebugShowImage(IsImageValid(0) ? &rsImage[0] : NULL,
								IsImageValid(1) ? &rsImage[1] : NULL,
								IsImageValid(0) ? &trackres[0] : NULL,
								IsImageValid(1) ? &trackres[1] : NULL,
								&trackPt, &slv.lops);
					}
				}
				else {
					DebugShowImage(IsImageValid(0) ? &rsImage[0] : NULL,
						IsImageValid(1) ? &rsImage[1] : NULL,
						IsImageValid(0) ? &trackres[0] : NULL,
						IsImageValid(1) ? &trackres[1] : NULL, NULL, NULL);
				}
			}
			timings[0].DebugTime += Timings::getTimeUsecs() - dStart;
			timings[1].DebugTime = timings[0].DebugTime;
    	}


        if(!ProcessKeys())
        	break;
        
    }

    finished = true;
    sleep(3);

    timings[0].RunTime = Timings::getTimeUsecs() - runStart;
    timings[1].RunTime = timings[0].RunTime;

    puts(timings[0].ProcessResults().c_str());
    puts(timings[1].ProcessResults().c_str());

    delete pFlowerMove;
    return 0;
}
