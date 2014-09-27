
#ifndef ITRACKER
#define ITRACKER

class TrackedPoint;
class Timings;

#define NUMTYPE float
#define POINTTYPE Point2f

class ITracker {
public:
    static const int FIND=0;
	static const int ACQUIRE=1;
	static const int TRACK=2;

	virtual ~ITracker() {};

	class Result {
	public:
		Result() {
			pObjectPos = NULL;
			pAllPoints = NULL;
			pPointStatus = NULL;
			pTrackedPoints = NULL;
		}

		POINTTYPE *pObjectPos;
		int State;
		vector<POINTTYPE> *pAllPoints;
		vector<uchar> *pPointStatus;
		map<unsigned int, TrackedPoint> *pTrackedPoints;
//		TrackedGroup ObjectGroup;
	};

	virtual Result trackNewImage(const cv::Mat &image, Timings &timings)=0;

protected:
	int TrackNum;

};


#endif
