#ifndef TIMINGS_H_
#define TIMINGS_H_

#include <stdio.h>
#include <sys/time.h>
#include <string>

using namespace std;

class Timings {
public:
	static double getTimeUsecs() {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
	}

	Timings() {
		NFrames = 0;
		FindFeatureTime = 0;
		TrackTime = 0;
		FeatureProcessTime = 0;
		GrabTime = 0;
		RetrieveTime = 0;
		DebugTime = 0;
	}

	string ProcessResults() {
	    double totalTime = FindFeatureTime + TrackTime + FeatureProcessTime + RetrieveTime + GrabTime + DebugTime;

		double avgSecs = totalTime / NFrames;
		double avgRunSecs = RunTime / NFrames;
		double avgFindFeatureSecs = FindFeatureTime / NFrames;
		double avgLkSecs = TrackTime / NFrames;
		double avgFeatureProcessTime = FeatureProcessTime / NFrames;
		double avgGrabTime = GrabTime / NFrames;
		double avgRetrieveTime = RetrieveTime / NFrames;
		double avgDebugTime = DebugTime / NFrames;

		char buf[500];
		snprintf(buf, 500, "%d frames, total time=%0.3f, measured time=%0.3f, find feature time=%0.3f, track time=%0.3f, feature process time=%0.3f\n",
				NFrames, RunTime, totalTime, FindFeatureTime, TrackTime, FeatureProcessTime);
		string res = string(buf);

		snprintf(buf, 500, "average (run) time=%0.3f, average (measured) time=%0.3f, avg find feature time=%0.3f, avg track time=%0.3f, avg feature process time=%0.3f\n",
				avgRunSecs, avgSecs, avgFindFeatureSecs, avgLkSecs, avgFeatureProcessTime);
		res = res + string(buf);

		snprintf(buf, 500, "average grab time=%0.3f, avg retrieve time=%0.3f, avg debug time=%0.3f\n",
				avgGrabTime, avgRetrieveTime, avgDebugTime);
		return res + string(buf);
	}

	int NFrames;
	double FindFeatureTime;
	double TrackTime;
	double FeatureProcessTime;
	double GrabTime;
	double RetrieveTime;
	double DebugTime;
	double RunTime;
};


#endif
