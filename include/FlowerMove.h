#ifndef FLOWERMOVE_H
#define FLOWERMOVE_H

#include "ardServo.h"
#include <map>

using namespace std;

class FlowerMove {
public:



	FlowerMove(vector<FlowerDef> &flowers) {
		if(pInstance==NULL) {
			pInstance = this;
		}

		Flowers = flowers;

		if(!initServo()) {
			fprintf(stderr, "Servo init failed\n");
			return;
		}

		centreServo();

		pthread_t worker;
		pthread_attr_t attr;
		pthread_attr_init(&attr);

		pthread_attr_setstacksize (&attr, 100000L);

		Finished = false;
		int res = pthread_create(&worker, &attr, moveWorker, NULL);
		if(res) {
			fprintf(stderr, "pthread_create moveWorker returned %d\n", res);
		}
		else {
			fprintf(stderr, "pthread_create moveWorker ok\n");
		}
	}

	virtual ~FlowerMove() {
		Finished = true;
	}

	void Move(int x, int y) {
		int fi=0;
		for(vector<FlowerDef>::iterator it = Flowers.begin(); it != Flowers.end(); ++it, fi++) {
			// fprintf(stderr, "Move %d,%d => %d,%d\n",it->x,it->y, x, y);
			int angleTo = (int)(atan2( y - it->y, x - it->x) * 180.0 / M_PI);
			if(angleTo < 0) {
				angleTo += 360;
			}

			SetAngle(fi, angleTo);
		}
	}

	void MoveAngle(int angle) {
		for(unsigned int fi=0; fi<Flowers.size(); fi++) {
			SetAngle(fi, angle);
		}
	}

	void debugToImage(Mat target, float xorg, float yorg, float xscale, float yscale) {
		int fi=0;
		for(vector<FlowerDef>::iterator it = Flowers.begin(); it != Flowers.end(); ++it, fi++) {
			float sx = xorg + it->x * xscale;
			float sy = yorg + it->y * yscale;
			cv::rectangle(target, Point(sx-4, sy-4), Point(sx+4,sy+4), Scalar(255,0,0), 2);
			float ex = sx + cos(RawAngles[fi] * M_PI / 180) * 40.0;
			float ey = sy - sin(RawAngles[fi] * M_PI / 180) * 40.0;
			cv::line(target, Point(sx,sy), Point(ex,ey), Scalar(255,0,0));
		}
	}

private:

	void SetAngle(unsigned int fi, int angleTo) {
		RawAngles[fi] = angleTo;

		int rotAngle = (angleTo - Flowers[fi].az) + Flowers[fi].range / 2;
		// fprintf(stderr, "%d: angleTo=%d, rotAngle=%d\n", fi, angleTo, rotAngle);

		if(rotAngle < 0) {
			rotAngle = 0;
		}
		else if(rotAngle > Flowers[fi].range) {
			rotAngle = Flowers[fi].range;
		}

		if(Angles.count(fi)==0 || Angles[fi] != rotAngle) {
			Angles[fi] = rotAngle;
			AngleChanged[fi] = true;
		}
	}


	static void *moveWorker(void *threadid) {
		while(!pInstance->Finished) {
			for(map<int, bool>::iterator it = pInstance->AngleChanged.begin(); it != pInstance->AngleChanged.end(); ++it) {
				if((*it).second) {
					// fprintf(stderr, "Servo: %d => %d\n", (*it).first, pInstance->Angles[(*it).first]);
					sendServo( (*it).first, pInstance->Angles[(*it).first]);
					(*it).second = false;

					usleep(200000L);
				}
			}

			usleep(20000L);
		}

		pthread_exit(NULL);
		return NULL;
	}

	static FlowerMove *pInstance;

	map<int, int> Angles;
	map<int, int> RawAngles;
	map<int, bool> AngleChanged;
	vector<FlowerDef> Flowers;
	bool Finished;
};

FlowerMove * FlowerMove::pInstance = NULL;

#endif
