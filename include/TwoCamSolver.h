/*
 * TwoCamSolver.h
 *
 *  Created on: Oct 8, 2012
 *      Author: richard
 */

#ifndef TWOCAMSOLVER_H_
#define TWOCAMSOLVER_H_

#include "solve.h"

class TwoCamSolver {
public:

	class CamDef {
	public:
		CamDef() {};

		CamDef(float x,float y,float z,float dd,float ad,float dp) {
			this->x = x;
			this->y = y;
			this->z = z;
			this->declinationDegrees = dd;
			this->azimuthDegrees = ad;
			this->degreesPerPixel = dp;
		}

		float x;
		float y;
		float z;
		float declinationDegrees;
		float azimuthDegrees;
		float degreesPerPixel;

	};

	TwoCamSolver() {
		posX = 1000;
		posY = 1000;
	}

	TwoCamSolver(CamDef cam1, CamDef cam2) {
		cams[0] = cam1;
		cams[1] = cam2;
	}

	virtual ~TwoCamSolver() {};

	Solve solve(bool cam1valid, float cam1x, float cam1y, int cam1width, int cam1height,
			bool cam2valid, float cam2x, float cam2y, int cam2width, int cam2height);


	CamDef cams[2];
	float posX;
	float posY;
};

#endif /* TWOCAMSOLVER_H_ */
