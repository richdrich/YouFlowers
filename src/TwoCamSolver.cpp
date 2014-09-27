/*
 * TwoCamSolver.cpp
 *
 *  Created on: Oct 8, 2012
 *      Author: richard
 */

// using namespace std;

#include <stdio.h>
#include <vector>
#include "lop.h"
#include "solve.h"
#include "TwoCamSolver.h"

Solve TwoCamSolver::solve(bool cam1valid, float cam1x, float cam1y, int cam1width, int cam1height,
		bool cam2valid, float cam2x, float cam2y, int cam2width, int cam2height) {
	float inpx[2] = {cam1x, cam2x};
	int widthPixels[2] = {cam1width, cam2width};
	bool camValid[2] = {cam1valid, cam2valid};
	// float inpy[2] = {cam1x, cam2y};

	vector<Lop> lops;
	for(unsigned int n=0; n<2; n++) {
		if(camValid[n]) {
			// The bearing lop
			float thetaDegrees = cams[n].azimuthDegrees
										-  (inpx[n] - widthPixels[n]/2) * cams[n].degreesPerPixel;
			Lop blop(cams[n].x, cams[n].y, thetaDegrees * M_PI / 180., 1.0f);
			// Lop::LineEnds le = blop.GetEnds(5000,5000);
			//printf("%d: input pixel %f => %f => (%f,%f)->(%f,%f)\n",
			//		n, inpx[n], thetaDegrees,
			//		le.sx,le.sy,le.ex,le.ey);
			lops.push_back(blop);
		}

		// TODO: floor intercept
	}

	if(lops.size() < 2) {
		int validCam = camValid[0] ? 0 : (camValid[1] ? 1 : -1);
		if(validCam >= 0) {

			// Set the alternate LOP based on existing x,y
			float recDegrees = 90 + cams[validCam].azimuthDegrees
						-  (inpx[validCam] - widthPixels[validCam]/2) * cams[validCam].degreesPerPixel;
			if(recDegrees > 360.0) recDegrees -= 360.0;

			Lop blop(posX, posY, recDegrees * M_PI / 180., 1.0f);
			// Lop::LineEnds le = blop.GetEnds(5000,5000);
			//printf("%d: alt LOP => %f => (%f,%f)->(%f,%f)\n",
			//		validCam, recDegrees,
			//		le.sx,le.sy,le.ex,le.ey);
			lops.push_back(blop);
		}
	}

	// Existing lops
#ifdef USE_EXISTING
	Lop xLop(posX, 0.0f, (float)(M_PI/2), 2.0f);
	Lop yLop(0.0f, posY, 0.0f, 2.0f);
	lops.push_back(xLop);
	lops.push_back(yLop);
#endif

	Solve solver(lops);

	// Move towards the centre at a maximum of 50 pixels per iteration
	posX += fmin(50.0, solver.x - posX);
	posY += fmin(50.0, solver.y - posY);

	return solver;
}


