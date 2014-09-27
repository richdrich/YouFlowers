/*
 * solve.h
 *
 *  Created on: Sep 18, 2012
 *      Author: richard
 */

#ifndef SOLVE_H_
#define SOLVE_H_

#include "lop.h"

using namespace std;


class Solve {
public:
	Solve(vector<Lop> lops) {
		this->lops = lops;

		// sum all the coefficients
		vector<float> sigmaCoeffs(6);
		for(unsigned int n=0; n<6; n++) {
			sigmaCoeffs[n] = 0;
		}

		for(unsigned int li=0; li<lops.size(); li++) {
			vector<float> coeffs = lops[li].GetWeightedPolynomialE2();
			for(unsigned int n=0; n<6; n++) {
				sigmaCoeffs[n] += coeffs[n];
			}
		}

#define A sigmaCoeffs[0]
#define B sigmaCoeffs[1]
#define C sigmaCoeffs[2]
#define D sigmaCoeffs[3]
#define E sigmaCoeffs[4]
#define F sigmaCoeffs[5]

//		printf("e2 = %0.2fx^2 + %0.2fxy + %0.2fy^2 + %0.2fx + %0.2fy + %0.2f\n",
//			A,B,C,D,E,F);
//
//		printf("d/dx = %02fx + %02fy + %02f\n", 2*A, B, D);
//		printf("d/dy = %02fy + %02fx + %02f\n", 2*C, B, E);

		// solve for the minimum

		y = (2*A*E - B*D) / (B*B - 4*A*C);
		x = -(B*y + D)/(2*A);

		// calculate the error2
		e2 = A * x * x + B * x * y + C * y * y + D * x + E * y + F;

	}

	float x;
	float y;
	float e2;
	vector<Lop> lops;
};


#endif /* SOLVE_H_ */
