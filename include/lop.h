/*
 * lop.h
 *
 *  Created on: Sep 14, 2012
 *      Author: richard
 */

#ifndef LOP_H_
#define LOP_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdarg.h>

#include <opencv/cxcore.h>

using namespace cv;
using namespace std;

class Lop {
public:
	Lop(float x, float y, float theta, float weight) {
		this->weight = weight;
		this->sx = x;
		this->sy = y;
		this->theta = theta;
		if(abs(theta-M_PI/2.) < 0.0001 || abs(theta-3.*M_PI/2.) < 0.0001) {
			// vertical line
			type = OFFSET;
			offset.d = x;
		}
		else {
			type = STRAIGHT;
			straight.m = tanf(theta);
			straight.c = y - straight.m * x;
		}
	};

//	Lop(float x, float y, float theta, float perp, float weight) {
//		this->weight = weight;
//		this->sx = x;
//		this->sy = y;
//		this->pointsUpRight = ???;
//
//		if(abs(theta) < 0.0001 || abs(theta-M_PI) < 0.0001) {
//			type = OFFSET;
//			offset.d = x + (theta==0 ? 1 : -1) * perp;
//		}
//		else {
//			type = STRAIGHT;
//
//			float alpha = theta + M_PI/2;
//			if(alpha > 2*M_PI) {
//				alpha -= 2*M_PI;
//			}
//
//			straight.m = tanf(alpha);
//
//			float xo = x + perp * cosf(theta);
//			float yo = y + perp * sinf(theta);
//			straight.c = yo - straight.m * xo;
//		}
//	};

	vector<float> GetWeightedPolynomialE2() {
		vector<float> res = GetPolynomialE2();

		for(unsigned int n=0; n<6; n++) {
			res[n] *= weight;
		}

		return res;
	}

	vector<float> GetPolynomialE2() {
		vector<float> res(6);

		if(type==STRAIGHT) {
			float m2p1 = straight.m*straight.m + 1;
			res[0] = straight.m * straight.m / m2p1;  		// x2
			res[1] = -2 * straight.m / m2p1;				// xy
			res[2] = 1 / m2p1;								// y2
			res[3] = 2 * straight.c * straight.m / m2p1;	// x
			res[4] = -2 * straight.c / m2p1;				// y
			res[5] = straight.c * straight.c / m2p1;
		}
		else {
			res[0] = 1;		// x2
			res[1] = 0;		// xy
			res[2] = 0;		// y2
			res[3] = -2 * offset.d;		// x
			res[4] = 0;		// y
			res[5] = offset.d * offset.d;
		}

		return res;
	};

	std::string string_format(const char *fmt, ...) {
	    int size=100;
	    std::string str;
	    va_list ap;
	    while (1) {
	        str.resize(size);
	        va_start(ap, fmt);
	        int n = vsnprintf((char *)str.c_str(), size, fmt, ap);
	        va_end(ap);
	        if (n > -1 && n < size) {
	            str.resize(n);
	            return str;
	        }
	        if (n > -1)
	            size=n+1;
	        else
	            size*=2;
	    }

	    return str;
	}

	string GetEquation() {
		if(type==STRAIGHT) {
			return string_format("y = %0.2fx + %0.2f", straight.m, straight.c);
		}
		else {
			return string_format("x = %02f", offset.d);
		}
	}

	string GetPolynomialString() {
		vector<float> coeffs = GetPolynomialE2();

		return string_format("%0.2fx^2 + %0.2fxy + %0.2fy^2 + %0.2fx + %0.2fy + %0.2f",
			coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]);
	}

	class LineEnds {
	public:
		LineEnds() {};

		LineEnds(float sx, float sy, float ex, float ey) {
			this->sx = sx;
			this->sy = sy;
			this->ex = ex;
			this->ey = ey;
		}

		float sx;
		float sy;
		float ex;
		float ey;
	};

	LineEnds GetEnds(float maxx, float maxy) {
		// printf("GetEnds %s\n", GetEquation().c_str());

		if(type==STRAIGHT) {
			LineEnds res;
//			if(straight.c >=0 && straight.c <= maxy) {
//				res.sx = 0;
//				res.sy = straight.c;
//			} else if(straight.m==0) {
//				return LineEnds(0,0,0,0);
//			} else {
//				float d = -straight.c / straight.m;
//				if(d >=0 && d <= maxx) {
//					res.sx = d;
//					res.sy = 0;
//				} else {
//					float e = (maxy - straight.c) / straight.m;
//					if(e >= 0 && e <= maxx) {
//						res.sx = e;
//						res.sy = maxy;
//					} else {
//						return LineEnds(0,0,0,0);
//					}
//				}
//			}

			res.sx = sx;
			res.sy = sy;

			if(straight.m==0) {
//				res.ex = maxx;
//				res.ey = straight.c;
				res.ex = theta < M_PI/2.0 || theta >= 3*M_PI/2.0 ? maxx : 0;
				res.ey = straight.c;
				return res;
			}

			// y = 0 = md+c; d=-c/m => line cust x axis where x=d
			float d = -straight.c / straight.m;
			if(d >=0 && d <= maxx && res.sy != 0) {
				// line cuts x axis inside box
				res.ex = d;
				res.ey = 0;
				return res;
			} else {
				// line cuts x axis outside box
				// line cuts top y bound at x=e
				float e = (maxy - straight.c) / straight.m;
				if(e >= 0 && e <= maxx && res.sy != maxy) {
					// Line cuts top bound inside box
					res.ex = e;
					res.ey = maxy;
					return res;
				} else {
					// Line cust left x bound at y=f
					float f = maxx * straight.m + straight.c;
					if(f >=0 && f < maxy) {
						// Line cuts left side inside box
						res.ex = maxx;
						res.ey = f;
						return res;
					} else {
						// Line is entirely outside box
						return LineEnds(0,0,0,0);
					}
				}
			}
		}
		else {
			if(theta < M_PI/2.0 || theta >= 3*M_PI/2.0) {
				return LineEnds(offset.d, sy, offset.d, maxy);
			}
			else {
				return LineEnds(0.0f, sy, offset.d, 0);
			}
		}
	}

	static const int STRAIGHT=0;
	static const int OFFSET=2;

	float weight;
	int type;
	float sx,sy;
	float theta;

	union {
		struct {
			float m;
			float c;
		} straight;
		struct {
			float d;
		} offset;
	};
};


#endif /* LOP_H_ */
