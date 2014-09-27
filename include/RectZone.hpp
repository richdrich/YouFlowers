
#ifndef RECTZONE_H_
#define RECTZONE_H_

using namespace std;

class RectZone {
public:
	int Xlimit;
	map<int, map<int, cv::Rect> > entriesByEdge;

	RectZone(int xlimit) {
		Xlimit = xlimit;
	};

	void Add(const cv::Rect &r) {
		int edge = r.br().x;

		entriesByEdge[edge][r.br().y] = r;
	}

	string ToString() {
		stringstream res;

		for (map<int, map<int, cv::Rect> >::iterator it=entriesByEdge.begin() ;
				it != entriesByEdge.end(); it++ ) {

			res << "Column (right)=" << it->first << endl;
			for(map<int, cv::Rect>::iterator cit=it->second.begin();
					cit != it->second.end(); cit++) {
				res << " " << cit->first << " [" << cit->second.x << ", "
					<< cit->second.y << ", "
					<< cit->second.br().x << ", "
					<< cit->second.br().y << "]" << endl;
			}
		}

		res << endl;

		return res.str();
	}

	cv::Point After(cv::Point from) {
		from.x++;	// next point along

		do {
			// Skip to next row when required
			if(from.x >= Xlimit) {
				from.x = 0;
				from.y++;
			}

			// Find a list of columns where the right edge is after x
			map<int, map<int, cv::Rect> >::iterator itcols = entriesByEdge.lower_bound(from.x);

			// iterate these columns
			// until we find a zone with our point in
			bool loopDone = false;
			while(!loopDone && itcols != entriesByEdge.end()) {
				map<int, cv::Rect> column = itcols->second;

				// find a list of zones where the bottom corner is after x,y
				map<int, cv::Rect>::iterator itrows = column.lower_bound(from.y);

				// find the first zone that contains our point
				while( itrows != column.end()) {
					if( itrows->second.contains(from)) {
						// move to the point after the zone edge
						from.x = itrows->second.br().x;
						loopDone = true;
						break;
					}

					itrows++;
				}

				itcols++;
			}
		}
		while(from.x >= Xlimit);

		return from;
	}


};

#endif
