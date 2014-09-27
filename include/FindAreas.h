
#ifndef FINDAREAS_H_
#define FINDAREAS_H_

#include <opencv2/opencv.hpp>
#include "RectZone.hpp"

using namespace std;

class FindAreas {
public:
	static cv::Rect FindLargest(unsigned char limit, unsigned char edge, cv::Mat &area, int termCount) {

		// Find the maximum in area
		unsigned char max=0;
		for(cv::MatConstIterator_<unsigned char> it=area.begin<unsigned char>();
				it != area.end<unsigned char>(); it++) {
			if(*it > max) max = *it;
		}

		int moveThresh = 55;

		if(max < moveThresh) {
			fprintf(stderr, "Nothing moving? max %d < %d\n", max, moveThresh);
			return cv::Rect(0,0,0,0);
		}

		limit = (unsigned char)((int)limit * max / 100);
		edge = (unsigned char)((int)edge * max / 100);

		cv::Point poi(0,0);

		RectZone zoneCover(area.cols);
		vector<cv::Rect> zones;
		int numScans = 0;

		while(poi.y < area.rows) {

			if(area.at<unsigned char>(poi) >= limit) {
				// Found a protuberance
				numScans++;

				if(numScans > termCount) {
					fprintf(stderr, "Too much noise, numScans %d > %d\n", numScans, termCount);
					return cv::Rect(0,0,0,0);
				}

				cv::Rect zone(poi, cv::Size(1,1));

				bool grown;
				do {
					grown = false;
					if(zone.x > 0
						&& vertZoneOver(area, edge, zone.x-1, (zone.y>0) ? zone.y-1 : 0,
								((zone.br().y) < area.rows) ? (zone.br().y) : area.rows)) {
						zone.x--;
						zone.width++;
						grown = true;
					}
					if((1+zone.br().x) < area.cols
						&& vertZoneOver(area, edge, zone.br().x+1, (zone.y>0) ? zone.y-1 : 0,
								((zone.br().y) < area.rows) ? (zone.br().y) : area.rows)) {
						zone.width++;
						grown = true;
					}

					if(zone.y > 0
						&& horZoneOver(area, edge, zone.y-1, (zone.x>0) ? zone.x-1 : 0,
								((zone.br().x) < area.cols) ? (zone.br().x) : area.cols)) {
						zone.y--;
						zone.height++;
						grown = true;
					}
					if((1+zone.br().y) < area.rows
						&& horZoneOver(area, edge, zone.br().y+1, (zone.x>0) ? zone.x-1 : 0,
								((zone.br().x) < area.cols) ? (zone.br().x) : area.cols)) {
						zone.height++;
						grown = true;
					}
				}
				while(grown);

				// add to zones
				zones.push_back(zone);

				// and to zoneCover
				zoneCover.Add(zone);
			}

			poi = zoneCover.After(poi);
		}

		fprintf(stderr, "FindAreas::FindLargest scans=%d, zones=%d\n", numScans, (int)zones.size());

		sort(zones.begin(), zones.end(), areaCompare);

		if(zones.empty()) {
			return cv::Rect(0,0,0,0);
		}
		else {
			return zones[0];
		}
	}

private:
	static bool areaCompare(cv::Rect a, cv::Rect b) {
		int areaA = a.height * a.width;
		int areaB = b.height * b.width;

		return areaA > areaB;
	}

	static bool vertZoneOver(cv::Mat &area, unsigned char edge, int x, int sy, int ey) {
		for(int y=sy; y<ey; y++) {
			if(area.at<unsigned char>(y,x) >= edge) {
				return true;
			}
		}

		return false;
	}

	static bool horZoneOver(cv::Mat &area, unsigned char edge, int y, int sx, int ex) {
		for(int x=sx; x<ex; x++) {
			if(area.at<unsigned char>(y,x) >= edge) {
				return true;
			}
		}

		return false;
	}
};

#endif
