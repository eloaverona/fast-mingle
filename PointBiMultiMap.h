#ifndef POINTBIMULTIMAP_H_
#define POINTBIMULTIMAP_H_
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include "Util.h"

class PointBiMultiMap {
public:
	PointBiMultiMap();
	void addEdge(Point point1, Point point2);
	void removeEdge(Point point1, Point point2);
	std::unordered_set<Point> getPointsWithEdgesToPoint(Point point);
	virtual ~PointBiMultiMap();
private:
	std::unordered_map<Point, std::unordered_set<Point>, PointHasher> keysToValuesMap;
	std::unordered_map<Point, std::unordered_set<Point>, PointHasher> valuesToKeysMap;
	void swapPoints(Point& point1, Point& point2)
};

#endif /* POINTBIMULTIMAP_H_ */
