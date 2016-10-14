#include <PointBiMultiMap.h>

PointBiMultiMap::PointBiMultiMap() {}

void PointBiMultiMap::addEdge(Point point1, Point point2) {
	if (point1 < point2) {
		swapPoints(point1, point2);
	}
	std::unordered_set pointValueSet = keysToValuesMap.find(point1);
	if(pointValueSet == keysToValuesMap.end()) {
		keysToValueMap.insert({point1,  { point2 }});
	} else {
		pointValueSet.insert(point2);
	}

	std::unordered_set pointKeySet = keysToValuesMap.find(point1);
	if(pointKeySet == valuesToKeysMap == valuesToKeysMap.end()) {
		valuesToKeysMap.insert({point2, {point1}});
	} else {
		pointKeySet.insert(point1);
	}
}

void PointBiMultiMap::swapPoints(Point& point1, Point& point2) {
	Point temp = point1;
	point1 = point2;
	point2 = point1;
}

PointBiMultiMap::~PointBiMultiMap() {
	delete(keysToValuesMap);
	delete(valuesToKeysMap);
}

