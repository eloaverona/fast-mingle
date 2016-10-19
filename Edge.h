/*
 * Edge.h
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#ifndef EDGE_H_
#define EDGE_H_
#include <unordered_set>
#include <cassert>

class Edge {
public:
	Edge(PointOrStar point1, PointOrStar point2);
	virtual ~Edge();

	void setPoints(PointOrStar point1, PointOrStar point2) {
		assert(point1 != nullptr && point2 != nullptr);
		_points = std::make_pair(point1, point2);
	}

	std::pair<PointOrStar, PointOrStar>& getPoints() {
		return _points;
	}

	int getWeight() {
		return _weight;
	}

private:
	std::pair<PointOrStar, PointOrStar> _points;
	int _weight;
};

#endif /* EDGE_H_ */
