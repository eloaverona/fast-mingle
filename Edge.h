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

	std::unordered_set<PointOrStar>& getPoints() {
		return _points;
	}

	void setPoints(PointOrStar point1, PointOrStar point2) {
		assert(point1 != nullptr && point2 != nullptr);
		_points.insert(point1);
		_points.insert(point2);
	}

private:
	std::unordered_set<PointOrStar> _points;
};

#endif /* EDGE_H_ */
