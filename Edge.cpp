/*
 * Edge.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#include <Edge.h>
#include <cassert>

Edge::Edge(PointOrStar point1, PointOrStar point2) {
	assert(point1 != nullptr && point2 != nullptr);
	_points = std::make_pair(point1, point2);
}


