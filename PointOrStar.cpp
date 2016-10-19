/*
 * PointOrStar.cpp
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#include <PointOrStar.h>
#include <cassert>

PointOrStar::PointOrStar(double x, double y) {
	assert(x != nullptr && y != nullptr);
	_x = x;
	_y = y;
}
