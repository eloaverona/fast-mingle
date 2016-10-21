/*
 * Point.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: alan
 */

#include <Point.h>
#include <cmath>

Point Point::operator-(Point& p) { return {x - p.x, y - p.y}; };
Point Point::operator+(Point& p) { return {x + p.x, y + p.y}; };
Point Point::operator*(int k) { return {k * x, k * y}; }
