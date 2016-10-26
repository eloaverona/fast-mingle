/*
 * Point implementation file.
 */

#include "Point.h"
#include <cmath>

Point Point::operator-(Point &p) { return {x - p.x, y - p.y}; };
Point Point::operator+(Point &p) { return {x + p.x, y + p.y}; };
Point Point::operator*(double k) { return {k * x, k * y}; }
