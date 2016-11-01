/*
 * Point implementation file.
 */

#include "Point.h"
#include <cmath>
#include <functional>

Point Point::operator-(Point &p) { return {x - p.x, y - p.y}; };
Point Point::operator+(Point &p) { return {x + p.x, y + p.y}; };
Point Point::operator*(double k) { return {(float) k * x, (float) k * y}; };
bool Point::operator==(const Point &other) const {
  return x == other.x && y == other.y;
};

std::size_t PointHasher::operator()(const Point &p) const {
    return std::hash<float>()(p.x) ^ (std::hash<float>()(p.y) << 1);
}
