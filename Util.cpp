#include "Util.h"

Point lerp(const Point &a, const Point &b, const double delta) {
  return {(b.x - a.x) * delta + a.x, (b.y - a.y) * delta + a.y};
}

Point Point::operator+(Point &p) { return {x + p.x, y + p.y}; }

Point Point::operator-(Point &p) { return {x - p.x, y - p.y}; }

Point Point::operator*(int k) { return {k * x, k * y}; }

double Point::operator*(Point &p) { return x * p.x + y * p.y; }

Point Point::operator/(double k) { return {x / k, y / k}; }

bool Point::operator==(const Point &other) const {
  return x == other.x && y == other.y;
}

bool Point::operator<(const Point &rightHandSide) {
	if (x < rightHandSide.x) {
		return true;
	} else if (rightHandSide.x > x) {
		return false;
	} else {
		if (y < rightHandSide.y) {
			return true;
		} else {
			return false;
		}
	}
}

std::size_t PointHasher::operator()(const Point &p) const {
  return std::hash<double>()(p.x) ^ (std::hash<double>()(p.y) << 1);
}
