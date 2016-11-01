/*
 * A point with x-y coordinates.
 */

#ifndef POINT_H_
#define POINT_H_

#include <cmath>

struct Point {
  float x, y;
  Point operator-(Point &p);
  Point operator+(Point &p);
  Point operator*(double k);
  bool operator==(const Point &other) const;

  static double getDistanceBetweenPoints(Point point1, Point point2) {
    double diffX = point1.x - point2.x;
    double diffY = point1.y - point2.y;
    return sqrt(diffX * diffX + diffY * diffY);
  }
};

struct PointHasher {
    std::size_t operator()(const Point& p) const;
};

#endif /* POINT_H_ */
