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

  static double getDistanceBetweenPoints(Point point1, Point point2) {
    double diffX = point1.x - point2.x;
    double diffY = point1.y - point2.y;
    return sqrt(diffX * diffX + diffY * diffY);
  }
};

#endif /* POINT_H_ */
