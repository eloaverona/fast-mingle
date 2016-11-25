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

  static double getDotProductOfVectors(Point point1, Point point2) {
         return point1.x * point2.x + point1.y * point2.y;
  }

  static double getLengthOfVector(Point vector) {
            return sqrt(vector.x * vector.x + vector.y * vector.y);
  }

  static double getAngleBetweenVectors(Point vector1, Point vector2) {
	  double dotProd = Point::getDotProductOfVectors(vector1, vector2);
	  double lengthProduct = Point::getLengthOfVector(vector1) * Point::getLengthOfVector(vector2);
	  return acos(dotProd / lengthProduct);
  }

};

struct PointHasher {
  std::size_t operator()(const Point &p) const;
};

#endif /* POINT_H_ */
