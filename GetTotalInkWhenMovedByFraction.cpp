/*
 * Implementation file of the GetTotalInkWhenMovedByFraction functor.
 */

#include "GetTotalInkWhenMovedByFraction.h"

GetTotalInkWhenMovedByFraction::GetTotalInkWhenMovedByFraction(
    Point &pointThatMoves, Point &moveVector, std::vector<Point> &childPoints) {
  _pointThatMoves = pointThatMoves;
  _moveVector = moveVector;
  _childPoints = childPoints;
}

double GetTotalInkWhenMovedByFraction::operator()(double moveFactor) {
  double totalInk = 0.0;
  Point move = _moveVector * moveFactor;
  Point newPoint = _pointThatMoves + move;
  for (Point childPoint : _childPoints) {
    totalInk += Point::getDistanceBetweenPoints(childPoint, newPoint);
  }
  return totalInk * STRAIGHT_EDGE_PARAM;
}
