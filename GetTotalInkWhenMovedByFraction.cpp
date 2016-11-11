/*
 * Implementation file of the GetTotalInkWhenMovedByFraction functor.
 */

#include "GetTotalInkWhenMovedByFraction.h"
#include <cassert>

GetTotalInkWhenMovedByFraction::GetTotalInkWhenMovedByFraction(
    Point &pointThatMoves, Point &moveVector, std::vector<Point> &childPoints, std::vector<int> &pointWeights) {
	assert(childPoints.size() == pointWeights.size());
  _pointThatMoves = pointThatMoves;
  _moveVector = moveVector;
  _childPoints = childPoints;
  _pointWeights = pointWeights;
}

double GetTotalInkWhenMovedByFraction::operator()(double moveFactor) {
  double totalInk = 0.0;
  Point move = _moveVector * moveFactor;
  Point newPoint = _pointThatMoves + move;
  for (int i = 0; i < _childPoints.size(); i++){
	  totalInk += Point::getDistanceBetweenPoints(_childPoints[i], newPoint)* _pointWeights[i];
  }
  return totalInk;
}
