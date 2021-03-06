/*
 * Implementation file of the GetTotalInkWhenMovedByFraction functor.
 */

#include "GetTotalInkWhenMovedByFraction.h"
#include <cassert>
#include <cmath>
#include <limits>

GetTotalInkWhenMovedByFraction::GetTotalInkWhenMovedByFraction(
    Point &sourcePoint, Point &targetPoint, Point &moveVector,
    std::vector<Point> &childPoints, std::vector<double> &pointWeights) {
  assert(childPoints.size() == pointWeights.size());
  _sourcePoint = sourcePoint;
  _moveVector = moveVector;
  _childPoints = childPoints;
  _pointWeights = pointWeights;
  _targetPoint = targetPoint;
  for (double weight : _pointWeights) {
    _totalWeight += weight;
  }
}

double GetTotalInkWhenMovedByFraction::operator()(double moveFactor) {
  Point move = _moveVector * moveFactor;
  Point newPoint = _sourcePoint + move;
  // log(totalWeight) + 1 is the ink weight. See getInkWeight on Edge.h for
  // details.
  double totalInk = Point::getDistanceBetweenPoints(newPoint, _targetPoint) *
                    (log(double(_totalWeight)) / 2 + 1);
  double minCosineAngle = 1.1; // Maximum value. Cosine ranges from -1 to 1..
  bool isOutOfBounds = false;
  for (int i = 0; i < _childPoints.size(); i++) {
    totalInk += Point::getDistanceBetweenPoints(_childPoints[i], newPoint) *
                _pointWeights[i];
    double cosineAngle = Point::getCosineOfAngleBetweenVectors(
        _targetPoint - _sourcePoint, _sourcePoint - _childPoints[i]);
    if (minCosineAngle > cosineAngle) {
      minCosineAngle = cosineAngle;
    }
  }
  return totalInk * (1 + minCosineAngle);
}
