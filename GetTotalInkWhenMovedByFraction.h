/*
 * This is a functor used to calculate the total distance between a point and a
 * list of points. The
 * functor takes a moveFactor parameter that moves the point along the move
 * vector and recalculates
 * the total distance. This functor is used by the boost brent search
 * optimization algorithm to find
 * the geometric median of the points.
 */

#ifndef GETTOTALINKWHENMOVEDBYFRACTION_H_
#define GETTOTALINKWHENMOVEDBYFRACTION_H_

#include "Point.h"
#include <vector>

class GetTotalInkWhenMovedByFraction {
public:
  /**
   * Build a functor object. It calculates the total distance between
   * pointThatMoves and
   * childPoints. It takes a moveVector to move pointThatMoves and recalculate
   * the new distance
   * when the point has been moved.
   *
   * @param pointThatMoves The point to be moved.
   * @param moveVector The vector that indicates the direction in which
   * pointThatMoves will be
   *            moved.
   * @param childPoints The distance is calculated between pointThatMoves and
   * these childPoints.
   */
  GetTotalInkWhenMovedByFraction(Point &pointThatMoves, Point &moveVector,
                                 std::vector<Point> &childPoints, std::vector<int> &pointWeights);
  /**
   * Calculate the new distance between pointThatMoves and childPoints when
   * pointThatMoves is
   * moved by a certain factor.
   *
   * @param moveFactor The move factor to move pointThatMoves.
   */
  double operator()(double moveFactor);

  /**
   * Straight edge parameter. In order to promote straight edges, the result
   * of the ink saving is multiplied by this parameter. Refer to the paper
   * for more information. 2.53209 is the value of 1 + cos(50) / 0.5, meaning
   * that the maximum turning angle is 50 degrees and the value of p is 0.5.
   */
  static constexpr double STRAIGHT_EDGE_PARAM = 1.69640;

private:
  Point _pointThatMoves;
  Point _moveVector;
  std::vector<Point> _childPoints;
  std::vector<int> _pointWeights;
};

#endif /* GETTOTALINKWHENMOVEDBYFRACTION_H_ */
