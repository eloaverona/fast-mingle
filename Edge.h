/*
 * Edge.h
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#ifndef EDGE_H_
#define EDGE_H_
#include <cassert>
#include <unordered_set>

class Edge {
 public:
  Edge(PointOrStar point1, PointOrStar point2);
  virtual ~Edge();

  void setPoints(PointOrStar point1, PointOrStar point2) {
    assert(point1 != nullptr && point2 != nullptr);
    _points = std::make_pair(point1, point2);
  }

  std::pair<PointOrStar, PointOrStar>& getPoints() { return _points; }

  int getWeight() {
    return _points.first.getWeight() + _points.second.getWeight();
  }

  double getInk() const { return _ink; }

 private:
  std::pair<PointOrStar, PointOrStar> _points;
  double _ink;

  double getDistanceBetweenPoints(PointOrStar point1, PointOrStar point2) {
    return sqrt((point1.getX() - point2.getX()) ^
                2 + (point1.getY() - point2.getY()) ^ 2);
  }
};

#endif /* EDGE_H_ */
