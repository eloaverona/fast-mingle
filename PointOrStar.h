/*
 * PointOrStar.h
 *
 *  Created on: Oct 19, 2016
 *      Author: alan
 */

#ifndef POINTORSTAR_H_
#define POINTORSTAR_H_

#include <cmath>
#include <vector>

class PointOrStar {
 public:
  PointOrStar(double x, double y);
  ~PointOrStar();
  bool isStar() { return _children.size() != 0; }
  bool isPoint() { return _children.size() == 0; }

  std::vector<PointOrStar> getChildren() { return _children; }

  void addChild(PointOrStar child) {
    _children.push_back(child);
    _weight += child.getWeight();
    _ink += distanceToPoint(child);
  }

  double getX() { return _x; }

  double getY() { return _y; }

  int getWeight() { return _weight; }

  double getInk() { return _ink; }

 private:
  std::vector<PointOrStar> _children;
  double _x;
  double _y;
  int _weight;
  double _ink;

  double distanceToPoint(PointOrStar point) {
    sqrt((_x - point.getX()) ^ 2 + (_y - point.getY()) ^ 2);
  }
};

#endif /* POINTORSTAR_H_ */
