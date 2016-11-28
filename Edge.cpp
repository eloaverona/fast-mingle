/*
 * Implementation file for the Edge class.
 */

#include "Edge.h"
#include "Point.h"

Edge::Edge(Point source, Point target) {
  _source = source;
  _target = target;
  _weight = 1;
  _childrenInk = 0;
  _parent = nullptr;
}

void Edge::addChild(Edge *child) {
  if (!isChildWithinAngle(child)) {
    throw "Child is not within the limit angle.";
  }
  _children.push_back(child);
  _childrenInk +=
      Point::getDistanceBetweenPoints(child->getSource(), getSource()) *
      child->getInkWeight();
  _childrenInk +=
      Point::getDistanceBetweenPoints(child->getTarget(), getTarget()) *
      child->getInkWeight();
  _weight += child->getWeight();
}

bool Edge::isChildWithinAngle(Edge *child) {
  bool isWithinAngle = true;
  Point childSource = child->getSource();
  Point childTarget = child->getTarget();
  if (Point::getAngleBetweenVectors(_target - _source, _source - childSource) >
      maximumAngle) {
    isWithinAngle = false;
  }
  if (Point::getAngleBetweenVectors(_source - _target, _target - childTarget) >
      maximumAngle) {
    isWithinAngle = false;
  }
  return isWithinAngle;
}

void Edge::clearParent() { _parent = nullptr; }
