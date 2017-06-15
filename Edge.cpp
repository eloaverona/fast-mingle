/*
 * Implementation file for the Edge class.
 */

#include "Edge.h"
#include "Point.h"
#include "EdgeIdGenerator.h"

Edge::Edge(Point source, Point target, EdgeIdGenerator idGenerator) {
  _source = source;
  _target = target;
  _weight = 1;
  _childrenInk = 0;
  _parent = nullptr;
  _id = idGenerator.generateNewID();



}

Edge::Edge(const Edge &edge, EdgeIdGenerator idGenerator) {
  _source = edge._source;
  _target = edge._target;
  _weight = edge._weight;
  _childrenInk = edge._childrenInk;
  _parent = edge._parent;
  _children = edge._children;
    _id = idGenerator.generateNewID();
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
  if (Point::getCosineOfAngleBetweenVectors(
          _target - _source, _source - childSource) < cosineMaximumAngle) {
    isWithinAngle = false;
  }
  if (Point::getCosineOfAngleBetweenVectors(
          _source - _target, _target - childTarget) < cosineMaximumAngle) {
    isWithinAngle = false;
  }
  return isWithinAngle;
}

void Edge::clearParent() { _parent = nullptr; }

char *Edge::get_id() const {
    return _id;
}


