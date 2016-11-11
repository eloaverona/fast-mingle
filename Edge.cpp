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
  _children.push_back(child);
  _childrenInk += Point::getDistanceBetweenPoints(child->getSource(), getSource()) * child->getInkWeight();
  _childrenInk += Point::getDistanceBetweenPoints(child->getTarget(), getTarget()) * child->getInkWeight();
  _weight += child->getWeight();
}

void Edge::clearParent() {
  _parent = nullptr;
}
