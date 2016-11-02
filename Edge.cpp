/*
 * Implementation file for the Edge class.
 */

#include "Edge.h"
#include "Point.h"

Edge::Edge(Point source, Point target) {
  _source = source;
  _target = target;
  _weight = 1;
  _ink = Point::getDistanceBetweenPoints(source, target);
  _parent = nullptr;
}

void Edge::addChild(Edge *child) {
  _children.push_back(child);
  _ink += Point::getDistanceBetweenPoints(child->getSource(), getSource());
  _ink += Point::getDistanceBetweenPoints(child->getTarget(), getTarget());
  _weight += 1;
}

void Edge::clearParent() {
//	delete _parent;
	_parent = nullptr;
}
