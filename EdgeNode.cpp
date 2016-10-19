#include "EdgeNode.h"

EdgeNode::EdgeNode(Point *pointA, Point *pointB){
	if (pointA == nullptr || pointB == nullptr) {
		throw std::invalid_argument( "Both points have to be defined." );
	}
	// The point with the smallest x coordinate goes on point one
	if (pointA->x < pointB->x) {
		_pointOne = pointA;
		_pointTwo = pointB;
	} else {
		_pointOne = pointB;
		_pointTwo = pointA;
	}
	/*
	 * The intial ink is just the difference between these two points.
	 */
	_ink = &_pointTwo - &_pointOne;
}

double EdgeNode::estimateInkSaved(EdgeNode *other) {
	double inkWhenbundled = 0;
	double currentInk = this->getInk() + other->getInk();
	return currentInk - inkWhenBundled;
}

std::vector<Point*> EdgeNode::getChildrenAtPointOnePoints() {
	std::vector<EdgeNode*> children = getChildrenAtPointOne();
	std::vector<Point*> points;
	points.resize(children.size());
	for (auto child : children) {
		points.push_back(child->getPointOne());
	}
	return points;
}

std::vector<Point*> EdgeNode::getChildrenAtPointTwoPoints() {
	std::vector<EdgeNode*> children = getChildrenAtPointTwo();
	std::vector<Point*> points;
	points.resize(children.size());
	for (auto child : children) {
		points.push_back(child->getPointTwo());
	}
	return points;
}

void EdgeNode::addChildAtPointOne(EdgeNode *child) {
	_childrenAtPointOne.push_back(child);
}

void EdgeNode::addChildAtPointTwo(EdgeNode *child) {
	_childrenAtPointTwo.push_back(child);
}
