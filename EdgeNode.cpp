#include "EdgeNode.h"

EdgeNode::EdgeNode(Point *pointA, Point *pointB){
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
