#ifndef MINGLEC_EDGENODE_H
#define MINGLEC_EDGENODE_H

#include "Util.h"
#include <stdexcept>

/*
 * Represents an edge between two points: pointOne and pointTwo. This is a node
 * in the 4-dimensional graph, Γ.
 */
class EdgeNode {
public:
	/**
	 * Make an edge node given two points.
	 */
    EdgeNode(Point *pointA, Point *pointB);

    ~EdgeNode();

	const EdgeNode*& getParentAtPointOne() const {
		return _parentAtPointOne;
	}

	const EdgeNode*& getParentAtPointTwo() const {
		return _parentAtPointTwo;
	}

	void setParentAtPointOne(EdgeNode *parent) {
		if (_parentAtPointTwo != nullptr) {
			throw std::invalid_argument( "Cannot set two parents at the same time. Node has parent at point two." );
		}
		_parentAtPointOne = parent;
	}

	void setParentAtPointTwo(EdgeNode *parent) {
		if (_parentAtPointOne != nullptr) {
			throw std::invalid_argument( "Cannot set two parents at the same time. Node has parent at point one." );
		}
		_parentAtPointTwo = parent;
	}

	Point *getPointOne() const {
		return _pointOne;
	}

	Point *getPointTwo() const {
		return _pointTwo;
	}

	bool hasChildren() {
		return _childrenAtPointOne.empty() && _childrenAtPointTwo.empty();
	}

	void addChildAtPointOne(EdgeNode *child);

	void addChildAtPointTwo(EdgeNode *child);

	double getInk() const {
		return _ink;
	}

	int getWeight() const {
		return _weight;
	}

	std::vector<EdgeNode*> getChildrenAtPointOne() {
		return _childrenAtPointOne;
	}

	std::vector<EdgeNode*> getChildrenAtPointTwo() {
		return _childrenAtPointTwo;
	}

	std::vector<Point*> getChildrenAtPointOnePoints();

	std::vector<Point*> getChildrenAtPointTwoPoints();

private:
	/**
	 * The first point of the edge. This point's x coordinate
	 * is smaller than point two's x coordinate.
	 */
    Point *_pointOne;
    /**
     * The second point of the edge. This point's x coordinate
     * is larger than pint one's x coordinate.
     */
    Point *_pointTwo;

    /**
     * The parent edge of this edge at point one.
     */
    EdgeNode *_parentAtPointOne = nullptr;

    /**
     * The parent edge of this edge at point two.
     */
    EdgeNode *_parentAtPointTwo = nullptr;

    /**
     * The children of this edge node at point one.
     */
    std::vector<EdgeNode*> _childrenAtPointOne;

    /**
     * The children of this edge node at point Two.
     */
    std::vector<EdgeNode*> _childrenAtPointTwo;

    /**
     * The weight of this node.
     */
    int _weight = 1;

    /**
     * The ink of this node. This ink is the difference between point one
     * and point two.
     */
    double _ink = 0;
};

#endif //MINGLEC_EDGENODE_H
