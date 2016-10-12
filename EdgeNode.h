#ifndef MINGLEC_EDGENODE_H
#define MINGLEC_EDGENODE_H

#include "Util.h"

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

	EdgeNode* getParent() {
		return _parent;
	}

	bool hasParent() {
		return _parent == nullptr;
	}

	void setParent(EdgeNode *parent) {
		_parent = parent;
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

	void estimateInkSaved(EdgeNode *other);

	int getInk() const {
		return _ink;
	}

	int getWeight() const {
		return _weight;
	}

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
     * The parent edge of this node.
     */
    EdgeNode *_parent = nullptr;

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
    int _ink = 0;
};

#endif //MINGLEC_EDGENODE_H
