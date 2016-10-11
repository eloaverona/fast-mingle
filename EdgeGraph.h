#ifndef MINGLEC_EDGEGRAPH_H
#define MINGLEC_EDGEGRAPH_H

#include "Util.h"
#include <unordered_set>


typedef uint32_t EdgeNodeId;


class EdgeNode {
public:
    EdgeNode(PointId pointOne, PointId pointTwo) : _pointOne(pointOne), _pointTwo(pointTwo) {}

    ~EdgeNode();

	EdgeNode* getParent() {
		return _parent;
	}

	void setParent(EdgeNode *parent) {
		_parent = parent;
	}

	PointId getPointOne() const {
		return _pointOne;
	}

	void setPointOne(PointId pointOne) {
		_pointOne = pointOne;
	}

	PointId getPointTwo() const {
		return _pointTwo;
	}

	void setPointTwo(PointId pointTwo) {
		_pointTwo = pointTwo;
	}

private:
	/**
	 * The first point of the edge.
	 */
    PointId _pointOne;
    /**
     * The second point of the edge.
     */
    PointId _pointTwo;

    /**
     * The parent edge of this node.
     */
    EdgeNode *_parent = nullptr;

    /**
     * The children of this edge node.
     */
    std::vector<EdgeNode> *_children = nullptr;

    /**
     * The weight of this node.
     */
    int weight = 1;
};


class EdgeGraph {
public:
    EdgeGraph() {}
    ~EdgeGraph() {}
    void readEdgesFromFile(const char *edgesFilePath);

private:
    /**
     * A list of points in this graph. The index of the point in the vector
     * determines the point's id.
     */
    std::vector<Point>_points;

    /**
     * The list of the top-level nodes in this graph.
     */
    std::vector<EdgeNode*> _nodes;

    void readNextEdgeInFile(FILE *filePointer, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId);

    void setIdOfPoint(Point *point, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId);
};

#endif //MINGLEC_EDGEGRAPH_H
