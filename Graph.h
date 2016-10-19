#ifndef MINGLE_GRAPH_H
#define MINGLE_GRAPH_H

#include "ANN/ANN.h"
#include <unordered_set>

/**
 * Represents the 4-dimensional graph, Γ, that has all the edges.
 */
class Graph {
public:
    Graph() {}
    ~Graph() {}
    /**
     * Reads the edges from a file.
     */
    void readEdgesFromFile(const char *edgesFilePath);

    /**
     * Mingles the nodes stored in this graph.
     */
    void doMingle();

    /**
     * Estimates the ink saved when two edges are bundled.
     */
    double estimateSavedInkWhenTwoEdgesBundled(EdgeNode *node1, EdgeNode *node2);

private:
    /**
     * These two members are needed by the ANN library
     * to make the search
     * annPoints stores the points in the graph.
     * annTree stores the points as a graph to perform the search.
     */
    ANNpointArray annPoints = nullptr;
    ANNkd_tree *annTree = nullptr;

    /**
     * A list of points in this graph. The index of the point in the vector
     * determines the point's id.
     */
    std::vector<Point>_points;

    /**
     * The list of the top-level nodes in this graph.
     */
    std::vector<EdgeNode*> _nodes;

    /**
     * Reads the next line of a file.
     */
    void readNextEdgeInFile(FILE *filePointer, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId);

    /**
     * When reading points from the file, sets the id of a point if the point
     * already been seen. If not, it assigns a new id to it.
     */
    void setIdOfPoint(Point *point, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId);

    /**
     * Rebuilds the ANN index to include all the root nodes. This allows us
     * to search with the new list of root nodes.
     */
    void rebuildAnnIndex();

    /**
     * Gets the centroid of the first points of node 1 and node 2.
     */
    Point getMeetingPointOneForNodes(EdgeNode *node1, EdgeNode *node2) {
    	return {node1->getPointOne()->x * node1->getWeight() + node2->getPointOne()->x * node2->getWeight(),
    			node1->getPointOne()->y * node1->getWeight() + node2->getPointOne()->y * node2->getWeight()};
    }

    /**
     * Gets the centroid of the second points of node 1 and node 2.
     */
    Point getMeetingPointTwoForNodes(EdgeNode *node1, EdgeNode *node2) {
    	return { node1->getPointTwo()->x * node1->getWeight() + node2->getPointTwo()->x * node2->getWeight(),
    			 node1->getPointTwo()->y * node1->getWeight() + node2->getPointTwo()->y * node2->getWeight()};
    }

    /**
     *
     */
    double getInkToDrawEdgesBetweenNodeAndChildren(Point *node, std::vector<Point*> children);

    /**
     *
     */

    const double LOOKUP_RANGE = 0.25;

    const double TOLERANCE_RANGE = 0.01;

    const double GOLD_RATIO = 1.618034;

    const int GOLD_SECTION_SEARCH_MAX_LOOPS = 10;

};

#endif //MINGLE_GRAPH_H
