#ifndef MINGLEC_EDGEGRAPH_H
#define MINGLEC_EDGEGRAPH_H

#include "Util.h"
#include "EdgeNode.h"
#include "ANN/ANN.h"
#include <unordered_set>

/**
 * Represents the 4-dimensional graph, Γ, that has all the edges.
 */
class EdgeGraph {
public:
    EdgeGraph() {}
    ~EdgeGraph() {}
    /**
     * Reads the edges from a file.
     */
    void readEdgesFromFile(const char *edgesFilePath);

    /**
     * Mingles the nodes stored in this graph.
     */
    void doMingle();

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

};

#endif //MINGLEC_EDGEGRAPH_H
