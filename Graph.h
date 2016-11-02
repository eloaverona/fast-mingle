/*
 * This is the the four-dimensional graph Γ mentioned in the paper. The graph is
 * made up of edges.
 * The graph handles the bundling between edges. This file is responsible for
 * the main algorithm.
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "ANN/ANN.h"
#include "Edge.h"
#include <boost/cstdint.hpp>
#include <limits>
#include <vector>
#include <unordered_map>

class Graph {
public:
  Graph();
  /**
   * Reads edges from a file path and loads them into the graph.
   */
  void readEdgesFromFile(const char *edgesFilePath);

  /**
   * Does the main mingling step.
   */
  void doMingle();

  void writePointsAndEdges(const char *pointsFilePath, const char *edgesFilePath);

private:
  // Members needed for the ANN search
  ANNpointArray annPoints = nullptr;
  ANNkd_tree *annTree = nullptr;

  /**
   * The edges currently being looked at for the mingling.
   */
  std::vector<Edge *> _edges;

  std::vector<Edge *> _parentEdges;

  /**
   * Read the next edge in the file and add it to the _edges array.
   */
  void readNextEdgeInFile(FILE *filePointer);

  /**
   * Rebuild the ANN index for correct querying of neighbors with ANN.
   */
  void rebuildIndex();

  /**
   * Find the neighbors of the target edge and fill in the neighbors array with
   * the edge's neighbors.
   *
   * @param target The edge to look neighbors for.
   * @param neighbors The neighbors array to be filled with the target
   * neighbors.
   */
  void fillNeighborArrayWithEdgeNeighbors(Edge &target,
                                          std::vector<Edge *> &neighbors);

  /**
   * Estimate the ink savings if two edges were to make up one bundle.
   */
  double estimateInkSavings(Edge &edge1, Edge &edge2);

  /**
   * Get the bundled edge if these two edges were to make up a bundle.
   */
  Edge* getBundledEdge(Edge &edge1, Edge &edge2);

  /**
   * Get the bundled edge of two edges that currently are not part of a bundle.
   */
  Edge* getBundledEdgeOfTwoUnbundledEdges(Edge &edge1, Edge &edge2);

  /**
   * Get the source centroid point for two edges.
   */

  Point getSourceCentroid(Edge &edge1, Edge &edge2);
  /**
   * Get the target centroid point for two edges.
   */
  Point getTargetCentroid(Edge &edge1, Edge &edge2);

  /**
   * Get the centroid point for a set of edges.
   */
  Point getCentroid(std::vector<Point> &points);

  /**
   * Grab the children of two bundles and put them into one bundle.
   */
  Edge* mergeTwoBundles(Edge &edge1, Edge &edge2);

  /**
   * Get the bundle if edge1 were to be added to bundle.
   */
  Edge* addEdgeToBundle(Edge &edge1, Edge &bundle);

  void putTwoEdgesOnSameBundle(Edge &edge1, Edge &edge2);

  void deleteParentEdge(Edge &edge);

  void writeEdges(FILE *pointsFilePointer, FILE *edgesfilePointer, int &nextPointId, Edge& edge, std::unordered_map<Point,int, PointHasher> &pointToPointIdMap);
  void writePoints(FILE *pointsFilePointer, Edge &edge, int &nextPointId, std::unordered_map<Point,int, PointHasher> &pointToPointIdMap);
  void addPointIfNotInMap(FILE *pointsFilePointer, Point point, int &nextPointId,  std::unordered_map<Point,int, PointHasher> &pointToPointIdMap);

  /**
   * Use the brent search minimization algorithm to search for the optimal
   * meeting point.
   *
   * @param sourcePoint The point to optimize. The optimized point is the
   * meeting point.
   * @param targetPoint The "other point" that sourcePoint is connected to.
   * @param sourcePoints The other points other than targetPoint that
   * sourcePoint is connected to.
   */
  Point brentSearchMeetingPoint(Point &sourcePoint, Point &targetPoint,
                                std::vector<Point> &sourcePoints);
  Edge* findBestNeighborForEdge(Edge &edge, std::vector<Edge *> &neighbors);

  double BRENT_SEARCH_RANGE = 0.25;
  int BRENT_SEARCH_PRECISION = std::numeric_limits<double>::digits;
  boost::uintmax_t BRENT_SEARCH_MAX_ITERATIONS = 20;

  /**
   * The number of neighbors used for this graph. This is populated after the
   * edges are read
   * by the readEdges from file function.
   */
  int numNeighbors = 0;
};

#endif /* GRAPH_H_ */
