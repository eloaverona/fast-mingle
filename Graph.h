#ifndef MINGLE_GRAPH_H
#define MINGLE_GRAPH_H

#include <cmath>
#include <unordered_set>
#include <vector>
#include "ANN/ANN.h"
#include "Edge.h"
#include "PointOrStar.h"
#include "Point.h"

/**
 * Represents the 4-dimensional graph, Γ, that has all the edges.
 */
class Graph {
 public:
  Graph() {}
  ~Graph() {}
  /**
   * Reads the edges from a file and loads them into the graph.
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
   * The list of the top-level nodes in this graph.
   */
  std::vector<Edge> _edges;

  /**
   * Reads the next line of a file.
   */
  void readNextEdgeInFile(FILE *filePointer);

  /**
   * Rebuilds the ANN index to include all the root nodes. This allows us
   * to search with the new list of root nodes.
   */
  void rebuildAnnIndex();

  /**
   * Estimates the ink saved when two edges are bundled.
   */
  double estimateSavedInkWhenTwoEdgesBundled(Edge *node1, Edge *node2);

  /**
   * Estimates the total ink if the two nodes were to be bundled together.
   *
   * @return The total ink with the bundle if the two edges were bundled
   * together.
   */
  double estimateInkWhenTwoEdgesBundled(Edge node1, Edge node2);

  /**
   * Makes an bundled edge with the source points and the target points. The
   * bundled edge has optimal meeting
   * points so that the ink used is the minimal.
   *
   * @param sourceSet A set of points which are close to each other. These will
   * be connected to one end of the edge.
   * @param targetSet A set of points which are close to each other. These will
   * be connected to the other end of the edge.
   */
  Edge getBundledEdge(std::pair<PointOrStar, PointOrStar> sourceSet,
                      std::pair<PointOrStar, PointOrStar> targetSet);

  /**
   * Gets the centroid of the points in the pair. The centroid's position
   * depends on each one of the point's weight.
   * The centroid will be placed closer to the point with the highest weight.
   * This is done for aesthetic purposes.
   *
   * @param sourceSet A pair of points to find the centroid.
   * @return The centroid of the two points in the sourceSet.
   */
  Point getCentroid(std::pair<PointOrStar, PointOrStar> set);

  /**
   * Searches for the optimal meeting point that will use the least ink. It uses
   * the golden section search algorithm to minimize
   * the ink function for the given sourceCentroid, targetCentroid and children.
   *
   * @param searchRangeDelta The search rage to search for the minimum.
   * @param tolerance The tolerance for the precision of the search. A lower
   * tolerance finds a more exact solution in a longer time.
   *                      A higher tolerance finds a less exact solution faster.
   * @param sourceCentroid The centroid of the source points.
   * @param targetCentroid The centroid of the target points.
   * @param sourceSet The set of points that are connected to the
   * sourceCentroid.
   */
  Point goldenSectionSearch(double searchRangeDelta, double tolerance,
                            Point sourceCentroid, Point targetCentroid,
                            std::pair<PointOrStar, PointOrStar> sourceSet);

  /**
   * Gets the ink that would be used if we used the given source point, with the
   * given targetPoint and sourceSet.
   *
   * @param sourcePoint The point that could be the optimal meeting point. This
   * sourcePoint is connected to the points in the sourceSet.
   * @param targetPoint The point on the other end of the edge.
   * @param sourceSet The points that would be connected to the sourcePoint.
   */
  double getInkForBoundPoint(Point sourcePoint, Point targetPoint,
                             std::pair<PointOrStar, PointOrStar> sourceSet);

  /**
   * Gets the distance between two points
   *
   * @return The distance between two points.
   */
  double getDistanceBetweenPoints(PointOrStar point1, PointOrStar point2) {
    return sqrt(pow((point1.getX() - point2.getX()), 2 )+ pow((point1.getY() - point2.getY()),  2));
  }

  /**
   * Gets the distance between two points.
   */
  double getDistanceBetweenPoints(PointOrStar point1, Point point2) {
    return sqrt(pow((point1.getX() - point2.x), 2) + pow((point1.getY() - point2.y), 2));
  }

  /**
   * Gets the distance between two points.
   */
  double getDistanceBetweenPoints(Point point1, Point point2) {
    return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
  }

  /**
   * This is the search range of the Golden section search as a fraction of the
   * difference between
   * the two meeting points.
   */
  const double GSS_SEARCH_RANGE_FRACTION = 0.25;

  const double GSS_TOLERANCE_FRACTION = 0.01;

  const double GOLD_RATIO = 1.618034;

  const int GSS_MAX_LOOPS = 10;
};

#endif  // MINGLE_GRAPH_H
