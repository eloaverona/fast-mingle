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
#include "InkAndBundle.h"
#include "NeighborAndBundle.h"
#include <boost/cstdint.hpp>
#include <limits>
#include <unordered_map>
#include <vector>

class Graph {
public:
  Graph();
  /**
   * Reads edges from a file path and loads them into the graph.
   */
  void readEdgesFromFile(const char *edgesFilePath);

  /**
   * Does mingling recursively.
   */
  void doRecursiveMingle();

  /**
   * Writes the point and edges to the given file paths.
   * @param pointsFilePath A string that indicates the file path
   *     of the file that will contain the points.
   * @param edgesFilePath A string that indicates the file path
   *     of the file that will contain the edges between points.
   */
  void writePointsAndEdges(const char *pointsFilePath,
                           const char *edgesFilePath);

private:
  // Members needed for the ANN search
  ANNpointArray annPoints = nullptr;
  ANNkd_tree *annTree = nullptr;

  /**
   * The edges currently being looked at for the mingling.
   */
  std::vector<Edge *> _edges;

  /**
   * A list of parent edges of the current edges that are being looked at.
   */
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
   * Sets the numNeighbors variable to the correct value according to
   * the number of edges. The numNeighbors variable is chosen to look
   * at a number of neighbors that is not too low but also not too large
   * for the number of edges.
   */
  void adjustNumNeighbors();

  /**
   * Does the main mingling step.
   *
   * @return The number of iterations done during this mingling step.
   *     The minimum number of iterations is 1.
   */
  int doMingle();

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
   * Get the bundled edge if these two edges were to make up a bundle.
   * @return A bundled edge that contains edge1 and edge2 as children, plus
   *     any other children that may be part of edge1 or edge2's bundles.
   */
  Edge *getBundledEdge(Edge &edge1, Edge &edge2);

  /**
   * Get the bundled edge of two edges that currently are not part of a bundle.
   * @return A new, bundled edge that contains edge1 and edge2.
   */
  Edge *getBundledEdgeOfTwoUnbundledEdges(Edge &edge1, Edge &edge2);

  /**
   * Get the source centroid point for two edges.
   * @return A point that is the centroid of edge1 and edge2 source points.
   */
  Point getSourceCentroid(Edge &edge1, Edge &edge2);

  /**
   * Get the target centroid point for two edges.
   * @return A point that is the centroid of edge1 and edge2 target points.
   */
  Point getTargetCentroid(Edge &edge1, Edge &edge2);

  /**
   * Get the centroid point for a set of edges.
   * @return A point that is the centroid of all of the edges in the input
   *     vector.
   */
  Point getCentroid(std::vector<Point> &points);

  /**
   * Grab the children of two bundles and put them into one bundle.
   * @param edge1 A bundle that has children and no parents.
   * @param edge2 A bundle that has children and no parents.
   * @return A  new bundle that contains all of the children in edge1 and edge2
   *     but is optimized to include the children of both.
   */
  Edge *mergeTwoBundles(Edge &edge1, Edge &edge2);

  /**
   * Get the bundle if edge1 were to be added to bundle.
   * @param edge1 An edge that may or may not have a bundle already.
   * @param bundle A bundle that already has children and no parents.
   * @return A new bundle that is optimized to include all of the children in
   * bundle
   *     and it also includes edge1.
   */
  Edge *addEdgeToBundle(Edge &edge1, Edge &bundle);

  /**
   * Grabs two edges and makes the necessary changes to put them on
   * one single bundle.  This is used after the findBestNeighborForEdge
   * method has returned the best neighhbor for a given edge. Because we
   * know that these two edges will give us ink savings, we put them together.
   *
   * @param edge1 The first edge to be put on the same bundle as edge2.
   * @param edge2 The second edge to be put on the same bundle as edge1.
   */
  void putTwoEdgesOnSameBundle(Edge &edge1, Edge &edge2, Edge *parent);

  /**
   * Deletes the parent edge of the provided edge from the _parentEdges array.
   * It also clears the parent pointer on the edge.
   *
   * @param edge The edge whose parent should be deleted.
   */
  void removeParentEdge(Edge &edge);

  /**
   * Writes the edges of a given edge. Looks at the edge's children and
   * writes the edge between this edge's points and the edge's children.
   * @param pointsFilePointer The FILE pointer of the file where the points
   *     will be written to.
   * @param edgesFilePointer The FILE pointer of the file where the edges
   *     will be written to.
   * @param nextPointId The next id to give to the next written point.
   * @param edge The edge to write it's point and children's edges.
   * @param pointToIdMap A map of points and their repsective id's.
   */
  void
  writeEdges(FILE *pointsFilePointer, FILE *edgesfilePointer, int &nextPointId,
             Edge &edge,
             std::unordered_map<Point, int, PointHasher> &pointToPointIdMap);

  /**
   * Writes the points of a given edge.
   * @param pointsFilePointer the FILe pointer of the file where the points
   *     will be written to.
   * @param edge The edge whose points should be written to the fle.
   * @param pointToIdMap A map of point to points Ids.
   */
  void
  writePoints(FILE *pointsFilePointer, Edge &edge, int &nextPointId,
              std::unordered_map<Point, int, PointHasher> &pointToPointIdMap);

  /**
   * Writes a point if the point hasn't been already written. It checks the
   * pointToPointId map to see if the point has not been written and if it
   * hasn't, it uses the nextPointId to write the point id to the file.
   * @param pointsFilePointer A pointer to the file to write the points to.
   * @param point The point to check and write if the point is not on the map.
   * @param nextPointId  The point id to give to the point if it is written.
   * @param pointToPointIdMap The map to check if the point has already been
   *     added.
   */
  void addPointIfNotInMap(
      FILE *pointsFilePointer, Point point, int &nextPointId,
      std::unordered_map<Point, int, PointHasher> &pointToPointIdMap);

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
                                std::vector<Point> sourcePoints,
                                std::vector<double> pointWeights);

  /**
   * Gets the current ink occupied by the edge or the edge's bundles combined.
   */
  double getCurrentInkOfTwoEdges(Edge &edge1, Edge &edge2);

  /**
   * Estimates the ink savings if edge1 and edge 2 were to be put together on
   * the same bundle.
   * @param edge1 The first edge.
   * @param edge2 The second edge.
   * @return The ink saved by putting both edges on a same bundle and the
   *     bundle that will contain them both if they were in the same bundle.
   */
  InkAndBundle estimateInkSavings(Edge &edge1, Edge &edge2);

  /**
   * Finds the neighbor that gives the most ink savings for an edge.
   * @param edge The edge to find whose neighbor gives the most ink savings.
   * @param neighbors The neighbors to look at to see which one has the most
   *     most ink savings.
   * @return The best neighbor to bundle with and the bundle that will contain
   *     both the edge and the neighbor if they are put together.
   */
  NeighborAndBundle findBestNeighborForEdge(Edge &edge,
                                            std::vector<Edge *> &neighbors);

  /**
   * Gets the source point that is closes to the points, but that still keeps
   * the angle
   * in the boundaries. Look at TAN_MAX_ANGLE to find out which angle is this.
   */
  Point getMinimumSourcePoint(std::vector<Point> points, Point sourcePoint,
                              Point targetPoint);

  /**
   * The tangent for the maximum turning angle. The tangent of 40 degrees is
   * 0.83909.
   */
  double TAN_MAX_ANGLE = 0.83909;

  /**
   * The precision to find the brent search minimum.
   */
  int BRENT_SEARCH_PRECISION = std::numeric_limits<double>::digits;

  /**
   * The maximum iterations of brent search.
   */
  boost::uintmax_t BRENT_SEARCH_MAX_ITERATIONS = 20;

  /**
   * The number of neighbors used for this graph. This is populated after the
   * edges are read
   * by the readEdges from file function.
   */
  int numNeighbors = 0;
};

#endif /* GRAPH_H_ */
