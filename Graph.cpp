/*
 * Graph implementation file.
 */

#include "Graph.h"
#include "GetTotalInkWhenMovedByFraction.h"
#include "Point.h"
#include </usr/include/boost/math/tools/minima.hpp>
#include <cassert>

Graph::Graph() {}

void Graph::readEdgesFromFile(const char *edgesFilePath) {
  _edges.clear();

  FILE *filePointer = fopen(edgesFilePath, "r");
  assert(filePointer != nullptr);

  int numEdges;
  fscanf(filePointer, "%i", &numEdges);
  _edges.reserve(numEdges);
  for (int i = 0; i < numEdges; ++i) {
    readNextEdgeInFile(filePointer);
  }
  fclose(filePointer);
  numNeighbors = _edges.size() - 1;
}

void Graph::readNextEdgeInFile(FILE *filePointer) {
  float pointAx;
  float pointAy;
  float pointBx;
  float pointBy;
  fscanf(filePointer, "%f", &pointAx);
  fscanf(filePointer, "%f", &pointAy);
  fscanf(filePointer, "%f", &pointBx);
  fscanf(filePointer, "%f", &pointBy);
  Edge *node = new Edge({pointAx, pointAy}, {pointBx, pointBy});
  _edges.push_back(node);
}

void Graph::rebuildIndex() {
  if (annTree != nullptr)
    delete (annTree);
  if (annPoints != nullptr)
    annDeallocPts(annPoints);
  annPoints = annAllocPts(_edges.size(), 4);
  assert(annPoints != nullptr);
  for (int i = 0; i < _edges.size(); ++i) {
    Edge edge = *_edges[i];
    ANNpoint p = annPoints[i];
    p[0] = edge.getSource().x;
    p[1] = edge.getSource().y;
    p[2] = edge.getTarget().x;
    p[3] = edge.getTarget().y;
  }
  annTree = new ANNkd_tree(annPoints, _edges.size(), 4);
  assert(annTree != nullptr);
}

void Graph::fillNeighborArrayWithEdgeNeighbors(Edge &target,
                                               std::vector<Edge *> &neighbors) {
  ANNidx indices[numNeighbors];
  ANNdist dists[numNeighbors];
  ANNcoord queryPoint[4];

  queryPoint[0] = target.getSource().x;
  queryPoint[1] = target.getSource().y;
  queryPoint[2] = target.getTarget().x;
  queryPoint[3] = target.getTarget().y;
  annTree->annkSearch(queryPoint, numNeighbors, indices, dists);
  neighbors.resize(numNeighbors);
  for (int j = 0; j < numNeighbors; ++j) {
    neighbors[j] = _edges[indices[j]];
  }
}

Edge Graph::getBundledEdge(Edge &edge1, Edge &edge2) {
  if (!edge1.hasParent() && !edge2.hasParent()) {
    return getBundledEdgeOfTwoUnbundledEdges(edge1, edge2);
  } else if (edge1.getParent() == edge2.getParent()) {
    return *edge1.getParent();
  } else if (edge1.hasParent() && edge2.hasParent()) {
    return mergeTwoBundles(*edge1.getParent(), *edge2.getParent());
  } else if (edge1.hasParent()) {
    return addEdgeToBundle(edge2, *edge1.getParent());
  } else {
    return addEdgeToBundle(edge1, *edge2.getParent());
  }
}

void Graph::doMingle() {
  std::vector<Edge *> neighbors;
  int numBundled = 0;
  rebuildIndex();
  do {
    numBundled = 0;
    for (Edge *edgePointer : _edges) {
      Edge edge = *edgePointer;
      if (!edgePointer->hasParent()) {
        fillNeighborArrayWithEdgeNeighbors(edge, neighbors);
        Edge *bestNeighborPointer = findBestNeighborForEdge(edge, neighbors);
        if (bestNeighborPointer != nullptr) {
          putTwoEdgesOnSameBundle(edgePointer, bestNeighborPointer);
          numBundled++;
        }
      }
    }
  } while (numBundled > 0);
}

void Graph::putTwoEdgesOnSameBundle(Edge* edge1Pointer, Edge* edge2Pointer) {
  Edge edge1 = *edge1Pointer;
  Edge edge2 = *edge2Pointer;
  Edge *parent = nullptr;
  if (!edge1.hasParent() && !edge2.hasParent()) {
	parent = new Edge(getBundledEdgeOfTwoUnbundledEdges(edge1, edge2));
  } else if (edge1.getParent() == edge2.getParent()) {
	// They are both in the same bundle so do nothing
	return;
  } else if (edge1.hasParent() && edge2.hasParent()) {
    deleteParentEdge(edge1.getParent());
    deleteParentEdge(edge2.getParent());
	parent = new Edge(mergeTwoBundles(*edge1.getParent(), *edge2.getParent()));
  } else if (edge1.hasParent()) {
	parent = new Edge(addEdgeToBundle(edge2, *edge1.getParent()));
	deleteParentEdge(edge1.getParent());
  } else {
	parent = new Edge(addEdgeToBundle(edge1, *edge2.getParent()));
	deleteParentEdge(edge2.getParent());
  }
  assert(parent != nullptr);
  edge1Pointer->setParent(parent);
  edge2Pointer->setParent(parent);
  _parentEdges.push_back(parent);
}

void Graph::deleteParentEdge(Edge* parent) {
	_parentEdges.erase(std::remove(_parentEdges.begin(), _parentEdges.end(), parent), _parentEdges.end());
	delete parent;
}


Edge* Graph::findBestNeighborForEdge(Edge &edge, std::vector<Edge *> &neighbors) {
	Edge *bestNeighborPointer = nullptr;
	double maxInkSaved = 0.0;
    for (Edge *neighborPointer : neighbors) {
      Edge neighbor = *neighborPointer;
      double inkSaved = estimateInkSavings(edge, neighbor);
      if (inkSaved > maxInkSaved) {
        maxInkSaved = inkSaved;
        bestNeighborPointer = neighborPointer;
      }
    }
    return bestNeighborPointer;
}

double Graph::estimateInkSavings(Edge &edge1, Edge &edge2) {
  // If both are bundled together, then there are no ink savings.
  if ( (edge1.hasParent() && edge2.hasParent()) && (edge1.getParent() == edge2.getParent())) {
    return 0.0;
  }
  // Calculate the total ink of each edge that belongs to.
  double currentInk = 0.0;
  if (edge1.hasParent()) {
    currentInk += edge1.getParent()->getInk();
  } else {
    currentInk += edge1.getInk();
  }
  if (edge2.hasParent()) {
    currentInk += edge2.getParent()->getInk();
  } else {
    currentInk += edge2.getInk();
  }
  Edge bundledEdge = getBundledEdge(edge1, edge2);
  return currentInk -bundledEdge.getInk();
}

Edge Graph::getBundledEdgeOfTwoUnbundledEdges(Edge &edge1, Edge &edge2) {
  std::vector<Point> sourcePoints;
  sourcePoints.push_back(edge1.getSource());
  sourcePoints.push_back(edge2.getSource());
  std::vector<Point> targetPoints;
  targetPoints.push_back(edge1.getTarget());
  targetPoints.push_back(edge2.getTarget());
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Point sourceMeetingPoint =
      brentSearchMeetingPoint(sourceCentroid, targetCentroid, sourcePoints);
  Point targetMeetingPoint =
      brentSearchMeetingPoint(targetCentroid, sourceCentroid, targetPoints);
  Edge parentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
  parentEdge.addChild(&edge1);
  parentEdge.addChild(&edge2);
  return parentEdge;
}

Edge Graph::mergeTwoBundles(Edge &edge1, Edge &edge2) {
  assert(edge1.hasChildren() && edge2.hasChildren());
  std::vector<Point> sourcePoints;
  std::vector<Point> targetPoints;
  for (Edge* childEdge : edge1.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
  }
  for (Edge* childEdge : edge2.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
  }
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Point sourceMeetingPoint =
      brentSearchMeetingPoint(sourceCentroid, targetCentroid, sourcePoints);
  Point targetMeetingPoint =
      brentSearchMeetingPoint(targetCentroid, sourceCentroid, targetPoints);
  Edge newParentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
  for (Edge* childEdge : edge1.getChildren()) {
    newParentEdge.addChild(childEdge);
  }
  for (Edge* childEdge : edge2.getChildren()) {
    newParentEdge.addChild(childEdge);
  }
  return newParentEdge;
}

Edge Graph::addEdgeToBundle(Edge &edge1, Edge &bundle) {
  assert(!edge1.hasChildren() && bundle.hasChildren());
  std::vector<Point> sourcePoints;
  std::vector<Point> targetPoints;
  for (Edge* childEdge : bundle.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
  }
  sourcePoints.push_back(edge1.getSource());
  targetPoints.push_back(edge1.getTarget());
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Point sourceMeetingPoint =
      brentSearchMeetingPoint(sourceCentroid, targetCentroid, sourcePoints);
  Point targetMeetingPoint =
      brentSearchMeetingPoint(targetCentroid, sourceCentroid, targetPoints);
  Edge newParentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
  for (Edge* childEdge : bundle.getChildren()) {
    newParentEdge.addChild(childEdge);
  }
  newParentEdge.addChild(&edge1);
  return newParentEdge;
}

Point Graph::getCentroid(std::vector<Point> &points) {
  float xSum = 0.0f;
  float ySum = 0.0f;
  int numPoints = 0;
  for (Point point : points) {
    xSum += point.x;
    ySum += point.y;
    numPoints += 1;
  }
  return {xSum / numPoints, ySum / numPoints};
}

Point Graph::brentSearchMeetingPoint(Point &sourcePoint, Point &targetPoint,
                                     std::vector<Point> &sourcePoints) {
  Point moveVector = targetPoint - sourcePoint;
  sourcePoints.push_back(targetPoint);
  GetTotalInkWhenMovedByFraction optimizationFunction =
      GetTotalInkWhenMovedByFraction(sourcePoint, moveVector, sourcePoints);
  std::pair<double, double> result = boost::math::tools::brent_find_minima(
      optimizationFunction, -BRENT_SEARCH_RANGE, BRENT_SEARCH_RANGE,
      BRENT_SEARCH_PRECISION, BRENT_SEARCH_MAX_ITERATIONS);
  Point diff = moveVector * result.first;
  return sourcePoint + diff;
}
