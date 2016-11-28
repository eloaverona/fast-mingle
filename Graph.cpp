/*
 * Graph implementation file.
 */

#include "Graph.h"
#include "GetTotalInkWhenMovedByFraction.h"
#include "Point.h"
#include </usr/include/boost/math/tools/minima.hpp>
#include <cassert>
#include <stdio.h>

Graph::Graph() {}

void Graph::readEdgesFromFile(const char *edgesFilePath) {
  _edges.clear();

  FILE *filePointer = fopen(edgesFilePath, "r");
  assert(filePointer != nullptr);

  int numEdges;
  fscanf(filePointer, "%i", &numEdges);
  printf("Reading %d edges.\n", numEdges);
  _edges.reserve(numEdges);
  for (int i = 0; i < numEdges; ++i) {
    readNextEdgeInFile(filePointer);
  }
  fclose(filePointer);
  adjustNumNeighbors();
}

void Graph::adjustNumNeighbors() {
  if (_edges.size() < 10) {
    numNeighbors = _edges.size() - 1;
  } else if (_edges.size() < 100) {
    numNeighbors = _edges.size() / 10;
  } else {
    numNeighbors = 20;
  }
}

void Graph::writePointsAndEdges(const char *pointsFilePath,
                                const char *edgesFilePath) {
  int nextPointId = 0;
  std::unordered_map<Point, int, PointHasher> pointToPointIdMap;
  FILE *pointsFilePointer = fopen(pointsFilePath, "w");
  FILE *edgesfilePointer = fopen(edgesFilePath, "w");
  for (Edge *edge : _edges) {
    writeEdges(pointsFilePointer, edgesfilePointer, nextPointId, *edge,
               pointToPointIdMap);
  }
}

void Graph::writePoints(
    FILE *pointsFilePointer, Edge &edge, int &nextPointId,
    std::unordered_map<Point, int, PointHasher> &pointToPointIdMap) {
  Point sourcePoint = edge.getSource();
  Point targetPoint = edge.getTarget();
  addPointIfNotInMap(pointsFilePointer, sourcePoint, nextPointId,
                     pointToPointIdMap);
  addPointIfNotInMap(pointsFilePointer, targetPoint, nextPointId,
                     pointToPointIdMap);
}

void Graph::addPointIfNotInMap(
    FILE *pointsFilePointer, Point point, int &nextPointId,
    std::unordered_map<Point, int, PointHasher> &pointToPointIdMap) {
  std::unordered_map<Point, int, PointHasher>::const_iterator result =
      pointToPointIdMap.find(point);
  if (result == pointToPointIdMap.end()) {
    pointToPointIdMap[point] = nextPointId;
    fprintf(pointsFilePointer, "%d %.4f %.4f\n", nextPointId, point.x, point.y);
    nextPointId++;
  }
}

void Graph::writeEdges(
    FILE *pointsFilePointer, FILE *edgesfilePointer, int &nextPointId,
    Edge &edge,
    std::unordered_map<Point, int, PointHasher> &pointToPointIdMap) {
  writePoints(pointsFilePointer, edge, nextPointId, pointToPointIdMap);
  Point sourcePoint = edge.getSource();
  Point targetPoint = edge.getTarget();
  if (!edge.hasParent()) {
    fprintf(edgesfilePointer, "%d:%d:%d\n", pointToPointIdMap[sourcePoint],
            pointToPointIdMap[targetPoint], edge.getWeight());
  }
  for (Edge *childPointer : edge.getChildren()) {
    Edge &child = *childPointer;
    writePoints(pointsFilePointer, child, nextPointId, pointToPointIdMap);
    fprintf(edgesfilePointer, "%d:%d:%d\n",
            pointToPointIdMap[child.getSource()],
            pointToPointIdMap[sourcePoint], childPointer->getWeight());
    fprintf(edgesfilePointer, "%d:%d:%d\n", pointToPointIdMap[targetPoint],
            pointToPointIdMap[child.getTarget()], childPointer->getWeight());
    if (child.hasChildren()) {
      writePoints(pointsFilePointer, child, nextPointId, pointToPointIdMap);
      writeEdges(pointsFilePointer, edgesfilePointer, nextPointId, child,
                 pointToPointIdMap);
    }
  }
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
    Edge &edge = *_edges[i];
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

Edge *Graph::getBundledEdge(Edge &edge1, Edge &edge2) {
  if (!edge1.hasParent() && !edge2.hasParent()) {
    return getBundledEdgeOfTwoUnbundledEdges(edge1, edge2);
  } else if (edge1.getParent() == edge2.getParent()) {
    return edge1.getParent();
  } else if (edge1.hasParent() && edge2.hasParent()) {
    return mergeTwoBundles(*edge1.getParent(), *edge2.getParent());
  } else if (edge1.hasParent()) {
    return addEdgeToBundle(edge2, *edge1.getParent());
  } else {
    return addEdgeToBundle(edge1, *edge2.getParent());
  }
}

int Graph::doMingle() {
  std::vector<Edge *> neighbors;
  int numBundled = 0;
  int numIterations = 0;
  int loopNum = 0;
  rebuildIndex();
  do {
    if (loopNum % 100 == 0) {
      printf("Loop number %d...\n", numIterations);
    }
    numBundled = 0;
    for (Edge *edgePointer : _edges) {
      Edge &edge = *edgePointer;
      if (!edgePointer->hasParent()) {
        fillNeighborArrayWithEdgeNeighbors(edge, neighbors);
        NeighborAndBundle bestNeighborAndBundle =
            findBestNeighborForEdge(edge, neighbors);
        if (bestNeighborAndBundle.neighbor != nullptr) {
          putTwoEdgesOnSameBundle(*edgePointer, *bestNeighborAndBundle.neighbor,
                                  bestNeighborAndBundle.bundle);
          numBundled++;
        }
      }
    }
    numIterations++;
    loopNum++;
  } while (numBundled > 0);
  return numIterations;
}

void Graph::doRecursiveMingle() {
  int numIterations = doMingle();
  int recursiveCount = 1;
  while (numIterations > 1) {
    printf("Recursive step number %d.\n", recursiveCount);
    for (Edge *edge : _edges) {
      if (!edge->hasParent()) {
        _parentEdges.push_back(edge);
      }
    }
    _edges = _parentEdges;
    _parentEdges.clear();
    adjustNumNeighbors();
    numIterations = doMingle();
    recursiveCount++;
  }
}

void Graph::putTwoEdgesOnSameBundle(Edge &edge1, Edge &edge2, Edge *parent) {
  if (!edge1.hasParent() && !edge2.hasParent()) {
    edge1.setParent(parent);
    edge2.setParent(parent);
  } else if (edge1.getParent() == edge2.getParent()) {
    // They are both in the same bundle so do nothing
    if (parent != nullptr) {
      delete parent;
    }
    return;
  } else if (edge1.hasParent() && edge2.hasParent()) {
    removeParentEdge(edge1);
    removeParentEdge(edge2);
  } else if (edge1.hasParent()) {
    removeParentEdge(edge1);
  } else {
    removeParentEdge(edge2);
  }
  for (Edge *edge : parent->getChildren()) {
    edge->setParent(parent);
  }
  _parentEdges.push_back(parent);
}

void Graph::removeParentEdge(Edge &edge) {
  Edge *parent = edge.getParent();
  _parentEdges.erase(
      std::remove(_parentEdges.begin(), _parentEdges.end(), parent),
      _parentEdges.end());
  delete parent;
  edge.clearParent();
}

NeighborAndBundle
Graph::findBestNeighborForEdge(Edge &edge, std::vector<Edge *> &neighbors) {
  Edge *bestNeighborPointer = nullptr;
  Edge *bestBundlePointer = nullptr;
  double maxInkSaved = 0.0;
  for (Edge *neighborPointer : neighbors) {
    InkAndBundle inkSavedAndBundle = estimateInkSavings(edge, *neighborPointer);
    if (inkSavedAndBundle.ink > maxInkSaved) {
      maxInkSaved = inkSavedAndBundle.ink;
      delete bestBundlePointer;
      bestBundlePointer = inkSavedAndBundle.bundle;
      bestNeighborPointer = neighborPointer;
    } else {
      // We won't need this anymore so we can free it.
      delete inkSavedAndBundle.bundle;
    }
  }
  return {bestNeighborPointer, bestBundlePointer};
}

InkAndBundle Graph::estimateInkSavings(Edge &edge1, Edge &edge2) {
  // If both are bundled together, then there are no ink savings.
  if ((edge1.hasParent() && edge2.hasParent()) &&
      (edge1.getParent() == edge2.getParent())) {
    return {0.0, nullptr};
  }
  if (Point::getDistanceBetweenPoints(edge1.getSource(), edge2.getSource()) >
      Point::getDistanceBetweenPoints(edge1.getSource(), edge2.getTarget())) {
    return {0.0, nullptr};
  }
  double currentInk = getCurrentInkOfTwoEdges(edge1, edge2);
  Edge *bundledEdge = getBundledEdge(edge1, edge2);
  if (bundledEdge == nullptr) {
    return {0.0, nullptr};
  }
  double inkSaving = currentInk - bundledEdge->getInk();
  return {inkSaving, bundledEdge};
}

double Graph::getCurrentInkOfTwoEdges(Edge &edge1, Edge &edge2) {
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
  return currentInk;
}

Edge *Graph::getBundledEdgeOfTwoUnbundledEdges(Edge &edge1, Edge &edge2) {
  std::vector<Point> sourcePoints;
  sourcePoints.push_back(edge1.getSource());
  sourcePoints.push_back(edge2.getSource());
  std::vector<Point> targetPoints;
  targetPoints.push_back(edge1.getTarget());
  targetPoints.push_back(edge2.getTarget());
  std::vector<double> pointWeights;
  pointWeights.push_back(edge1.getInkWeight());
  pointWeights.push_back(edge2.getInkWeight());
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Edge *parentEdge = nullptr;
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    parentEdge = new Edge(sourceMeetingPoint, targetMeetingPoint);
    parentEdge->addChild(&edge1);
    parentEdge->addChild(&edge2);
    return parentEdge;
  } catch (const char *msg) {
    // No points
    if (parentEdge != nullptr) {
      delete parentEdge;
    }
    return nullptr;
  }
}

Edge *Graph::mergeTwoBundles(Edge &edge1, Edge &edge2) {
  assert(edge1.hasChildren() && edge2.hasChildren() && !edge1.hasParent() &&
         edge2.hasParent());
  std::vector<Point> sourcePoints;
  std::vector<Point> targetPoints;
  std::vector<double> pointWeights;
  for (Edge *childEdge : edge1.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
    pointWeights.push_back(childEdge->getInkWeight());
  }
  for (Edge *childEdge : edge2.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
    pointWeights.push_back(childEdge->getInkWeight());
  }
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Edge *newParentEdge = nullptr;
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    newParentEdge = new Edge(sourceMeetingPoint, targetMeetingPoint);
    for (Edge *childEdge : edge1.getChildren()) {
      newParentEdge->addChild(childEdge);
    }
    for (Edge *childEdge : edge2.getChildren()) {
      newParentEdge->addChild(childEdge);
    }
    return newParentEdge;
  } catch (const char *msg) {
    if (newParentEdge != nullptr) {
      delete newParentEdge;
    }
    return nullptr;
  }
}

Edge *Graph::addEdgeToBundle(Edge &edge1, Edge &bundle) {
  assert(!bundle.hasParent() && bundle.hasChildren());
  std::vector<Point> sourcePoints;
  std::vector<Point> targetPoints;
  std::vector<double> pointWeights;
  for (Edge *childEdge : bundle.getChildren()) {
    sourcePoints.push_back(childEdge->getSource());
    targetPoints.push_back(childEdge->getTarget());
    pointWeights.push_back(childEdge->getInkWeight());
  }
  sourcePoints.push_back(edge1.getSource());
  targetPoints.push_back(edge1.getTarget());
  pointWeights.push_back(edge1.getInkWeight());
  Point sourceCentroid = getCentroid(sourcePoints);
  Point targetCentroid = getCentroid(targetPoints);
  Edge *newParentEdge = nullptr;
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    newParentEdge = new Edge(sourceMeetingPoint, targetMeetingPoint);
    for (Edge *childEdge : bundle.getChildren()) {
      newParentEdge->addChild(childEdge);
    }
    newParentEdge->addChild(&edge1);
    return newParentEdge;
  } catch (const char *msg) {
    if (newParentEdge != nullptr) {
      delete newParentEdge;
    }
    return nullptr;
  }
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
                                     std::vector<Point> sourcePoints,
                                     std::vector<double> pointWeights) {
  assert(sourcePoints.size() == pointWeights.size());
  Point moveVector = targetPoint - sourcePoint;
  GetTotalInkWhenMovedByFraction optimizationFunction =
      GetTotalInkWhenMovedByFraction(sourcePoint, targetPoint, moveVector,
                                     sourcePoints, pointWeights);
  std::pair<double, double> result = boost::math::tools::brent_find_minima(
      optimizationFunction, 0.00, 1.00, BRENT_SEARCH_PRECISION,
      BRENT_SEARCH_MAX_ITERATIONS);
  Point diff = moveVector * result.first;
  return sourcePoint + diff;
}

Point Graph::getMinimumSourcePoint(std::vector<Point> points, Point sourcePoint,
                                   Point targetPoint) {
  assert(points.size() > 0);
  Point moveUnitVector = Point::getUnitVector(targetPoint - sourcePoint);
  Point perpendicularVector = Point::getPerPendicularVector(moveUnitVector);
  double maximum = -DBL_MAX;
  for (Point childPoint : points) {
    double totalMove = 0.00;
    if (!(abs(Point::getSlopeOfPoints(targetPoint, sourcePoint) -
              Point::getSlopeOfPoints(sourcePoint, childPoint)) < 0.1)) {
      double determinant = 1.00 / ((perpendicularVector.x * moveUnitVector.y) -
                                   (perpendicularVector.y * moveUnitVector.x));
      double perpendicularMoveFactor =
          determinant * (moveUnitVector.y * (sourcePoint.x - childPoint.x) -
                         moveUnitVector.x * (sourcePoint.y - childPoint.y));
      double moveFactor =
          -1.00 * determinant *
          (perpendicularVector.x * (sourcePoint.y - childPoint.y) -
           perpendicularVector.y * (sourcePoint.x - childPoint.x));
      double maximumMoveFactor = abs(perpendicularMoveFactor) / TAN_MAX_ANGLE;
      totalMove = moveFactor + maximumMoveFactor;
    }
    if (maximum < totalMove) {
      maximum = totalMove;
    }
  }
  // The point exceeds the target point, therefore throw an exception.
  if (Point::getDistanceBetweenPoints(targetPoint, sourcePoint) < maximum) {
    throw "The minimum point exceeds the target point. There is no point that "
          "will make the point sstay within the angle.";
  }
  Point addition = moveUnitVector * maximum;
  return sourcePoint + addition;
}
