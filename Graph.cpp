/*
 * Graph implementation file.
 */

#include "Graph.h"
#include "GetTotalInkWhenMovedByFraction.h"
#include "InkBundleAndBundleOperationType.h"
#include "Point.h"
#include </usr/include/boost/math/tools/minima.hpp>
#include <cassert>
#include <fstream>
#include <stdio.h>
#include <string.h>

Graph::Graph() {
  pointToPointId = new std::unordered_map<Point, std::string, PointHasher>();
  pointIdToPoint = new std::unordered_map<std::string, Point>();
}

void Graph::readVerticesAndEdges(const char *verticesFilePath,
                                 const char *edgesFilePath) {
  _edges.clear();

  std::ifstream verticesStream(verticesFilePath);

  if (!verticesStream.is_open()) {
    throw std::runtime_error(
        "Could not open the vertices file. Check vertices file path.");
  }
  int numPoints;
  verticesStream >> numPoints;

  if (numPoints == 0) {
    throw std::runtime_error("Error reading the number of vertices. Check the "
                             "vertices file format.");
  }
  printf("Reading %d vertices. \n", numPoints);
  for (int i = 0; i < numPoints; i++) {
    readNextPointInFile(verticesStream);
  }
  verticesStream.close();

  std::ifstream edgesStream(edgesFilePath);
  if (!edgesStream.is_open()) {
    throw std::runtime_error(
        "Could not open the edges file. Check edges file path.");
  }
  int numEdges;
  edgesStream >> numEdges;
  if (numEdges == 0) {
    throw std::runtime_error(
        "Error reading the number of edges. Check the edges file format.");
  }
  if (numEdges == 0) {
    throw "Error reading the number of edges. Check the edges file format.";
  }
  printf("Reading %d edges.\n", numEdges);
  _edges.reserve(numEdges);
  for (int i = 0; i < numEdges; ++i) {
    readNextEdgeInFile(edgesStream);
  }
  edgesStream.close();
  adjustNumNeighbors();
}

void Graph::readNextPointInFile(std::ifstream &verticesStream) {
  float xCoord;
  float yCoord;
  std::string pointIdString;
  try {
    verticesStream >> pointIdString >> xCoord >> yCoord;
  } catch (...) {
    throw std::runtime_error("Error reading from the vertices file. Check the "
                             "vertices file format.");
  }

  if (pointIdString == "") {
    throw std::runtime_error("Read a blank vertex id. Please check that the "
                             "number of vertices in the first line is "
                             "correct.");
  }
  Point point = {xCoord, yCoord};
  pointIdToPoint->insert(std::make_pair(pointIdString, point));
  pointToPointId->insert(std::make_pair(point, pointIdString));
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
  FILE *pointsFilePointer = fopen(pointsFilePath, "w");
  FILE *edgesfilePointer = fopen(edgesFilePath, "w");
  for (Edge *edge : _edges) {
    writeEdges(pointsFilePointer, edgesfilePointer, nextPointId, *edge);
  }
}

void Graph::writePoints(FILE *pointsFilePointer, Edge &edge, int &nextPointId) {
  Point sourcePoint = edge.getSource();
  Point targetPoint = edge.getTarget();
  addPointIfNotInMap(pointsFilePointer, sourcePoint, nextPointId);
  addPointIfNotInMap(pointsFilePointer, targetPoint, nextPointId);
}

void Graph::addPointIfNotInMap(FILE *pointsFilePointer, Point point,
                               int &nextPointId) {
  std::unordered_map<Point, std::string, PointHasher>::const_iterator result =
      pointToPointId->find(point);
  if (result == pointToPointId->end()) {
    std::string newPointId =
        "mingledBundleVertexId" + std::to_string(nextPointId);
    pointToPointId->insert(std::make_pair(point, newPointId));
    fprintf(pointsFilePointer, "%s %.4f %.4f\n", newPointId.c_str(), point.x,
            point.y);
    nextPointId++;
  }
}

void Graph::writeEdges(FILE *pointsFilePointer, FILE *edgesfilePointer,
                       int &nextPointId, Edge &edge) {
  writePoints(pointsFilePointer, edge, nextPointId);
  Point sourcePoint = edge.getSource();
  Point targetPoint = edge.getTarget();
  if (!edge.hasParent()) {
    fprintf(edgesfilePointer, "%s %s %d\n",
            pointToPointId->at(sourcePoint).c_str(),
            pointToPointId->at(targetPoint).c_str(), edge.getWeight());
  }
  for (Edge *childPointer : edge.getChildren()) {
    Edge &child = *childPointer;
    writePoints(pointsFilePointer, child, nextPointId);
    fprintf(edgesfilePointer, "%s %s %d\n",
            pointToPointId->at(child.getSource()).c_str(),
            pointToPointId->at(sourcePoint).c_str(), childPointer->getWeight());
    fprintf(edgesfilePointer, "%s %s %d\n",
            pointToPointId->at(targetPoint).c_str(),
            pointToPointId->at(child.getTarget()).c_str(),
            childPointer->getWeight());
    if (child.hasChildren()) {
      writePoints(pointsFilePointer, child, nextPointId);
      writeEdges(pointsFilePointer, edgesfilePointer, nextPointId, child);
    }
  }
}

void Graph::readNextEdgeInFile(std::ifstream &edgesStream) {
  std::string pointIdOne;
  std::string pointIdTwo;

  edgesStream >> pointIdOne >> pointIdTwo;
  std::unordered_map<std::string, Point>::const_iterator pointOneIterator =
      pointIdToPoint->find(pointIdOne);
  std::unordered_map<std::string, Point>::const_iterator pointTwoIterator =
      pointIdToPoint->find(pointIdTwo);
  if (pointIdOne == "" || pointIdTwo == "") {
    throw std::runtime_error("Read a blank vertex id. Please check that the "
                             "number of edges in the first line is correct.");
  }
  if (pointOneIterator == pointIdToPoint->end()) {
    throw std::runtime_error("Could not find point with pointId " + pointIdOne +
                             " in the vertices file.");
  }
  if (pointTwoIterator == pointIdToPoint->end()) {
    throw std::runtime_error("Could not find point with pointId " + pointIdTwo +
                             " in the vertices file.");
  }
  Edge *node = new Edge(pointOneIterator->second, pointTwoIterator->second);
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

EdgeAndBundleOperationType Graph::getBundledEdge(Edge &edge, Edge &neighbor) {
  try {
    if (!edge.hasParent() && !neighbor.hasParent()) {
      return {getBundledEdgeOfTwoUnbundledEdges(edge, neighbor),
              BundleOperationType::NEW_BUNDLE};
    } else if (edge.getParent() == neighbor.getParent()) {
      throw "They already are in the same bundle.";
    } else if (edge.hasParent() && neighbor.hasParent()) {
      return {mergeTwoBundles(*edge.getParent(), *neighbor.getParent()),
              BundleOperationType::MERGE_BUNDLES};
    } else if (edge.hasParent()) {
      return {addEdgeToBundle(neighbor, *edge.getParent()),
              BundleOperationType::ADD_NEIGHBOR_TO_PARENT_OF_EDGE};
    } else {
      return {addEdgeToBundle(edge, *neighbor.getParent()),
              BundleOperationType::ADD_EDGE_TO_PARENT_OF_NEIGHBOR};
    }
  } catch (const char *msg) {
    throw "The edges are unable to bundle together.";
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
        try {
          BundleOperation bestBundleOperation =
              findBestNeighborForEdge(edge, neighbors);
          processBundleOperation(bestBundleOperation);
          numBundled++;
        } catch (const char *msg) {
          // Unable to find best bundle operation.
          continue;
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

void Graph::processBundleOperation(BundleOperation bundleOperation) {
  switch (bundleOperation.operationType) {
  case NEW_BUNDLE:
    bundleOperation.edge.setParent(bundleOperation.bundle);
    bundleOperation.neighborPointer->setParent(bundleOperation.bundle);
    break;
  case MERGE_BUNDLES:
    removeParentEdge(bundleOperation.edge);
    removeParentEdge(*bundleOperation.neighborPointer);
    break;
  case ADD_NEIGHBOR_TO_PARENT_OF_EDGE:
    removeParentEdge(bundleOperation.edge);
    break;
  case ADD_EDGE_TO_PARENT_OF_NEIGHBOR:
    removeParentEdge(*bundleOperation.neighborPointer);
    break;
  default:
    break;
  }
  for (Edge *edge : bundleOperation.bundle->getChildren()) {
    edge->setParent(bundleOperation.bundle);
  }
  _parentEdges.push_back(bundleOperation.bundle);
}

void Graph::removeParentEdge(Edge &edge) {
  Edge *parent = edge.getParent();
  _parentEdges.erase(
      std::remove(_parentEdges.begin(), _parentEdges.end(), parent),
      _parentEdges.end());
  delete parent;
  edge.clearParent();
}

BundleOperation Graph::findBestNeighborForEdge(Edge &edge,
                                               std::vector<Edge *> &neighbors) {
  Edge *bestNeighborPointer = nullptr;
  if (neighbors.size() == 0) {
    throw "There are no neighbors.";
  }
  Edge bestBundle = *neighbors[0];
  BundleOperationType bestOperation;
  double maxInkSaved = 0.0;
  for (Edge *neighborPointer : neighbors) {
    try {
      InkBundleAndBundleOperationType inkSavedBundleAndOperationType =
          estimateInkSavings(edge, *neighborPointer);
      if (inkSavedBundleAndOperationType.ink > maxInkSaved) {
        maxInkSaved = inkSavedBundleAndOperationType.ink;
        bestBundle = inkSavedBundleAndOperationType.bundle;
        bestOperation = inkSavedBundleAndOperationType.bundleOperationType;
        bestNeighborPointer = neighborPointer;
      }
    } catch (const char *msg) {
      continue;
    }
  }
  // If no maximum ink saved, throw.
  if (maxInkSaved < 0.0001) {
    throw "No ink saved.";
  }
  return {bestOperation, edge, bestNeighborPointer, new Edge(bestBundle),
          maxInkSaved};
}

InkBundleAndBundleOperationType Graph::estimateInkSavings(Edge &edge1,
                                                          Edge &edge2) {
  if ((edge1.hasParent() && edge2.hasParent()) &&
      (edge1.getParent() == edge2.getParent())) {
    throw "There are no ink savings for these two edges because they are in "
          "the same bundle.";
  }
  if (Point::getDistanceBetweenPoints(edge1.getSource(), edge2.getSource()) >
      Point::getDistanceBetweenPoints(edge1.getSource(), edge2.getTarget())) {
    throw "There are no ink savings for these two edges because the source "
          "points are too far away.";
  }
  double currentInk = getCurrentInkOfTwoEdges(edge1, edge2);
  try {
    EdgeAndBundleOperationType edgeAndBundle = getBundledEdge(edge1, edge2);
    double inkSaving = currentInk - edgeAndBundle.edge.getInk();
    return {inkSaving, edgeAndBundle.edge, edgeAndBundle.type};
  } catch (const char *msg) {
    throw "There are no ink savings for these two edges.";
  }
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

Edge Graph::getBundledEdgeOfTwoUnbundledEdges(Edge &edge1, Edge &edge2) {
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
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    Edge parentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
    parentEdge.addChild(&edge1);
    parentEdge.addChild(&edge2);
    return parentEdge;
  } catch (const char *msg) {
    throw "Unable to merge two edges because there are no meeting points in "
          "the allowed angle.";
  }
}

Edge Graph::mergeTwoBundles(Edge &edge1, Edge &edge2) {
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
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    Edge newParentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
    for (Edge *childEdge : edge1.getChildren()) {
      newParentEdge.addChild(childEdge);
    }
    for (Edge *childEdge : edge2.getChildren()) {
      newParentEdge.addChild(childEdge);
    }
    return newParentEdge;
  } catch (const char *msg) {
    throw "Unable to merge two edges because there are no meeting points in "
          "the allowed angle.";
  }
}

Edge Graph::addEdgeToBundle(Edge &edge1, Edge &bundle) {
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
  try {
    Point adjustedSourcePoint =
        getMinimumSourcePoint(sourcePoints, sourceCentroid, targetCentroid);
    Point adjustedTargetPoint = getMinimumSourcePoint(
        targetPoints, targetCentroid, adjustedSourcePoint);
    Point sourceMeetingPoint = brentSearchMeetingPoint(
        adjustedSourcePoint, adjustedTargetPoint, sourcePoints, pointWeights);
    Point targetMeetingPoint = brentSearchMeetingPoint(
        adjustedTargetPoint, sourceMeetingPoint, targetPoints, pointWeights);
    Edge newParentEdge = Edge(sourceMeetingPoint, targetMeetingPoint);
    for (Edge *childEdge : bundle.getChildren()) {
      newParentEdge.addChild(childEdge);
    }
    newParentEdge.addChild(&edge1);
    return newParentEdge;
  } catch (const char *msg) {
    throw "Unable to merge two edges because there are no meeting points in "
          "the allowed angle.";
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
          "will make the point stay within the angle.";
  }
  Point addition = moveUnitVector * maximum;
  return sourcePoint + addition;
}
