#include "Graph.h"
#include <cassert>
#include <cmath>
#include <unordered_set>

void Graph::readEdgesFromFile(const char *edgesFilePath) {
  _edges.clear();

  FILE *filePointer = fopen(edgesFilePath, "r");
  assert(filePointer != NULL);

  int numEdges;
  fscanf(filePointer, "%i", &numEdges);
  _edges.reserve(numEdges);
  for (int i = 0; i < numEdges; ++i) {
    readNextEdgeInFile(filePointer);
  }
  fclose(filePointer);
}

void Graph::readNextEdgeInFile(FILE *filePointer) {
  double pointAx;
  double pointAy;
  double pointBx;
  double pointBy;

  fscanf(filePointer, "%f", pointAx);
  fscanf(filePointer, "%f", pointAy);
  fscanf(filePointer, "%f", pointBx);
  fscanf(filePointer, "%f", pointBy);

  PointOrStar point1 = new PointOrStar(pointAx, pointAy);
  PointOrStar point2 = new PointOrStar(pointBx, pointBy);

  Edge node = new Edge(point1, point2);
  _edges.push_back(node);
}

void Graph::rebuildAnnIndex() {
  if (annTree != nullptr) delete (annTree);
  if (annPoints != nullptr) annDeallocPts(annPoints);
  annPoints = annAllocPts(_edges.size(), 4);
  assert(annPoints != nullptr);
  for (int i = 0; i < _edges.size(); ++i) {
    Edge *edge = _edges[i];
    ANNpoint p = annPoints[i];
    std::pair<PointOrStar, PointOrStar> points = edge->getPoints();
    p[0] = points.first.getX();
    p[1] = points.first.getY();
    p[2] = points.second.getX();
    p[3] = points.second.getY();
  }
  annTree = new ANNkd_tree(annPoints, _edges.size(), 4);
  assert(annTree != nullptr);
}

void Graph::doMingle() {}

double Graph::estimateInkWhenTwoEdgesBundled(Edge node1, Edge node2) {
  std::pair<PointOrStar, PointOrStar> sourcePoints;
  std::pair<PointOrStar, PointOrStar> targetPoints;
  if (getDistanceBetweenPoints(node1.getPoints().first,
                               node2.getPoints().first) <
      getDistanceBetweenPoints(node1.getPoints().first,
                               node2.getPoints().second)) {
    sourcePoints =
        std::make_pair(node1.getPoints().first, node1.getPoints().second);
    targetPoints =
        std::make_pair(node2.getPoints().second, node2.getPoints().first);
  } else {
    sourcePoints =
        std::make_pair(node1.getPoints().first, node1.getPoints().first);
    targetPoints =
        std::make_pair(node2.getPoints().second, node2.getPoints().second);
  }
  Edge bundledEdge = getBundledEdge(sourcePoints, targetPoints);
  return bundledEdge.getInk();
}

Edge Graph::getBundledEdge(std::pair<PointOrStar, PointOrStar> sourceSet,
                           std::pair<PointOrStar, PointOrStar> targetSet) {
  Point sourceCentroid = getCentroid(sourceSet);
  Point targetCentroid = getCentroid(targetSet);
  Point sourceMeetingPoint =
      goldenSectionSearch(GSS_SEARCH_RANGE_FRACTION, GSS_TOLERANCE_FRACTION,
                          sourceCentroid, targetCentroid, sourceSet);
  Point targetMeetingPoint =
      goldenSectionSearch(GSS_SEARCH_RANGE_FRACTION, GSS_TOLERANCE_FRACTION,
                          targetCentroid, sourceCentroid, targetSet);
  PointOrStar meetingPoint1 =
      PointOrStar(sourceMeetingPoint.x, sourceMeetingPoint.y);
  meetingPoint1.addChild(sourceSet.first);
  meetingPoint1.addChild(sourceSet.second);
  PointOrStar meetingPoint2 =
      PointOrStar(targetMeetingPoint.x, targetMeetingPoint.y);
  meetingPoint2.addChild(targetSet.first);
  meetingPoint2.addChild(targetSet.second);
  return Edge(meetingPoint1, meetingPoint2);
}

Point Graph::getCentroid(std::pair<PointOrStar, PointOrStar> set) {
  PointOrStar point1 = set.first;
  PointOrStar point2 = set.second;
  int combinedWeight = point1.getWeight() + point2.getWeight();
  double x = (point1.getX() * point1.getWeight() +
              point2.getX() * point2.getWeight()) /
             combinedWeight;
  double y = (point1.getY() * point1.getWeight() +
              point2.getY() * point2.getWeight()) /
             combinedWeight;
  return {x, y};
}

Point Graph::goldenSectionSearch(
    double searchRangeDelta, double tolerance, Point sourceCentroid,
    Point targetCentroid, std::pair<PointOrStar, PointOrStar> sourceSet) {
  double lowerBound = -searchRangeDelta;
  double upperBound = searchRangeDelta;
  Point differenceVector = targetCentroid - sourceCentroid;
  double goldenDifference = (upperBound - lowerBound) / GOLD_RATIO;
  double newUpperBound = upperBound - goldenDifference;
  double newLowerBound = lowerBound + goldenDifference;
  int loopCountdown = GSS_MAX_LOOPS;
  while (abs(newUpperBound - newLowerBound) > tolerance && loopCountdown > 0) {
    Point newUpperBoundPoint =
        sourceCentroid + differenceVector * newUpperBound;
    Point newLowerBoundPoint =
        sourceCentroid + differenceVector * newLowerBound;
    double inkForNewUpperBound =
        getInkForBoundPoint(newUpperBoundPoint, targetCentroid, sourceSet);
    double inkForNewLowerBound =
        getInkForBoundPoint(newLowerBoundPoint, targetCentroid, sourceSet);
    if (inkForNewUpperBound < inkForNewLowerBound) {
      upperBound = newUpperBound;
    } else {
      lowerBound = newLowerBound;
    }
    double newUpperBound = upperBound - goldenDifference;
    double newLowerBound = lowerBound + goldenDifference;
    loopCountdown--;
  }
  double mid = (newUpperBound + newLowerBound) / 2;
  return sourceCentroid + mid * differenceVector;
}

double Graph::getInkForBoundPoint(
    Point sourcePoint, Point targetPoint,
    std::pair<PointOrStar, PointOrStar> sourceSet) {
  double inkSum = 0.0;
  for (PointOrStar child : sourceSet.first.getChildren()) {
    double distance = getDistanceBetweenPoints(child, sourcePoint);
    inkSum += distance;
  }
  for (Point child : sourceSet.second.getChildren()) {
    double distance = getDistanceBetweenPoints(child, sourcePoint);
    inkSum += distance;
  }
  inkSum += getDistanceBetweenPoints(targetPoint, sourcePoint);
  return inkSum;
}
