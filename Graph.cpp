#include "Graph.h"
#include <cmath>
#include <cassert>

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

		Edge *node = new Edge(point1, point2);
		_edges.push_back(node);
}

void Graph::setIdOfPoint(Point *point, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId){
	std::unordered_set<Point, PointHasher>::const_iterator foundPoint = seen.find(*point);
	if (foundPoint == seen.end()){
		point->id = *nextPointId;
		_points.push_back(*point);
		seen.insert(*point);
		++*nextPointId;
	} else {
		point->id = foundPoint->id;
	}
}

void Graph::rebuildAnnIndex() {
    if (annTree != nullptr) delete(annTree);
    if (annPoints != nullptr)  annDeallocPts(annPoints);
    annPoints = annAllocPts(_nodes.size(), 4);
	assert(annPoints != nullptr);
	for (int i = 0; i < _nodes.size(); ++i) {
		EdgeNode *edge = _nodes[i];
		ANNpoint p = annPoints[i];
		p[0] = edge->getPointOne()->x;
		p[1] = edge->getPointOne()->y;
		p[2] = edge->getPointTwo()->x;
		p[3] = edge->getPointTwo()->y;
	}
	annTree = new ANNkd_tree(annPoints, _nodes.size(), 4);
	assert(annTree != nullptr);
}

void Graph::doMingle() {}

double Graph::estimateSavedInkWhenTwoEdgesBundled(EdgeNode *node1, EdgeNode *node2) {
	int totalWeight = node1->getWeight() + node2->getWeight();
	Point meetingPointOne = getMeetingPointOneForNodes(node1, node2);
	Point meetingPointTwo = getMeetingPointTwoForNodes(node1, node2);
	EdgeNode node = EdgeNode(meetingPointOne, meetingPointTwo);
	node.addChildAtPointOne(node1);
	node.addChildAtPointTwo(node2);
	Point differenceVector = {meetingPointTwo - meetingPointOne};
	double differenceVectorLength = differenceVector.norm();
	goldenSectionSearch(
			-differenceVectorLength * LOOKUP_RANGE,
			differenceVectorLength * LOOKUP_RANGE,
			differenceVectorLength * TOLERANCE_RANGE,
			meetingPointOne,
			differenceVector,
			meetingPointOne->getChildrenAtPointOnePoints(),
			meetingPointTwo);
}

double Graph::goldenSectionSearch(double lowerBound, double upperBound, double tolerance, Point meetingPointToOptimize, Point meetingPointMoveDirection, std::vector<Point*> meetingPointToOptimizeChildren,  Point otherMeetingPoint) {
	double goldenDifference = (upperBound - lowerBound) / GOLD_RATIO;
	double newUpperBound = upperBound - goldenDifference;
	double newLowerBound = lowerBound + goldenDifference;
	int loopCountdown = GOLD_SECTION_SEARCH_MAX_LOOPS;
	while (abs(newUpperBound - newLowerBound) > tolerance && loopCountdown > 0) {
		double inkForNewUpperBound = getInkForBound(meetingPointToOptimize, meetingPointMoveDirection, newUpperBound, otherMeetingPoint, meetingPointToOptimizeChildren);
		double inkForNewLowerBound = getInkForBound(meetingPointToOptimize, meetingPointMoveDirection, newLowerBound, otherMeetingPoint, meetingPointToOptimizeChildren);
		if (inkForNewUpperBound < inkForNewLowerBound) {
			upperBound = newUpperBound;
		} else {
			lowerBound = newLowerBound;
		}
		double newUpperBound = upperBound - goldenDifference;
		double newLowerBound = lowerBound + goldenDifference;
		loopCountdown--;
	}
}

double Graph::getInkForBound(Point *meetingPointToOptimize, Point meetingPointMoveDirection, double meetingPointMoveFactor, Point *otherMeetingPoint, std::vector<Point*> children) {
	Point newCentroid = meetingPointToOptimize + (meetingPointMoveDirection * meetingPointMoveFactor);
	double inkSum = 0.0;
	for(Point* child : children) {
		double distance = sqrt((child->x - newCentroid->x) ^ 2 - (child->y - newCentroid->y) ^ 2);
		inkSum += inkSum;
	}
	inkSum += (otherMeetingPoint - newCentroid).norm();
	return inkSum;
}



