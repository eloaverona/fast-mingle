#include "EdgeGraph.h"

void EdgeGraph::readEdgesFromFile(const char *edgesFilePath) {
    _nodes.clear();
    _points.clear();

    FILE *filePointer = fopen(edgesFilePath, "r");
    assert(filePointer != NULL);

    int numEdges;
    fscanf(filePointer, "%i", &numEdges);
    _nodes.reserve(numEdges);
    std::unordered_set<Point, PointHasher> seen;
    PointId nextPointId = 0;
	for (int i = 0; i < numEdges; ++i) {
		readNextEdgeInFile(filePointer, seen, &nextPointId);
	}

    fclose(filePointer);
}

void EdgeGraph::readNextEdgeInFile(FILE *filePointer, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId) {
		Point pointA;
		Point pointB;

		fscanf(filePointer, "%f", &pointA.x);
		fscanf(filePointer, "%f", &pointA.y);
		fscanf(filePointer, "%f", &pointB.x);
		fscanf(filePointer, "%f", &pointB.y);

		setIdOfPoint(&pointA, seen, nextPointId);
		setIdOfPoint(&pointB, seen, nextPointId);

		EdgeNode *node = new EdgeNode(&pointA, &pointB);
		_nodes.push_back(node);
}

void EdgeGraph::setIdOfPoint(Point *point, std::unordered_set<Point, PointHasher> seen, PointId *nextPointId){
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

void EdgeGraph::rebuildAnnIndex() {
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

