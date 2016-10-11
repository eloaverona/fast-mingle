
#include "EdgeGraph.h"
#include <unordered_set>

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
		Point pointOne;
		Point pointTwo;

		fscanf(filePointer, "%f", &pointOne.x);
		fscanf(filePointer, "%f", &pointOne.y);
		fscanf(filePointer, "%f", &pointTwo.x);
		fscanf(filePointer, "%f", &pointTwo.y);

		setIdOfPoint(&pointOne, seen, nextPointId);
		setIdOfPoint(&pointTwo, seen, nextPointId);

		EdgeNode *node = new EdgeNode(pointOne.id, pointTwo.id);
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
