using namespace std;
#include "main.h"
#include "Graph.h"

double getDistanceBetweenPoints(Point point1, Point point2) {
  return sqrt(pow((point1.x - point2.x), 2) + pow((point1.y - point2.y), 2));
}

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3) {
    fprintf(stderr, "usage: %s path_edges.txt path_output.txt\n", *argv);
    return 1;
  }
  Graph *graph = new Graph();
  graph->readEdgesFromFile(argv[1]);
  graph->doMingle();

  return 0;
}
