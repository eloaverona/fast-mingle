using namespace std;
#include "main.h"
#include "Graph.h"

int main(int argc, char *argv[]) {
  if (argc != 5) {
    fprintf(stderr, "usage: %s path_input_vertices.txt path_input_edges.txt "
                    "path_output_vertices.txt "
                    "path_output_edges.txt\n",
            *argv);
    return 1;
  }
  Graph *graph = new Graph();
  printf("Created graph.\n");
  graph->readVerticesAndEdges(argv[1], argv[2]);
  graph->doRecursiveMingle();
  graph->writePointsAndEdges(argv[3], argv[4]);
  return 0;
}
