using namespace std;
#include "main.h"
#include "Graph.h"

int main(int argc, char *argv[]) {
  if (argc != 4) {
    fprintf(stderr, "usage: %s path_edges.txt path_output_nodes.txt "
                    "path_output_edges.txt\n",
            *argv);
    return 1;
  }
  Graph *graph = new Graph();
  printf("Created graph.\n");
  graph->readEdgesFromFile(argv[1]);
  graph->doRecursiveMingle();
  graph->writePointsAndEdges(argv[2], argv[3]);
  return 0;
}
