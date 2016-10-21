#include "main.h"
#include "Graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 2 && argc != 3) {
        fprintf(stderr, "usage: %s path_edges.txt path_output.txt\n", *argv);
        return 1;
    }

    Graph graph = Graph();
    graph.readEdgesFromFile(argv[1]);
    return 0;
}
