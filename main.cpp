#include "main.h"
#include "EdgeGraph.h"

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 2 && argc != 3) {
        fprintf(stderr, "usage: %s path_edges.txt path_output.txt\n", *argv);
        return 1;
    }

    EdgeGraph graph = EdgeGraph();
    graph.readEdgesFromFile(argv[1]);


    return 0;
}
