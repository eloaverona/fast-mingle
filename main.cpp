#include "main.h"
#include "EdgeBundleTree.h"

using namespace std;

EdgeBundler *bundler;

void initBundler(std::vector<Point> &points, std::vector<EdgeNode> &edges) {
    int numNeighbors;
    if (edges.size() < 10) {
        numNeighbors = edges.size() - 1;
    } else if (edges.size() < 100) {
        numNeighbors = edges.size() / 10;
    } else {
        numNeighbors = 20;
    }
    bundler = new EdgeBundler(&points, &edges, numNeighbors);
}

int main(int argc, char *argv[]) {
    if (argc != 2 && argc != 3) {
        fprintf(stderr, "usage: %s path_edges.txt path_output.txt\n", *argv);
        return 1;
    }
    std::vector<Point> nodes;
    std::vector<EdgeNode> edges;
    BaseNode::ReadEdges(argv[1], nodes, edges);
    initBundler(nodes, edges);
    bundler->doMingle();
    bundler->getTree().write("nodes.txt", "edges.txt");
    return 0;
}
