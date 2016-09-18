#include "main.h"
#include "EdgeBundleTree.h"

using namespace std;

EdgeBundler *bundler;

const int WIDTH = 2000;
const int HEIGHT = 1000;
const double ZOOM_CONST = 1.9;

double dataWidth;
double dataHeight;
Point dataCenter;

/**
 * Calculates the dataWidth, dataHeight and dataCenter based on the edges.
 */
void calculateDataBounds(std::vector<EdgeNode> &edges) {
    double right = 0, top = 0, left = 0, bottom = 0;
    for (auto &e : edges) {
        Point &p1 = *e.getS();
        Point &p2 = *e.getT();
        left = p1.x < left ? p1.x : left;
        right = p2.x > right ? p2.x : right;
        bottom = p1.y < bottom ? p1.y : bottom;
        bottom = p2.y < bottom ? p2.y : bottom;
        top = p1.y > top ? p1.y : top;
        top = p2.y > top ? p2.y : top;
    }
    dataWidth = right - left;
    dataHeight = top - bottom;
    dataCenter.x = (float) (dataWidth / 2 + left);
    dataCenter.y = (float) (dataHeight / 2 + bottom);
}

void initBundler(std::vector<Point> &points, std::vector<EdgeNode> &edges) {
    calculateDataBounds(edges);
    int numNeighbors;
    if (edges.size() < 10) {
        numNeighbors = edges.size() - 1;
    } else if (edges.size() < 100) {
        numNeighbors = edges.size() / 10;
    } else {
        numNeighbors = 20;
    }
    bundler = new EdgeBundler(&points, &edges, numNeighbors, 0.8f);
    printf("Created Edge Bundler\n");
    bundler->doMingle();
    printf("Finished mingling");
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
    bundler->getTree().write("node.txt", "edges.txt");

    printf("sizes are %ld\n", nodes.size());
    printf("sizes are %ld\n", edges.size());
    return 0;
}
