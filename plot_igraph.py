from igraph import *
import sys


def plotFromNodeAndEdgesFile(nodesFilePath, edgesFilePath):
    graph = Graph()
    layout = []
    with open(nodesFilePath) as nodesFile:
        for line in nodesFile:
            parts = line.split()
            graph.add_vertex(parts[0])
            layout.append((float(parts[1]), float(parts[2])))
    with open(edgesFilePath) as edgesFile:
        for line in edgesFile:
            parts = line.split(":")
            graph.add_edge(parts[0], parts[1], weight=parts[2])
    plot(graph, layout=layout, vertex_size=3, bbox=(1000, 1000), edge_curved=0)


if __name__ == "__main__":
    if (len(sys.argv) < 3):
        printf("Usage python plot_igraph.py path_nodes.txt path_edges.txt")
    else:
        plotFromNodeAndEdgesFile(sys.argv[1], sys.argv[2])
