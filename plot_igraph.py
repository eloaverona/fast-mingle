from igraph import *
import sys
import argparse


def addVerticesFileToGraphAndLayout(verticesFilePath, graph, layout):
    with open(verticesFilePath) as verticesFile:
        for line in verticesFile:
            parts = line.split()
            if (len(parts) != 3):
                continue
            graph.add_vertex(parts[0])
            layout.append((float(parts[1]), -float(parts[2])))


def plotFromVerticesAndEdgesFile(verticesFilePath,
                                 edgesFilePath,
                                 vertex_size,
                                 moreVerticesFilePath=None):
    graph = Graph(directed=True)
    layout = []
    addVerticesFileToGraphAndLayout(verticesFilePath, graph, layout)
    if (moreVerticesFilePath):
        addVerticesFileToGraphAndLayout(moreVerticesFilePath, graph, layout)
    with open(edgesFilePath) as edgesFile:
        for line in edgesFile:
            parts = line.split()
            if (len(parts) == 3):
                graph.add_edge(
                    parts[0],
                    parts[1],
                    weight=parts[2],
                    width=float(parts[2]) / 10)
            elif (len(parts) == 2):
                graph.add_edge(parts[0], parts[1], weight=1)
            else:
                continue
    plot(
        graph,
        layout=layout,
        vertex_size=vertex_size,
        bbox=(1000, 1000),
        edge_curved=0,
        edge_arrow_size=0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='''Plots a graph from vertex file(s) and an edges file. The vertex files consist of three items on each line of the file that are space separated: A point id, an x coordinate and a y coordinate. The edges file consists of three items on each line of the file that are space separated: the first node id of the edge, the second node id of the edge and the weight of the edge.'''
    )
    parser.add_argument(
        '--vertices',
        required=True,
        type=str,
        nargs=1,
        help="A vertices file in the file format specified above. Normally, this is the original vertices file fed into the mingle program."
    )
    parser.add_argument(
        '--more_vertices',
        required=False,
        type=str,
        nargs=1,
        help="An additional vertices file in the file format specified above. Normally, this is the output vertices file from the mingle program that contains the mingled vertices."
    )
    parser.add_argument(
        '--edges',
        required=True,
        type=str,
        nargs=1,
        help="An edges file in the file format specified above.")
    parser.add_argument(
        '--vertex_size',
        required=False,
        type=int,
        nargs=1,
        default=3,
        help="The size to display the vertices in.")
    args = parser.parse_args()
    if (args.more_vertices):
        plotFromVerticesAndEdgesFile(args.vertices[0], args.edges[0],
                                     args.vertex_size, args.more_vertices[0])
    else:
        plotFromVerticesAndEdgesFile(args.vertices[0], args.edges[0],
                                     args.vertex_size)
