import sys
import argparse

def doTranscription(linksFilePath, coordinatesFilePath, outputVerticesPath, outputEdgesPath):
	nodeIdToNodeCoord = {}
	with open(coordinatesFilePath) as coordinatesFile:
		next(coordinatesFile)
		for line in coordinatesFile:
			nodeid, xcoord, ycoord = line.split()
			nodeIdToNodeCoord[nodeid] = (xcoord, ycoord)
	edges = []
	with open(linksFilePath) as linksFile:
		next(linksFile)
		for line in linksFile:
			parts = line.split()
			nodeid = parts[0]
			for link in parts[1:]:
				try:
					edges.append((nodeIdToNodeCoord[nodeid], nodeIdToNodeCoord[link]))
				except KeyError as e:
					continue
	outputVerticesFile = open(outputVerticesPath, 'w')
	outputEdgesFile = open(outputEdgesPath, 'w')
	outputEdgesFile.write(str(len(edges)) + "\n")
	outputVerticesFile.write(str(len(nodeIdToNodeCoord)) + "\n")
	for nodeId, coordinates in nodeIdToNodeCoord.iteritems():
		outputVerticesFile.write(nodeId + " " + coordinates[0] + " " + coordinates[1] + "\n")
	for edge in edges:
		node1 = edge[0]
		node2 = edge[1]
		outputEdgesFile.write(node1[0] + " " + node1[1] + " " + node2[0] + " " + node2[1] + "\n")
	outputVerticesFile.close()
	outputEdgesFile.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='''Converts the coordinates.tsv and links.tsv file into a file format that mingle can handle.'''
    )
    parser.add_argument(
        '--links',
        required=True,
        type=str,
        nargs=1,
        help="The links.tsv file that contains the pair of nodes that are connected between each other."
    )
    parser.add_argument(
        '--coordinates',
        required=True,
        type=str,
        nargs=1,
        help="The coordinates.tsv file that contains all of the nodes id as well as their x and y coordinates."
    )
    parser.add_argument(
        '--output_vertices',
        required=True,
        type=str,
        nargs=1,
        help="The output vertices file that mingle needs to bundle the nodes."
    )
    parser.add_argument(
        '--output_edges',
        required=True,
        type=str,
        nargs=1,
        help="The output edges file that mingle needs to bundle the nodes."
    )
    args = parser.parse_args()
    doTranscription(args.links[0], args.coordinates[0], args.output_vertices[0], args.output_edges[0])