import sys

def generateFile(coordinatesFilePath, linksFilePath, outputFilePath):
	nodesMap = {}
	with open(coordinatesFilePath) as coordinatesFile:
		next(coordinatesFile)
		for line in coordinatesFile:
			nodeid, xcoord, ycoord = line.split()
			nodesMap[nodeid] = (xcoord, ycoord)
	nodes = []
	with open(linksFilePath) as linksFile:
		next(linksFile)
		for line in linksFile:
			parts = line.split()
			nodeid = parts[0]
			for link in parts[1:]:
				try:
					nodes.append((nodesMap[nodeid], nodesMap[link]))
				except KeyError as e:
					continue
	outputFile = open(outputFilePath, 'w')
	outputFile.write(str(len(nodes)) + "\n")
	for nodePair in nodes:
		outputFile.write(nodePair[0][0] + " " + nodePair[0][1] + " " + nodePair[1][0] + " " + nodePair[1][0] + "\n")
	outputFile.close()

if __name__ == "__main__":
    if (len(sys.argv) < 4):
        print("Usage python format_transcriber.py coordinates.tsv links.tsv output.txt")
    else:
        generateFile(sys.argv[1], sys.argv[2], sys.argv[3])