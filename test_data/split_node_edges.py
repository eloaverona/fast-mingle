import sys

class Point:
	def __init__(self, x, y):
		self.x = x
		self.y = y

	def __eq__(self, other):
		return (self.x == other.x) and (self.y == other.y)

	def __ne__(self, other):
		return not self.__eq__(other)

	def __hash__(self):
		return hash(tuple([self.x, self.y]))

def split_nodes_edges(edgesFileName, nodesOutputFileName, edgesOutputFileName):
	pointToPointIdMap = {}
	edgesPairList = []
	with open(edgesFileName, 'r') as edgesFile:
		next(edgesFile)
		nextPointId = 0
		for line in edgesFile:
			parts = line.split()
			pointA = Point(parts[0], parts[1])
			pointB = Point(parts[2], parts[3])
			if pointA not in pointToPointIdMap:
				nextPointId = nextPointId + 1
				pointToPointIdMap[pointA] = "pointId" + str(nextPointId)
			if pointB not in pointToPointIdMap:
				nextPointId = nextPointId + 1
				pointToPointIdMap[pointB] = "pointId" + str(nextPointId)
			edgesPairList.append(pointToPointIdMap[pointA] + " " + pointToPointIdMap[pointB] + "\n")
	
	with open(edgesOutputFileName, 'w') as edgesOutputFile:
		edgesOutputFile.write(str(len(edgesPairList)) + "\n")
		edgesOutputFile.writelines(edgesPairList)

	with open(nodesOutputFileName, 'w') as nodesOutputFile:
		nodesOutputFile.write(str(len(pointToPointIdMap)) + "\n")
		for point, pointId in pointToPointIdMap.items():
			nodesOutputFile.write(pointId + " " + point.x + " " + point.y + "\n")

if __name__ == "__main__":
	# First argument is the file. Second argument is the output nodes. Third argument is the output edges.
	split_nodes_edges(sys.argv[1], sys.argv[2], sys.argv[3])