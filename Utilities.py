import numpy as np

def parseMaze(fileName):
    path = './' + fileName
    with open(path) as file:
        maze = [[c for c in line.strip()] for line in file]
    return maze

def writeMazeToFile(maze, fileName):
    np.savetxt(fileName, np.array(maze), fmt = '%s', delimiter = '')


def getStartPoint(maze):
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == "P":
                return i, j
    return None

def getGoalPoints(maze):
    goalPoints = []
    for i in range(len(maze)):
        for j in range(len(maze[0])):
            if maze[i][j] == ".":
                goalPoints.append((i, j))
    return goalPoints


def getAdjacentNodes(maze, node):
    adjacentNodes = []
    x = node[0]
    y = node[1]
    allAdjNodes = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
    for node in allAdjNodes:
        if(isWalkableNode(maze, node)):
            adjacentNodes.append(node)
    return adjacentNodes


def isWalkableNode(maze, node):
    cols = len(maze)
    rows = len(maze[0])
    x = node[0]
    y = node[1]
    if x >= 0 and y >=0 and x < cols and y < rows:
        if maze[x][y] != "%":
            return True
        return False
    return False