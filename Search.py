from collections import deque
import Utilities


'''
Node class to represent different nodes in the maze
'''
class Node:
    def __init__(self, x, y, cost, parent, heuristic = 1):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.cost == other.cost and self.heuristic == other.heuristic


'''
Implementation of breadth-first search. Takes a 2D maze, start position and goal position as input and
returns a Node representing the goal state once it is reached.
'''
def BFS(maze, start, goal):
    visited = []
    frontier = deque()
    expanded = 0
    visited.append(start)
    s = Node(start[0], start[1], 0, None)
    frontier.append(s)

    while frontier:
        print("Expanded: " + str(expanded) + "\n")
        current = frontier.popleft()
        expanded+=1
        if current.x == goal[0] and current.y == goal[1]:
            return current, expanded
        for adjacent in Utilities.getAdjacentNodes(maze, (current.x, current.y)):
            if adjacent not in visited:
                visited.append(adjacent)
                frontier.append(Node(adjacent[0], adjacent[1], 1 + current.cost, current))

'''
Implementation of depth-first search. Takes a 2D maze, start position and goal position as input and
returns a Node representing the goal state once it is reached.
'''
def DFS(maze, start, goal):
    visited = []
    frontier = []
    expanded = 0
    visited.append(start)
    s = Node(start[0], start[1], 0, None)
    frontier.append(s)

    while frontier:
        print("Expanded: " + str(expanded) + "\n")
        current = frontier.pop()
        expanded+=1
        if current.x == goal[0] and current.y == goal[1]:
            return current, expanded
        for adjacent in Utilities.getAdjacentNodes(maze, (current.x, current.y)):
            if adjacent not in visited:
                visited.append(adjacent)
                frontier.append(Node(adjacent[0], adjacent[1], 1 + current.cost, current))


'''
Implementation of greedy best first search. Takes a 2D maze, start position and goal position as input and
returns a Node representing the goal state once it is reached.
'''
def GBFS(maze, start, goal):
    visited = []
    frontier = []
    expanded = 0
    visited.append(start)
    s = Node(start[0], start[1], 0, None, Utilities.getManhattanDistance(start, goal))
    frontier.append(s)

    while frontier:
        print("Expanded: " + str(expanded) + "\n")
        current = Utilities.getLowestHeuristicNode(frontier)
        frontier.remove(current)
        expanded+=1
        if current.x == goal[0] and current.y == goal[1]:
            return current, expanded
        for adjacent in Utilities.getAdjacentNodes(maze, (current.x, current.y)):
            if adjacent not in visited:
                visited.append(adjacent)
                frontier.append(Node(adjacent[0], adjacent[1], 1 + current.cost, current, Utilities.getManhattanDistance(adjacent, goal)))

'''
Implementation of A* search. Takes a 2D maze, start position and goal position as input and
returns a Node representing the goal state once it is reached. Next node chosen is based on lowest
heuristic for A*
'''
def a_star1(maze, start, goal):
    visited = []
    frontier = []
    expanded = 0
    visited.append(start)
    s = Node(start[0], start[1], 0, None, Utilities.getManhattanDistance(start, goal))
    frontier.append(s)

    while frontier:
        print("Expanded: " + str(expanded) + "\n")
        current = Utilities.getLowestHeuristicNode_astar(frontier)
        frontier.remove(current)
        expanded+=1
        if current.x == goal[0] and current.y == goal[1]:
            return current, expanded
        for adjacent in Utilities.getAdjacentNodes(maze, (current.x, current.y)):
            if adjacent not in visited:
                visited.append(adjacent)
                frontier.append(Node(adjacent[0], adjacent[1], 1 + current.cost, current, Utilities.getManhattanDistance(adjacent, goal)))
                
'''
Uses the output of basic search functions to print a report and print the solution
to fileName
'''
def printBasicReport(maze, goal, expanded, fileName):
    pathCost = goal.cost
    current = goal

    while (current.parent is not None):
        print("Path: (" + str(current.x) + "," + str(current.y) + ")\n")
        maze[current.x][current.y] = '.'
        current = current.parent
    Utilities.writeMazeToFile(maze, fileName)

    print("Path cost: " + str(pathCost) + "\nExpanded: " + str(expanded))



'''
Helper function that executes the basic search functions. Takes mazeFileName as input maze and
writes to outputFileName as output maze
'''

def executeBasicSearch(searchFunc, mazeFileName, outputFileName):
    maze = Utilities.parseMaze(mazeFileName)
    start = Utilities.getStartPoint(maze)
    goals = Utilities.getGoalPoints(maze)
    goal, expanded = searchFunc(maze, start, goals[0])
    printBasicReport(maze, goal, expanded, outputFileName)





#executeBasicSearch(BFS, "bigMaze.txt", "bigMazeSol.txt")


executeBasicSearch(GBFS, "bigMaze.txt", "bigMazeSol.txt")
