#!/usr/bin/env python
from os.path import dirname, abspath
from math import sqrt, radians
from enum import Enum
import matplotlib.pyplot as plt
import gc


# For representing a Point in a coordinate system
class Point:

    # Takes (x, y)-coordinates for a Point
    def __init__(self, x, y):
        self.x = x
        self.y = y


    # String representation of a Point object
    def __str__(self):
        to_str = "%s, %s" % (self.x, self.y)
        return to_str


# For representing a Directed Graph
# A Graph object consists of:
#     a list of Node objects
class Graph:

    # Takes an array of Node objects, which make up the Graph (optional)
    def __init__(self, nodes=None):
        self.nodes = nodes if nodes else []


    # String representation of a Graph object
    def __str__(self):
        to_str = ""
        for node in self.nodes[:-1]:
            to_str += "%s\n" % (node)
        if len(self.nodes) > 0:
            to_str += "%s" % (self.nodes[-1])
        return to_str


    # Takes another Graph object
    # Adds all Nodes from that Graph to this Graph (without creating duplicates)
    def addGraph(self, graph):
        for node in graph.nodes:
            self.addNode(node)


    # Takes a Node object
    # Adds that Node to this Graph (without creating duplicates)
    def addNode(self, node):
        current_node = self.getNode(node.x, node.y)
        if current_node:
            for e in node.out_edges:
                self.addNode(e)
                current_node.addOutEdge(e)
        else:
            self.nodes.append(node)


    # Takes (x, y)-coordinates for a Node
    #
    # If there is a Node object with given coordinates in this Graph:
    #     Returns that Node object
    # Otherwise:
    #     Reurns None 
    def getNode(self, x, y):
        for node in self.nodes:
            if node == Node(x, y):
                # Node object found
                return node
        # Node object Not found
        return None


    # Takes (x, y)-coordinates for a Node
    #
    # If there is a Node object with given coordinates in this Graph:
    #     Removes that Node object from the Graph (but NOT from out_edges of other Nodes)
    #     Returns True
    # Otherwise:
    #     Reurns False
    def removeNode(self, x, y):
        for i in range(len(self.nodes)):
            if self.nodes[i] == Node(x, y):
                # Node object found
                self.nodes.pop(i)
                return True
        # Node object Not found
        return False


    # For making an independent copy of this Graph (i.e. without referencing the same objects)
    def copyGraph(self):
        copy = Graph()

        # Copy each Node in this Graph, and add to the new Graph
        for node in self.nodes:
            copy.addNode(Node(node.x, node.y))

        # Add all outedges
        for node in self.nodes:
            new_node = copy.getNode(node.x, node.y)

            for out_edge in node.out_edges:
                new_out_edge = copy.getNode(out_edge.x, out_edge.y)
                new_node.addOutEdge(new_out_edge)

        return copy


# Takes an array of Node objects, and (x, y)-coordinates for a Node
#
# If there is a Node object with given coordinates in the Node-array:
#     Returns that Node object
# Otherwise:
#     Reurns None 
def getNodeFromList(nodes, x, y):
    for node in nodes:
        if node == Node(x, y):
            # Node object found
            return node
    # Node object Not found
    return None


# For representing a Node
# A Node object consists of:
#     (x, y)-coordinates for this Node
#     a list of references to other Node objects, to which there is an out-edge from this Node
class Node:

    # Takes (x, y)-coordinates for this Node
    # and an array of Node objects, to which there is an out-edge from this Node (optional)
    def __init__(self, x, y, out_edges=None):
        self.x = x
        self.y = y
        self.out_edges = out_edges if out_edges else []

        # For use in shortestpath()
        self.distance = float("inf")
        self.visited = False
        self.in_edges = []


    # String representation of a Node object
    def __str__(self):
        to_str = "(%s, %s) [" % (self.x, self.y)
        for node in self.out_edges[:-1]:
            to_str += "(%s, %s) " % (node.x, node.y)
        if len(self.out_edges) > 0:
            to_str += "(%s, %s)" % (self.out_edges[-1].x, self.out_edges[-1].y)
        to_str += "]"
        return to_str


    # Definition of equality between two Node objects
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


    # Adds an outgoing edge from this Node to the given Node object (without creating duplicates)
    def addOutEdge(self, node):
        out_edge = self.getOutEdge(node.x, node.y)
        if not out_edge:
            self.out_edges.append(node)


    # Takes (x,y)-coordinates for a Node
    #
    # If there is a Node object with given coordinates in the out-edges from this Node:
    #     Returns that Node object
    # Otherwise:
    #     Reurns None 
    def getOutEdge(self, x, y):
        for node in self.out_edges:
            if node.x == x and node.y == y:
                # Node object found
                return node
        # Node object Not found
        return None


    # Returns the length of an edge between this Node and the given Node object
    def getEdgeLength(self, to_node):
        return sqrt((to_node.x - self.x)**2 + (to_node.y - self.y)**2)


# Takes a path (relative to current directory) to a text file containing a Graph representation
# The textfile should be in the format specified in 'example_graph.txt'
#
# If parsing was completed without syntax errors:
#     Returns a Graph object with all nodes in its nodelist
# Otherwise:
#     Returns None
def readFileToGraph(path):
    graph = Graph()
    current_node = None
    dirpath = dirname(abspath(__file__))
    line_counter = 0

    # Variables for validation of syntax
    begin = False
    valid = False

    # Opening the file, parsing the lines one by one
    with open(dirpath + path, "r") as file:
        for line in file:
            # Line counter, for debugging of input file
            line_counter += 1
            # Removing whitespaces and tabs
            line = line.replace(" ", "")
            line = line.replace("\t", "")

            # Ignoring comments and empty lines
            if not line.startswith("#") and not line.startswith("\n"):

                # Start of node declaration
                if line == "NODE\n":
                    if not begin:
                        begin = True
                    else:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # End of node declaration
                elif line == "ENDNODE\n":
                    if begin and current_node:
                        begin = False
                        valid = False
                        current_node = None
                    else:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # Coordinates for current Node and its out-edges (connected Nodes)
                elif line[0].isdigit():

                    # Validating syntax
                    if begin:
                        # Removing newline characters and splitting line on list separator ';'
                        array = line.replace("\n", "").split(";")

                        # Out-edges
                        if current_node:
                            for elem in array:
                                elem = elem.split(",")
                                valid = True if len(elem) == 2 and elem[0].isdigit() and elem[1].isdigit() else False
                                # Break on Syntax error
                                if not valid:
                                    break

                                (x, y) = (int(elem[0]), int(elem[1]))
                                # If the connected Node is not already in the node-list, create a new Node object for it
                                outedge = graph.getNode(x, y)
                                if not outedge:
                                    outedge = Node(x, y)
                                    graph.addNode(outedge)
                                current_node.addOutEdge(outedge)

                        # Current Node
                        elif len(array) == 1:
                            elem = array[0].split(",")
                            valid = True if len(elem) == 2 and elem[1].isdigit() else False
                            # Break on Syntax error
                            if not valid:
                                break

                            (x, y) = (int(elem[0]), int(elem[1]))
                            # If the current Node is not already in the node-list, create a new Node object for it
                            current_node = graph.getNode(x, y)
                            if not current_node:
                                current_node = Node(x, y)
                                graph.addNode(current_node)

                        # Syntax error
                        else:
                            valid = False

                    # Syntax error
                    if not valid:
                        print "Syntax error on line %s in '%s'" % (line_counter, path)
                        return None

                # Syntax error
                else:
                    print "Syntax error on line %s in '%s'" % (line_counter, path)
                    return None

        return graph


# Takes a Graph and a filename
# Stores the Graph data in a file with given filename,
# in a format that can be re-read by using 'readFileToGraph(path_to_savefile)'
def saveGraphToFile(graph, filename):
    with open(filename, "w") as file:

        for node in graph.nodes:
            file.write("NODE\n")
            file.write("    %s,%s\n" % (node.x, node.y))

            for out_edge in node.out_edges[:1]:
                file.write("    %s,%s" % (out_edge.x, out_edge.y))
            for out_edge in node.out_edges[1:]:
                file.write(" ; %s,%s" % (out_edge.x, out_edge.y))

            if node.out_edges:
                file.write("\n")
            file.write("ENDNODE\n\n")


# Takes an array of Point objects
#
# Returns a Directed Graph, with an edge from each Point to the next one in the array
def pointsToGraph(points):
    graph = Graph()

    # Creating Node objects for all points
    for point in points:
        graph.addNode(Node(point.x, point.y))
    
    # Creating an edge from each Node to the next one in the array
    for i in range(len(points)-1):
        current_node = graph.getNode(points[i].x, points[i].y)
        next_node = graph.getNode(points[i+1].x, points[i+1].y)
        current_node.addOutEdge(next_node)

    return graph


# Takes a Graph and two Point objects with (x, y)-coordinates for start and end point
#
# If there is a path from the start Point to the end Point:
#     Returns the shortest path between them as an array of tuples of (x, y)-coordinates
# Otherwise:
#     Returns None
def shortestPath(graph, start, end):

    # Used to specify search range for finding closest point
    search_range = 200

    start_node = None
    end_node = None

    # Finding start and end Node:
    # (the Nodes which are closest to start resp. end point)

    # Start:
    # Selecting all Nodes which are in range from the start point
    nodes = getAllInRangeX(getAllInRangeY(graph.nodes, start, search_range), start, search_range)
    # Returning None if No Node is in range
    if not nodes:
        return None
    # Finding the two Nodes which are closest to the start point, x-wise resp. y-wise
    closest_x = getClosestX(nodes, start)
    closest_y = getClosestY(nodes, start)
    # Selecting the Node which is closest to the start point
    dx = Node(start.x, start.y).getEdgeLength(closest_x)
    dy = Node(start.x, start.y).getEdgeLength(closest_y)
    start_node = closest_x if dx <= dy else closest_y

    # End:
    # Selecting all Nodes which are in range from the end point
    nodes = getAllInRangeX(getAllInRangeY(graph.nodes, end, search_range), end, search_range)
    # Returning None if No Node is in range
    if not nodes:
        return None
    # Finding the two Nodes which are closest to the end point, x-wise resp. y-wise
    closest_x = getClosestX(nodes, end)
    closest_y = getClosestY(nodes, end)
    # Selecting the Node which is closest to the end point
    dx = Node(end.x, end.y).getEdgeLength(closest_x)
    dy = Node(end.x, end.y).getEdgeLength(closest_y)
    end_node = closest_x if dx <= dy else closest_y

    return findShortestPath(graph, start_node, end_node)


# Help function for 'shortestPath' and 'shortestPath2'
# Takes a Graph and two Nodes for start and end point
#
# If the start Node is the same as the end Node:
#     Returns []
# If there is a path from the start Node to the end Node:
#     Returns the shortest path between them as an array of tuples of (x, y)-coordinates
# Otherwise:
#     Returns None
def findShortestPath(graph, start_node, end_node):
    if start_node == end_node:
        return []

    unvisited_set = graph.copyGraph()

    start = unvisited_set.getNode(start_node.x, start_node.y)
    end = unvisited_set.getNode(end_node.x, end_node.y)

    path = []

    # Changing the value of 'distance' to 0 for the start Node
    # ('distance' is set to infinity for every Node, by default, on object creation)
    start.distance = 0
    current_node = start

    # Repeating until the end Node has been visited
    # (or until we know that there is no path from the start Node to the end Node)
    while not end.visited:

        # Checking if there are any more Nodes that can be visited,
        # returning None if there aren't any (since that means that the end Node can't be reached)
        if current_node.distance == float("inf"):
            return None

        # Going through all unvisited out-edges from 'current_node'
        # Calculating the distance to each such Node from the start point, via 'current_node',
        # and updating their 'distance' value if this distance is smaller than the current value
        for out_edge in current_node.out_edges:
            if not out_edge.visited:
                edge_length = current_node.getEdgeLength(out_edge)
                new_distance = current_node.distance + edge_length
                out_edge.in_edges.append(current_node)

                if new_distance < out_edge.distance:
                    out_edge.distance = new_distance

        # Removing 'current_node' from the unvisited set
        # (it is now considered visited, and will never be checked again)
        # and setting 'visited' to True
        # ('visited' is set to False for every Node, by default, on object creation)
        current_node.visited = True
        unvisited_set.removeNode(current_node.x, current_node.y)

        # Selecting the unvisited Node that has the smallest 'distance' as the new 'current_node'
        smallest_node = None
        smallest_distance = float("inf")
        for node in unvisited_set.nodes:
            if node.distance < smallest_distance:
                smallest_node = node
                smallest_distance = node.distance

        current_node = smallest_node

    # Backtracing to construct the path
    current_node = end
    path.insert(0, (current_node.x, current_node.y))

    # Repeating until start Node is reached
    while current_node != start:

        # Going through all in-edges, to find the Node which preceeds 'current_node'
        for node in current_node.in_edges:
            edge_length = node.getEdgeLength(current_node)
            if (node.distance + edge_length) == current_node.distance:
                current_node = node
                path.insert(0, (current_node.x, current_node.y))
                break

    del unvisited_set
    gc.collect()
    return path


# Takes an array of Node objects, Point object and range limits to the left resp. to the right
#
# Returns an array with all Nodes that are in range (set by given limits) x-wise from given Point
def getAllInRangeX(nodes, point, range_left, range_right=None):
    range_right = range_right if range_right != None else range_left
    node_list = []
    for node in nodes:
        # Selecting all Nodes that are in range x-wise
        if node.x >= point.x-range_left and node.x <= point.x+range_right:
            node_list.append(node)
    return node_list


# Takes an array of Node objects, Point object and range limits above resp. below
#
# Returns an array with all Nodes that are in range (set by given 'limit') y-wise from given Point
def getAllInRangeY(nodes, point, range_above, range_below=None):
    range_below = range_below if range_below != None else range_above
    node_list = []
    for node in nodes:
        # Selecting all Nodes that are in range y-wise
        if node.y >= point.y-range_above and node.y <= point.y+range_below:
            node_list.append(node)
    return node_list


# Takes an array of Node objects and a Point object
#
# Returns the Node that is (x-wise) closest to given Point
def getClosestX(nodes, point):
    closest_node = None
    closest_dist = float("inf")

    for node in nodes:
        dist = abs(point.x - node.x)
        # If no Node can be closer
        if dist == 0:
            return node
        elif dist < closest_dist:
            closest_dist = dist
            closest_node = node

    return closest_node


# Takes an array of Node objects and a Point object
#
# Returns the Node that is (y-wise) closest to given Point
def getClosestY(nodes, point):
    closest_node = None
    closest_dist = float("inf")
    
    for node in nodes:
        dist = abs(point.y - node.y)
        # If no Node can be closer
        if dist == 0:
            return node
        elif dist < closest_dist:
            closest_dist = dist
            closest_node = node

    return closest_node


Direction = Enum("Direction", "up down left right")


# Takes an array of Node objects, a Point object and a Direction
#
# Returns an array with all Nodes that have an out-edge in the given Direction
def getAllInRightDir(nodes, point, direction):
    node_list = []
    for node in nodes:
        # Selecting all Nodes that have an out-edge in the right direction
        if hasOutEdgeInRightDir(node, direction):
            node_list.append(node)

    return node_list


# Takes a Node and a Direction
#
# If given Node has an out-edge in given Direction:
#     Returns True
# Otherwise:
#     Returns False
def hasOutEdgeInRightDir(node, direction):

    # Used to specify minimum difference between given Node and an out-edge-node,
    # (x-wise resp. y-wise) for the edge to be considered as going in a specific Direction
    min_offset = 50

    # Going through all out-edges,
    # returning True as soon as we find one that goes in the given Direction
    for out_edge in node.out_edges:

        if direction == Direction.up:
            # If given Node has an out-edge upwards
            if (node.y - out_edge.y) > min_offset:
                return True

        elif direction == Direction.down:
            # If given Node has an out-edge downwards
            if (node.y - out_edge.y) < -min_offset:
                return True

        elif direction == Direction.left:
            # If given Node has an out-edge to the left
            if (node.x - out_edge.x) > min_offset: 
                return True

        elif direction == Direction.right:
            # If given Node has an out-edge to the right
            if (node.x - out_edge.x) < -min_offset:
                return True 

    # No out-edge in the given Direction
    return False


# Takes a Graph and a VehicleState object
#
# If there is a Node with an out-edge in the right Direction (with respect to theta):
#     Returns the closest Node which has an out-edge in the right Direction
# Otherwise:
#     Returns None
def getClosestToVehicle(graph, vehicle_state):

    # Used to specify search range for finding closest point
    search_range = 1000

    pos = Point(vehicle_state.x, vehicle_state.y)
    start_node = None

    # Normalizing theta
    theta = vehicle_state.theta_1 % 360

    # Vehicle angle: bottom-to-top
    if theta > 45 and theta <= 135:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(graph.nodes, pos, search_range, 0), pos, search_range)
        # Selecting all Nodes which have an out-edge upwards from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.up)

    # Vehicle angle: right-to-left
    elif theta > 135 and theta <= 225:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(graph.nodes, pos, search_range), pos, search_range, 0)
        # Selecting all Nodes which have an out-edge to the left from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.left)

    # Vehicle angle: top-to-bottom
    elif theta > 225 and theta <= 315:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(graph.nodes, pos, 0, search_range), pos, search_range)
        # Selecting all Nodes which have an out-edge downwards from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.down)

    # Vehicle angle: left-to-right
    elif theta > 315 or theta <=45:
        # Selecting all Nodes which are in range from the vehicle
        nodes = getAllInRangeX(getAllInRangeY(graph.nodes, pos, search_range), pos, 0, search_range)
        # Selecting all Nodes which have an out-edge to the right from vehicle position
        nodes = getAllInRightDir(nodes, pos, Direction.right)

    # Returning None if No Node is in range
    if not nodes:
        return None
        
    # Finding the two Nodes which are closest to the vehicle, x-wise resp. y-wise
    closest_x = getClosestX(nodes, pos)
    closest_y = getClosestY(nodes, pos)
    # Selecting the Node which is closest to the vehicle:
    dx = Node(pos.x, pos.y).getEdgeLength(closest_x)
    dy = Node(pos.x, pos.y).getEdgeLength(closest_y)
    start_node = closest_x if dx <= dy else closest_y

    return start_node


# For plotting a Graph
# Parameter 'color' should be in format "color",
# eg. "b" for blue, "k" for black, etc
def plotGraph(graph, color, scale=1):
    ax = plt.axes()

    # Plotting all graph edges
    for node in graph.nodes:
        for out_edge in node.out_edges:
            dx = out_edge.x/scale - node.x/scale
            dy = out_edge.y/scale - node.y/scale
            #ax.arrow(node.x/scale, node.y/scale, dx, dy, head_width=80/scale, head_length=100/scale, fc=color, ec=color)
            ax.arrow(node.x/scale, node.y/scale, dx, dy, fc=color, ec=color)
