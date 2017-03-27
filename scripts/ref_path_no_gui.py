#!/usr/bin/env python
from graph_func import *
from map_func import *

import warnings
import _tkinter
from math import sin, cos, radians
import matplotlib.pyplot as plt


GRAPH_PATH = "/graph.txt"


class VehicleState:

    def __init__(self, x, y, theta_1, theta_2=0):
        self.x = x
        self.y = y
        self.theta_1 = theta_1
        self.theta_2 = theta_2


class RefPath:

    def __init__(self):
        self.graph = readFileToGraph(GRAPH_PATH)
        self.path = []


    # Takes a VehicleState object, and an array of tuples of (x, y)-coordinates
    # Calculates the shortest path from vehicle position to the first coordinate point,
    # and from each coordinate point to the next
    #
    # Returns a reference path in the form of an array of tuples of (x, y)-coordinates
    def getRefPath(self, vehicle_state, pts):

        # Finding the Node (in valid direction) which is closest to the vehicle,
        # to use as a start point
        # If the vehicle is too far away from a valid start point,
        # returns []
        start_point = getClosestToVehicle(self.graph, vehicle_state)
        if not start_point:
            print "== ERROR: The vehicle is too far away from a valid path"
            return []

        self.path = []
        partial_path = []

        # Adding 'start_point' first in 'pts'
        pts.insert(0, (start_point.x, start_point.y))

        # If at least one coordinate point was given
        if len(pts) > 1:

            # Calculating shortest path between the points
            for point in pts[1:]:
                path = shortestPath(self.graph, start_point, Point(point[0], point[1]))
                if path != None:
                    self.path += path
                    start_point = Point(self.path[-1][0], self.path[-1][1])

                # If the given coordinate points were not in range of any Nodes
                else:
                    self.path = []
                    print "== ERROR: Reference path out of range for %s" % str(point)
                    break

        # Printing status msg
        if self.path == []:
            print "[] returned"
        else:
            print "Path returned"
            #print "Path:", self.path

        return self.path


# Main, used for testing
if __name__ == "__main__":

    SCALE = 10  # Map img is in scale 1:10

    COORDS_VALID = [
        (2550.5208333333326, 3392.5781250000009),
        (3178.7760416666661, 7162.109375),
        (2600.78125, 8569.4010416666679)
    ]

    COORDS_INVALID = [
        (2701.3020833333326, 3392.5781250000009),
        (3203.90625, 5151.6927083333339),
        (4133.7239583333321, 6860.546875)  # This point is out of range
    ]

    refpath_obj = RefPath()
    vehicle_state = VehicleState(4000, 1630, -90)


    # INVALID PATH
    """
    path = refpath_obj.getRefPath(vehicle_state, COORDS_INVALID)
    """

    # VALID PATH
    
    path = refpath_obj.getRefPath(vehicle_state, COORDS_VALID)

    # Plotting graph
    plt.axis("scaled")
    plt.xlim( (0, 490) )
    plt.ylim( (965, 0) )
    plotGraph(refpath_obj.graph, "b", 10)

    # Plotting path
    xs = map(lambda x: x[0] / SCALE, path)
    ys = map(lambda x: x[1] / SCALE, path)
    plt.plot(xs, ys, "-r", linewidth=3.0)

    plt.show()
    
