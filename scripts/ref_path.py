#!/usr/bin/env python
from graph_func import *
from map_func import *

import matplotlib.pyplot as plt
from pylab import ginput
from PIL import Image
from os.path import dirname, abspath
import warnings
import _tkinter
from math import sin, cos, radians


IMG_PATH = "/map.png"
GRAPH_PATH = "/graph.txt"

SCALE = 10  # Map img is in scale 1:10

COLORS = ["r", "c", "m", "g", "y"]


class VehicleState:

    def __init__(self, x, y, theta_1, theta_2=0):
        self.x = x
        self.y = y
        self.theta_1 = theta_1
        self.theta_2 = theta_2


class RefPath:

    def __init__(self):
        dirpath = dirname(abspath(__file__))

        self.map_img = Image.open(dirpath + IMG_PATH)
        self.graph = readFileToGraph(GRAPH_PATH)
        self.path = []

        # To handle user input
        self.valid = False
        self.pts = None
        self.start_point = None
        self.exit_handler = None
        self.key_handler = None
        self.path_plot = None
        self.plot_nbr = 0
        self.ax = None
        self.fig = None


    # Handler for 'key_press_event'
    def onKeyPress(self, event):

        # Enter (accepting path)
        if event.key == "enter":
            self.path += self.partial_path
            self.fig.canvas.mpl_disconnect(self.exit_handler)
            plt.close()

        # Backspace (discarding path)
        elif event.key == "backspace":
            self.valid = False
            self.pts = None

            # Removing the plotted path
            for i, line in enumerate(self.path_plot):
                self.path_plot.pop(i)
                line.remove()
            plt.draw()
            print "Path discarded\n====="
            print "Select a new path"

            # Letting user input a new path
            self.createRefPath()

        # 'a' or 'A' (adding to path)
        elif event.key == "a" or event.key == "A":
            last = self.pts[len(self.pts)-1]
            self.start_point = Point(last[0], last[1])
            print "Select points to add to the path"
            
            self.valid = False
            self.pts = None

            self.plot_nbr += 1
            self.path += self.partial_path[:-1]
            #print "Current path:", self.path
            
            self.createRefPath()


    # Handler for 'close_event' (closing down the window without accepting a path)
    def onExit(self, event):
        self.path = []
        self.valid = True


    # Used to plot partial paths in different colors
    def getColor(self):
        i = self.plot_nbr % 5
        return COLORS[i]


    # Takes a VehicleState object,
    # and an array of tuples of (x, y)-coordinates (optional)
    # Lets user input destination and calculates the shortest path to that destination
    #
    # Returns a reference path in the form of an array of tuples of (x, y)-coordinates
    def getRefPath(self, vehicle_state, pts=None):
        theta = vehicle_state.theta_1
        vx, vy = vehicle_state.x / SCALE, vehicle_state.y / SCALE
        dx = 300 / SCALE * cos(radians(theta))
        dy = -300 / SCALE * sin(radians(theta))

        # Finding the Node (in valid direction) which is closest to the vehicle,
        # to use as a start point
        # If the vehicle is too far away from a valid start point,
        # returns []
        self.start_point = getClosestToVehicle(self.graph, vehicle_state)
        if not self.start_point:
            print "The vehicle is too far away from a valid path"
            return []

        print ("=====\n'Left click' on the image to select points\n" +
              "'Right click' to undo the last point\n" +
              "'Middle button' or 'Enter' to create the path")

        xlim = self.map_img.size[0]
        ylim = self.map_img.size[1]

        self.fig = plt.figure()
        self.ax = plt.axes()

        # Graph settings
        plt.axis("scaled")
        plt.xlim( (0, xlim) )
        plt.ylim( (ylim, 0) )
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("getRefPath()")

        # Displaying map image
        plt.imshow(self.map_img)
        # Plotting graph
        plotGraph(self.graph, "b", 10)
        # Plotting vehicle position and direction
        plt.plot(vx, vy, "ob", markersize=10)
        self.ax.arrow(vx, vy, dx, dy, linewidth=2, head_width=10, head_length=12, fc="b", ec="b")
        # Plotting start point
        plt.plot(self.start_point.x / SCALE, self.start_point.y / SCALE, "or", markersize = 5)

        self.key_handler = None
        self.valid = False

        self.path = []
        self.pts = pts

        # Calling help function, to create the reference path
        self.createRefPath()

        # Returns [] when closed before user has accepted a path
        if self.path == []:
            print "=====\n[] returned"
        else:
            print "=====\nPath returned"
            print "Path:", self.path

        return self.path


    # Help function to 'getRefPath()'
    # Lets user input destination and calculates the shortest path to that destination
    def createRefPath(self):
        self.exit_handler = self.fig.canvas.mpl_connect("close_event", self.onExit)

        # Repeating until a valid path is created
        while not self.valid:
            self.fig.canvas.mpl_disconnect(self.key_handler)
            self.partial_path = []

            # If 'pts' was Not given as an argument
            if not self.pts:
                # Gathering input
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    try:
                        self.pts = ginput(n=0, timeout=0, mouse_add=1, mouse_pop=3, mouse_stop=2)
                    except _tkinter.TclError:
                        return []
                # Adjusting for SCALE
                self.pts = map(lambda pt: (pt[0] * SCALE, pt[1] * SCALE), self.pts)

            # Adding 'start_point' first in 'pts'
            self.pts.insert(0, (self.start_point.x, self.start_point.y))

            # If at least one point was inputted
            if len(self.pts) > 1:
                self.valid = True
                start = self.pts[0]

            # Calculating shortest path between the points
            for point in self.pts[1:]:
                path = shortestPath(self.graph, Point(start[0], start[1]), Point(point[0], point[1]))
                if path != None:
                    self.partial_path += path
                    start = point

                # If the given points were not in range of any Nodes,
                # setting 'valid' to False to let user retry
                else:
                    self.valid = False
                    self.pts = None
                    print "=====\nA path could not be created, please input new points"
                    break

            # If a valid path was created,
            # letting user decide whether to keep or discard the path
            if self.valid:

                # Plotting the path
                xs = map(lambda x: x[0] / SCALE, self.partial_path)
                ys = map(lambda x: x[1] / SCALE, self.partial_path)
                self.path_plot = self.ax.plot(xs, ys, "-"+self.getColor(), linewidth=3.0)
                self.key_handler = self.fig.canvas.mpl_connect("key_press_event", self.onKeyPress)
                print "=====\nPress 'Enter' to keep the path, 'a' to add to the path, 'Backspace' to discard"
                plt.show()


# Main, used for testing
if __name__ == "__main__":
    refpath_obj = RefPath()
    vehicle_state = VehicleState(4000, 1630, -90)
    
    path = refpath_obj.getRefPath(vehicle_state)
