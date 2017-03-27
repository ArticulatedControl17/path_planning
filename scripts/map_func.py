#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.text as text
from os.path import dirname, abspath
from math import ceil


IMG_PATH = "/map.png"

SCALE = 10  # Map img is in scale 1:10


# For representing an obstacle on the track
class Obstacle:

    def __init__(self, x, y, width, height, padding):
        self.x = x  # Lower left corner
        self.y = y  # Lower left corner
        self.width = width
        self.height = height
        self.padding = padding

        self.text_x = (self.x + self.width + 100) / SCALE
        self.text_y = (self.y - self.height) / SCALE

        self.activated_patch = patches.Rectangle(
                (self.x / SCALE, self.y / SCALE),  # Lower left corner
                self.width / SCALE,
                -self.height / SCALE,
                fc="r", ec="0.5",
                linewidth=padding/10
            )

        self.deactivated_patch = patches.Rectangle(
                (self.x / SCALE, self.y / SCALE),  # Lower left corner
                self.width / SCALE,
                -self.height / SCALE,
                fc="b", ec="0.5",
                linewidth=padding/10
            )

        self.matrix_backup = []

        # For handling user input
        self.active = False
        self.plot = None
        self.text = None


OBSTACLES = [
        Obstacle(1780, 3750, 200, 110, 20),
        Obstacle(2020, 4360, 140, 120, 20),
        Obstacle(1470, 6860, 400, 80, 10),
        Obstacle(2980, 7710, 1000, 350, 30)
    ]


class Map:

    def __init__(self):
        self.matrix = readImgToMatrix(IMG_PATH)
        self.scale = SCALE
        self.obstacles = OBSTACLES


    def getMapAndScale(self):
        return (self.matrix, self.scale)


    # Takes an Index value
    # Adds the corresponding Obstacle from 'obstacles' to the 'matrix' of this Map
    #
    # If an Obstacle was added:
    #     Returns True
    # If given index is out of bounds, or the corresponding Obstacle is already activated:
    #     Returns False
    def addObstacle(self, index):

        # Checking the validity of the given index
        try:
            obstacle = self.obstacles[index]
        except IndexError:
            print "Obstacle index out of bounds"
            return False

        # If given obstacle is already active, there is no need to add it again
        if obstacle.active:
            return False

        height = ceil(float(obstacle.height)/float(self.scale))
        width = ceil(float(obstacle.width)/float(self.scale))
        padding = ceil(float(obstacle.padding)/float(self.scale))
        (x, y) = (int(obstacle.x/self.scale), int(obstacle.y/self.scale))

        # Going through all rows in the matrix
        for i in range(int(height)):
            row = []

            # Going through all elements on each row
            for j in range(int(width)):

                # For storing the old element value in 'matrix_backup' of given obstacle
                row.insert(j, self.matrix[y-i][x+j])

                # If there is padding, adding a grey frame with padding width above and below the obstacle
                if i+1 <= padding or padding > (height-1 - i):
                    self.matrix[y-i][x+j] = 2
                # If there is padding, adding a grey frame with padding width to the left and right of the obstacle
                elif j+1 <= padding or padding > (width-1 - j):
                    self.matrix[y-i][x+j] = 2
                # Adding non-padding area
                else:
                    self.matrix[y-i][x+j] = 0

            # Storing the old element values in 'matrix_backup' of given obstacle
            obstacle.matrix_backup.insert(i, row)
            
        obstacle.active = True
        return True


    # Takes an Index value
    # Removes the corresponding Obstacle from 'obstacles' from the 'matrix' of this Map
    #
    # If an Obstacle was removed:
    #     Returns True
    # If given index is out of bounds, or the corresponding Obstacle is already deactivated:
    #     Returns False
    def removeObstacle(self, index):
        try:
            obstacle = self.obstacles[index]
        except IndexError:
            print "Obstacle index out of bounds"
            return False

        # If given obstacle is Not active, there is no need to reset the matrix
        if not obstacle.active:
            return False

        height = ceil(float(obstacle.height)/float(self.scale))
        width = ceil(float(obstacle.width)/float(self.scale))
        padding = ceil(float(obstacle.padding)/float(self.scale))
        (x, y) = (int(obstacle.x/self.scale), int(obstacle.y/self.scale))


        # Using 'matrix_backup' of given obstacle to remove the obstacle
        # and reset the affected section of the matrix:

        # Going through all rows in the matrix
        for i in range(int(height)):
            row = []

            # Going through all elements on each row
            for j in range(int(width)):
                self.matrix[y-i][x+j] = obstacle.matrix_backup[i][j]

        # Clearing 'backup' of given obstacle
        obstacle.matrix_backup = []
        obstacle.active = False
        return True


    # Takes (x, y)-coordinates for an element
    #
    # If given coordinates are valid for the given matrix:
    #     Returns the value of element at given index,
    #     with respect to the scale of the matrix
    # If given coordinates are out of bounds:
    #     Returns None 
    def getValue(self, x, y):
        (ix, iy) = (int(x/self.scale), int(y/self.scale))
        try:
            return self.matrix[iy][ix]
        except IndexError:
            print "Map element index out of bounds"
            return None


# Takes a path (relative to current directory) to a an image file containing a Map representation
# The file should be in '.png'-format
#
# Returns an array of rows, where each row is an array of elements
def readImgToMatrix(path):
    dirpath = dirname(abspath(__file__))
    matrix = np.asarray(cv2.imread(dirpath + path, 0), dtype=np.uint8).tolist()

    # Going through all elements, adjusting their value to match [0, 1, 2]
    # Where:
    #     0 = Black
    #     1 = White
    #     2 = Grey
    for row in range(len(matrix)):
        for elem in range(len(matrix[row])):
            # Not black
            if matrix[row][elem] != 0:
                # White
                if matrix[row][elem] == 255:
                    matrix[row][elem] = 1
                # Grey
                else:
                    matrix[row][elem] = 2

    #plt.imshow(matrix)
    #plt.show()
    return matrix
