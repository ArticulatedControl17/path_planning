#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
from os.path import dirname, abspath


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


# Takes a matrix, the scale of that matrix (eg. 10 for scale 1:10)
# and (x, y)-coordinates for an element (w.r.t. to the scale of the matrix)
#
# If given coordinates are valid for the given matrix:
#     Returns the value of element at given index,
#     with respect to the scale of the given matrix
# If given coordinates are out of bounds:
#     Returns None
def getValue(matrix, scale, x, y):
    (ix, iy) = (int(x/scale), int(y/scale))
    try:
        return matrix[iy][ix]
    except IndexError:
        print "Index out of bounds"
        return None
