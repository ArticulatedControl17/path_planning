import numpy as np
import cv2
import matplotlib.pyplot as plt

matrix = np.asarray(cv2.imread('rondell_3.png', 0), dtype=np.bool).tolist()

print matrix[800][81] #False or True

plt.imshow(matrix)
(start, end) = plt.ginput(2)

print start, end
