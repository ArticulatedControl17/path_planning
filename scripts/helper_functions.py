from Point import *
from math import *


HEADER_LENGTH = 27.0
TRAILER_LENGTH = 49.0
HL_FRONT = 10.0
TL_BACK = 13.5
HEADER_WIDTH = 18.0;
TRAILER_WIDTH = 18.0

LANE_WIDTH = 20
PADDING_WEIGHT = 10
OUTSIDE_TURN_ERROR = 8

def calculate_steering(steering_min, steering_max, dd, iters, target_error, pos, theta1, theta2, ec):
    #Calculates a point within 1 unit of the optimal path, return the closest possibility if we cant find the optimal path
    steering_new = (steering_min + steering_max)/2
    (new_point, t1, t2) = calculateNextState(theta1, theta2, pos, dd, steering_new)
    error = ec.calculateError(new_point)
    if abs(error-target_error)<0.1 or iters==0:
        return (new_point, t1, t2)
    elif error<target_error:
        #search right
        return calculate_steering(steering_new, steering_max, dd, iters-1, target_error, pos, theta1, theta2, ec)
    else:
        #search left
        return calculate_steering(steering_min, steering_new, dd, iters-1, target_error, pos, theta1, theta2, ec)


def calculateNextState(theta1, theta2, pos, dd, steering_angle_rad):

    next_theta1 = theta1 + (dd * tan(steering_angle_rad)) / HEADER_LENGTH
    next_theta2 = theta2 + (dd * sin(theta1 - theta2))/ TRAILER_LENGTH

    dx = pos.x - HEADER_LENGTH/2 * cos(theta1)
    dy = pos.y - HEADER_LENGTH/2 * sin(theta1)

    next_x = dx + dd * cos(next_theta1) + HEADER_LENGTH/2 * cos(next_theta1)
    next_y = dy + dd * sin(next_theta1) + HEADER_LENGTH/2 * sin(next_theta1)


    return (Point(next_x,next_y), next_theta1, next_theta2)
