from Point import *
from math import *

LENGTH_HEADER = 23
LENGTH_TRAILER = 48.5
LANE_WIDTH = 20
PADDING_WEIGHT = 2

w1 = 0#0.275
w2 = 0#0.70#0.15#0.09
w4 = 0
w5 = 0.

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
    next_theta1 = theta1 + (dd * tan(steering_angle_rad)) / LENGTH_HEADER
    next_theta2 = theta2 + (dd * sin(theta1 - theta2))/ LENGTH_TRAILER
    #next_theta2 = self.theta2 + (dd * sin(next_theta1 - self.theta2))/ self.length_trailer


    a1 = theta2 - theta1
    a2 = next_theta2 - next_theta1


    da = a2 - a1
    #print da
    if a2 > 0:
        if da > 0:
            next_theta2 += da * w4
        else:
            next_theta2 += abs(da) * w5
    else:
        if da < 0:
            next_theta2 -= abs(da) * w4
        else:
            next_theta2 -= da * w5


    dt1 = next_theta1 - theta1
    dt2 = next_theta2 - theta2
    alpha = next_theta2 - next_theta1


    if alpha > 0:
        next_theta2 += dt2 * w2
        next_theta2 += (-dt1) * w1
    else:
        next_theta2 -= (-dt2) * w2
        next_theta2 -= dt1 * w1



    next_x = pos.x + dd * cos(next_theta1)
    next_y = pos.y + dd * sin(next_theta1)

    #next_x = self.pos.x + dd * cos(self.theta1)
    #next_y = self.pos.y + dd * sin(self.theta1)

    return (Point(next_x,next_y), next_theta1, next_theta2)
