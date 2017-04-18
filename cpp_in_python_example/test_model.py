#!/usr/bin/env python
from math import *
import matplotlib.pyplot as plt
from helper_functions import HEADER_LENGTH, HEADER_WIDTH, TRAILER_LENGTH, TRAILER_WIDTH, HL_FRONT, TL_BACK



class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y

class Vector:
    def __init__(self, x , y):
        self.values = (x,y)

    def magnitude(self):
        #magnitude of the vector
        (x,y)= self.values
        return math.sqrt(x*x + y*y)

    def rotate(self, theta):
        #rotate the vector theta degrees
        theta = math.radians(theta)
        dc, ds = math.cos(theta), math.sin(theta)
        (x, y) = self.values
        (x, y) = dc*x - ds*y, ds*x + dc*y
        return Vector(x, y)

point = Point(0,0)
point1 = Point(0,0)



class truck:

    def calculateCorners(self, pointFront, th1, th2):

        point = self.back_middle_trailer(pointFront, th1, th2)

        # hitch joint
        x2 = point.x + (TRAILER_LENGTH + TL_BACK) *cos(th2);
        y2 = point.y + (TRAILER_LENGTH + TL_BACK) *sin(th2);

        #print x2, y2


        #left back
        x4 = point.x - cos(pi/2-th2)*TRAILER_WIDTH/2;
        y4 = point.y + sin(pi/2-th2)*TRAILER_WIDTH/2;
        print "left back:", x4, y4

        #right back
        x5 = point.x + cos(pi/2-th2)*TRAILER_WIDTH/2;
        y5 = point.y - sin(pi/2-th2)*TRAILER_WIDTH/2;
        print "right back:", x5, y5


        #left back axis
        x12 = x4 + TL_BACK * cos(th2)
        y12 = y4 + TL_BACK * sin(th2)
        print "left back axis:", x12, y12

        #right back axis
        x13 = x5 + TL_BACK * cos(th2)
        y13 = y5 + TL_BACK * sin(th2)
        print "right back axis:", x13, y13



        #left joint wheel
        x6 = x2 - cos(pi/2-th1)*HEADER_WIDTH/2;
        y6 = y2 + sin(pi/2-th1)*HEADER_WIDTH/2;

        #right joint wheel
        x7 = x2 + cos(pi/2-th1)*HEADER_WIDTH/2;
        y7 = y2 - sin(pi/2-th1)*HEADER_WIDTH/2;

        #left front axis
        x8 = x6 + (HEADER_LENGTH-5.0)*cos(th1);
        y8 = y6 + (HEADER_LENGTH-5.0)*sin(th1);
        print "left front axis:", x8, y8

        #right front axis
        x9 = x7 + (HEADER_LENGTH-5.0)*cos(th1);
        y9 = y7 + (HEADER_LENGTH-5.0)*sin(th1);
        print "right front axis:", x9, y9


        #left front

        x10 = x6 + ((HEADER_LENGTH-5.0) + HL_FRONT)*cos(th1);
        y10 = y6 + ((HEADER_LENGTH-5.0) + HL_FRONT)*sin(th1);
        print "left front:", x10, y10

        #right front

        x11 = x7 + ((HEADER_LENGTH-5.0) + HL_FRONT)*cos(th1);
        y11 = y7 + ((HEADER_LENGTH-5.0) + HL_FRONT)*sin(th1);
        print "right front:", x11, y11



        return((x8,y8), (x9,y9), (x6,y6), (x7,y7), (x12, y12), (x13, y13), (x10, y10), (x11, y11), (x4, y4), (x5, y5))

    def rightbackWheel(self):

        point = self.back_middle_trailer()

        x5 = point.x + cos(pi/2-th2)*TRAILER_WIDTH/2;
        y5 = point.y - sin(pi/2-th2)*TRAILER_WIDTH/2;

    def back_middle_trailer(self, pointFront, th1, th2):
        jpx = pointFront.x - cos(th1) * (HEADER_LENGTH-5.0)
        jpy = pointFront.y - sin(th1) * (HEADER_LENGTH-5.0)

        px = jpx - cos(th2) * (TRAILER_LENGTH + TL_BACK)
        py = jpy - sin(th2) * (TRAILER_LENGTH + TL_BACK)
        return Point(px,py)




if __name__ == '__main__':

    truck = truck()
    truck.calculateCorners(Point(200, 200), radians(-90), radians(-90))








