from math import *


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


class rectangle:


    def vectorToPoint(self, vector , prevToPoint ):
        (x,y) = vector.values
        return (prevToPoint.x + x, prevToPoint.y + y)

    def calculateCorners(self, fromPoint, toPoint):
        self.width = 18
        self.length =  27
        self.theta = 90- math.degrees(math.atan((self.length)/(self.width)))

        distance = math.sqrt((self.length/2)*(self.length/2) + (self.width/2)*(self.width/2))

        prevV = Vector(toPoint.x - fromPoint.x , toPoint.y - fromPoint.y)
        mag = prevV.magnitude()
        nPrevV = tuple( comp/mag for comp in prevV.values )
        (x,y) = tuple(comp*distance for comp in nPrevV) #the vector from toPoint to a straight new point

        rightTopCorner = self.vectorToPoint(Vector(x,y).rotate(self.theta), toPoint)
        leftTopCorner = self.vectorToPoint(Vector(x,y).rotate(-(self.theta)), toPoint)
        rightBottomCorner = self.vectorToPoint(Vector(x,y).rotate(180-self.theta), toPoint)
        leftBottomCorner = self.vectorToPoint(Vector(x,y).rotate(-(180-self.theta)), toPoint)

        return (rightTopCorner, leftTopCorner, rightBottomCorner, leftBottomCorner)
        #return rightTopCorner

class truck:
    def calculateCorners(self, pointFront, th1, th2):
        self.header_length = 27;
        self.trailer_length = 62;

        self.header_width = 18;
        self.trailer_width = 18;

        px = pointFront.x + cos(th2) * self.trailer_length
        py = pointFront.y - sin(th2) * self.trailer_length
        point = Point(px,py)

        #point = pointFront

        # hitch joint
        x2 = point.x + self.header_length*cos(th2);
        y2 = point.y - self.header_length*sin(th2);

        #middle of front of header
        x3 = x2 + self.header_length*cos(th1);
        y3 = y2 - self.header_length*sin(th1);

        #left back wheel
        x4 = point.x - cos(pi/2-th2)*self.trailer_width/2;
        y4 = point.y - sin(pi/2-th2)*self.trailer_width/2;

        #right back wheel
        x5 = point.x + cos(pi/2-th2)*self.trailer_width/2;
        y5 = point.y + sin(pi/2-th2)*self.trailer_width/2;

        #left joint wheel
        x6 = x2 - cos(pi/2-th1)*self.header_width/2;
        y6 = y2 - sin(pi/2-th1)*self.header_width/2;

        #right joint wheel
        x7 = x2 + cos(pi/2-th1)*self.header_width/2;
        y7 = y2 + sin(pi/2-th1)*self.header_width/2;

        #left front wheel
        x8 = x6 + self.header_length*cos(th1);
        y8 = y6 - self.header_length*sin(th1);

        #right front wheel
        x9 = x7 + self.header_length*cos(th1);
        y9 = y7 - self.header_length*sin(th1);

        return((x8,y8), (x9,y9), (x6,y6), (x7,y7), (x4,y4), (x5,y5))



#if __name__ == '__main__':
#    rect = rectangle()
#    print rect.calculateCorners(Point(75,200),Point(75,190))










#
