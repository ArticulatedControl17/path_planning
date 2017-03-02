import math


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
        self.width = 30
        self.length =  50
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

class rectangle:


#if __name__ == '__main__':
#    rect = rectangle()
#    print rect.calculateCorners(Point(75,200),Point(75,190))










#
