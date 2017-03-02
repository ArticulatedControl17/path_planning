
from math import *
import sys
import decimal
import model

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y

class graphFinder:

    def __init__(self):
        self.model = model.rectangle()

    def checkIfInTrack(self, (fromPoint, toPoint), th1, th2):
        p0 = Point(100,100) #corner position

        points = self.model.calculateCorners(toPoint, th1, th2)
        for (x,y) in points:
            if x <=0 or y <=40 or y >= 500 or x >=500 :
                #truck to the left of track
                return False
            if x>100 and y>100:
                return False

        #check if p0 is to the left of the line, that means that the line enter forbidden area
        isLeft = ((points[0][0] - points[2][0])*(p0.y - points[2][1]) - (points[0][1] - points[2][1])*(p0.x - points[2][0])) <=0
        dist = sqrt( (100 - points[0][0])**2 + (100 - points[0][1])**2 )
        if isLeft and dist <10:
            return False
        if fromPoint.y<100 and toPoint.y>100:
            #going back down again
            return False
        return True

    def creategraph(self, startPoint, endPoint):
        self.dt = 17
        self.speed = 1
        self.length_header = 27
        self.length_trailer = 62
        self.theta1 = radians(90)
        self.theta2 = radians(90)
        self.pos = startPoint

        self.model = model.truck()


        dd = self.speed * self.dt
        steering_angle_rad = radians(0)


        (vectorStrait,strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)
        vectorStraitVal = (round(vectorStrait[1].x, 0), round(vectorStrait[1].y, 0))
        toPoint = Point(vectorStraitVal[0], vectorStraitVal[1])

        #initiate fields
        self.visited = set([]) #the points we have visited
        self.fromPoints ={(toPoint.x, toPoint.y) : ((startPoint.x , startPoint.y), self.theta1, self.theta2)} #a Map which hold the frompoints given a toPoint
        self.toVisit = [((toPoint.x,toPoint.y),self.theta1, self.theta2)] #a stack of points to visit,
        self.pos = startPoint
        count =0 #for debugging

        while len(self.toVisit)>0:
            #loop until all possible nodes have been visited
            count= count+1
            print count
            while True:
                if len(self.toVisit)==0:
                    print "reached End, no solution found"
                    return self.visited
                ((x,y),t1, t2) = self.toVisit.pop()
                if ((x,y),t1, t2) not in self.visited:
                    break
            ((a,b), pt1, pt2) = self.fromPoints[x, y]
            self.pos = Point(x, y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            frontx = x + self.length_header*cos(self.theta1)
            fronty = y - self.length_header*sin(self.theta1)
            dist = sqrt( (endPoint.x - frontx)**2 + (endPoint.y - fronty)**2 )
            if dist <15:
                #reached end, gather the path
                path = self.gatherPath(startPoint)
                return path

            #goin straight
            dd = self.speed * self.dt
            steering_angle_rad = radians(0)

            (vectorStrait,strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)
            vectorStraitVal = (round(vectorStrait[1].x, 0), round(vectorStrait[1].y, 0))

            #going right
            steering_angle_rad = radians(21) #max right angle

            (vectorRight1,right_theta1,right_theta2) = self.calculateNextState(dd, steering_angle_rad)
            vectorRightVal1 = (round(vectorRight1[1].x, 0), round(vectorRight1[1].y, 0))

            steering_angle_rad = radians(8) #middle right angle

            (vectorRight2,right_theta_narrow1,right_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)
            vectorRightVal2 = (round(vectorRight2[1].x, 0), round(vectorRight2[1].y, 0))

            #going left
            steering_angle_rad = radians(-16) #max left angle

            (vectorLeft1,left_theta1, left_theta2) = self.calculateNextState(dd, steering_angle_rad)
            vectorLeftVal1 = (round(vectorLeft1[1].x, 1), round(vectorLeft1[1].y, 1))

            steering_angle_rad = radians(-10) #middle left angle

            (vectorLeft2,left_theta_narrow1,left_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)
            vectorLeftVal2 = (round(vectorLeft2[1].x, 1), round(vectorLeft2[1].y, 1))

            #see if the endpoint is to the right/left/strait and go correct direction from there

            fx = self.pos.x + self.length_header*cos(self.theta1)
            fy = self.pos.y - self.length_header*sin(self.theta1)

            goRightOrLeft = ((fx - self.pos.x)*(endPoint.y - self.pos.y) -
                (fy - self.pos.y)*(endPoint.x - self.pos.x))

            #TODO: Never goes strait, change to following a line, and add stearing angle according to distance from line

            if goRightOrLeft==0:
                #Go strait
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorLeft1,left_theta1, left_theta2):
                    self.addState(vectorLeftVal1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)
                if self.checkIfInTrack(vectorRight1, right_theta1, right_theta2):
                    self.addState(vectorRightVal1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)
                if self.checkIfInTrack(vectorStrait, strait_theta1, strait_theta2):
                    self.addState(vectorStraitVal, strait_theta1, strait_theta2)

            elif goRightOrLeft<0:
                #Go right
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorLeft1, left_theta1, left_theta2):
                    self.addState(vectorLeftVal1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)
                if self.checkIfInTrack(vectorStrait, strait_theta1, strait_theta2):
                    self.addState(vectorStraitVal, strait_theta1, strait_theta2)
                if self.checkIfInTrack(vectorRight1, right_theta1, right_theta2):
                    self.addState(vectorRightVal1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)

            else:
                #Go left
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorRight1, right_theta1, right_theta2):
                    self.addState(vectorRightVal1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)
                if self.checkIfInTrack(vectorStrait, strait_theta1, strait_theta2):
                    self.addState(vectorStraitVal, strait_theta1, strait_theta2)
                if self.checkIfInTrack(vectorLeft1, left_theta1, left_theta2):
                    self.addState(vectorLeftVal1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)

            #mark the previous vector as visited
            self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

        print "reached End, no solution found"
        return self.visited

    def gatherPath(self, startPoint):
        path = []
        preX = self.pos.x
        preY = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        while preX!= startPoint.x and preY != startPoint.y:
            path.append(((preX,preY), pret1, pret2))
            ((nX,nY),nt1, nt2) = self.fromPoints[preX,preY]
            preX=nX
            preY=nY
            pret1 = nt1
            pret2 = nt2
        #path.append(((startPoint.x,startPoint.y), radians(180), radians(180)))
        print self.pos.x, self.pos.y #point found
        print path
        return path

    def calculateNextState(self, dd, steering_angle_rad):
        next_theta1 = self.theta1 + (dd * tan(steering_angle_rad)) / self.length_header
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2))/ self.length_trailer
        next_x = self.pos.x + dd * cos(next_theta1)
        next_y = self.pos.y - dd * sin(next_theta1)  # Subtracting instead of adding, since the y-axis is flipped

        return ((self.pos, Point(next_x,next_y)), next_theta1, next_theta2)

    def addState(self, vectorVal, th1, th2):
        #add the vector as an adjacent vector to the previous vector in the graph
        self.fromPoints[vectorVal] = ((self.pos.x, self.pos.y),self.theta1, self.theta2)
        self.toVisit.append((vectorVal, th1, th2))

#if __name__ == '__main__':

    #pf = graphFinder()
    #pf.creategraph(Point(75,200), Point(200,75))

#    **************************Tests****************************

#    pf = graphFinder()
#    self.pos=Point(0,5)
#    prevFromPoint=Point(0,10)
#    distance = 5
#    prevV = Vector(self.pos.x - prevFromPoint.x , self.pos.y - prevFromPoint.y)
#    mag = prevV.magnitude()
#    print "mag :", mag
#    nPrevV = tuple( comp/mag for comp in prevV.values )
#    (newx,newy) = tuple(comp*distance for comp in nPrevV) #the vector from self.pos to newPoint
#    toPoint = pf.vectorToPoint(Vector(newx,newy), self.pos) # new strait point
#    print "x: " , toPoint.x, "y: " , toPoint.y
#    newV = Vector(newx,newy).rotate(90)
#    toPoint = pf.vectorToPoint(newV, self.pos) # new strait point 45 degrees to the right
#    print "x: " , toPoint.x, "y: " , toPoint.y


#    graphf = graphFinder()
#    print "true :"
#    print graphf.checkIfInTrack(Point(75,200),Point(82,192))
#    print graphf.checkIfInTrack(Point(75,100),Point(25,50))
#    print graphf.checkIfInTrack(Point(25,50),Point(50,25))
#    print graphf.checkIfInTrack(Point(50,25),Point(75,50))
#    print graphf.checkIfInTrack(Point(75,50),Point(200,75))
#
#    print "false:"
#    print graphf.checkIfInTrack(Point(100,50),Point(150,150))
#    print graphf.checkIfInTrack(Point(50,50),Point(200,125))
