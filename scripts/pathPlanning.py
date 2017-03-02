
from math import *
import sys
import decimal
import model
import error_calc

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y

class graphFinder:

    def __init__(self):
        self.dt = 25
        self.speed = 1
        self.length_header = 27
        self.length_trailer = 62
        self.theta1 = radians(90)
        self.theta2 = radians(90)

        self.model = model.truck()
        self.ec = error_calc.errorCalc("optimal_path.txt")
        self.prev_right_wheel = Point(-1,-1)


    def checkIfInTrack(self, toPoint, th1, th2):


        p0 = Point(100,100) #corner position

        points = self.model.calculateCorners(toPoint, th1, th2)
        for (x,y) in points:
            if x <=0 or y <=30 or y >= 600 or x >=600 :
                #truck to the left of track
                return False
            if x>100 and y>100:
                return False

        right_wheel = Point(points[5][0], points[5][1])

        isLeft = ((right_wheel.x - self.prev_right_wheel.x)*(p0.y - self.prev_right_wheel.y) - (right_wheel.y - self.prev_right_wheel.y)*(p0.x - self.prev_right_wheel.x)) >=0

        #check if p0 is to the left of the line, that means that the line enter forbidden area
        #isLeft = ((points[0][0] - points[2][0])*(p0.y - points[2][1]) - (points[0][1] - points[2][1])*(p0.x - points[2][0])) <=0
        dist = sqrt( (100 - right_wheel.x)**2 + (100 - right_wheel.y)**2 )
        if isLeft and dist <20:
            return False
        return True

    def creategraph(self, startPoint, endPoint):

        dd = self.speed * self.dt
        steering_angle_rad = radians(0)

        #initiate fields
        self.pos = startPoint
        (toPoint,strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)
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
            self.pos = Point(x, y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            if dist <10:
                #reached end, gather the path
                path = self.gatherPath(startPoint)
                return path

            #goin straight
            dd = self.speed * self.dt
            steering_angle_rad = radians(0)

            (to_point_strait, strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)

            #going right
            steering_angle_rad = radians(16) #max right angle

            (to_point_Right1,right_theta1,right_theta2) = self.calculateNextState(dd, steering_angle_rad)

            #steering_angle_rad = radians(8) #middle right angle

            #(vectorRightVal2,right_theta_narrow1,right_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)
            #vectorRightVal2 = (round(vectorRight2[1].x, 0), round(vectorRight2[1].y, 0))

            #going left
            steering_angle_rad = radians(-21) #max left angle

            (to_point_Left1,left_theta1, left_theta2) = self.calculateNextState(dd, steering_angle_rad)

            #steering_angle_rad = radians(-10) #middle left angle

            #(vectorLeftVal2,left_theta_narrow1,left_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)
            #vectorLeftVal2 = (round(vectorLeft2[1].x, 1), round(vectorLeft2[1].y, 1))

            #see if the endpoint is to the right/left/strait and go correct direction from there

            goRightOrLeft = self.ec.calculateError(Point(self.pos.x,self.pos.y))
            #TODO: Never goes strait, change to following a line, and add stearing angle according to distance from line

            if goRightOrLeft<=10 and goRightOrLeft>=-10:
                #Go strait
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(to_point_Left1,left_theta1, left_theta2):
                    self.addState(to_point_Left1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)
                if self.checkIfInTrack(to_point_Right1, right_theta1, right_theta2):
                    self.addState(to_point_Right1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)
                if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                    self.addState(to_point_strait, strait_theta1, strait_theta2)

            elif goRightOrLeft<10:
                #Go right
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(to_point_Left1, left_theta1, left_theta2):
                    self.addState(to_point_Left1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)
                if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                    self.addState(to_point_strait, strait_theta1, strait_theta2)
                if self.checkIfInTrack(to_point_Right1, right_theta1, right_theta2):
                    self.addState(to_point_Right1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)

            else:
                #Go left
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(to_point_Right1, right_theta1, right_theta2):
                    self.addState(to_point_Right1, right_theta1, right_theta2)
                #if self.checkIfInTrack(vectorRight2, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(vectorRightVal2, self.pos, right_theta_narrow1, right_theta_narrow2)
                if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                    self.addState(to_point_strait, strait_theta1, strait_theta2)
                if self.checkIfInTrack(to_point_Left1, left_theta1, left_theta2):
                    self.addState(to_point_Left1, left_theta1, left_theta2)
                #if self.checkIfInTrack(vectorLeft2, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(vectorLeftVal2, self.pos, left_theta_narrow1, left_theta_narrow2)

            #mark the previous vector as visited
            self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

        print "reached End, no solution found"
        return self.visited

    def gatherPath(self, startPoint):
        path = []
        prex = self.pos.x
        prey = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(((prex,prey), pret1, pret2))
            ((nx,ny),nt1, nt2) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
        path.append(((startPoint.x,startPoint.y), radians(180), radians(180)))
        print self.pos.x, self.pos.y #point found
        print path
        return path

    def calculateNextState(self, dd, steering_angle_rad):
        next_theta1 = self.theta1 + (dd * tan(steering_angle_rad)) / self.length_header
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2))/ self.length_trailer
        next_x = self.pos.x + dd * cos(next_theta1)
        next_y = self.pos.y - dd * sin(next_theta1)  # Subtracting instead of adding, since the y-axis is flipped

        return (Point(next_x,next_y), next_theta1, next_theta2)

    def addState(self, point, th1, th2):
        #add the vector as an adjacent vector to the previous vector in the graph
        self.fromPoints[(point.x, point.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2)
        self.toVisit.append(((point.x,point.y), th1, th2))

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
