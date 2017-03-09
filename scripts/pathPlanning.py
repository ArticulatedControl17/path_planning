
from math import *
import sys
import decimal
import model
import error_calc
import os
import numpy as np
import cv2
import time

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y

class graphFinder:

    def __init__(self):
        self.speed = 1
        self.length_header = 27
        self.length_trailer = 62
        self.findOptimalPath = True
        self.solutions = 5
        self.steps = 1
        self.max_time = 0.05

        self.all_paths = []
        self.model = model.truck()
        self.map = np.asarray(cv2.imread('rondell_3.png', 0), dtype=np.bool).tolist()


    def checkIfInTrack(self, toPoint, th1, th2):
        #check if point and key wheels are in the track

        #check the range of the matrix with the allowed positions, to avoid index error
        if toPoint.x <0 or toPoint.y <0 or toPoint.x >1000 or toPoint.y >1000:
            return False

        points = self.model.calculateCorners(toPoint, th1, th2)

        if not self.map[int(toPoint.y)][int(toPoint.x)]:
            return False


        right_back_wheel = Point(points[5][0], points[5][1])
        left_back_wheel = Point(points[4][0], points[4][1])
        right_front_wheel = Point(points[1][0], points[1][1])
        left_front_wheel = Point(points[0][0], points[0][1])

        #check right front wheels
        if right_front_wheel.x <0 or right_front_wheel.y <0 or right_front_wheel.x >1000 or right_front_wheel.y >1000:
            return False
        if not self.map[int(right_front_wheel.y)][int(right_front_wheel.x)]:
            return False
        #check left front wheels
        if left_front_wheel.x <0 or left_front_wheel.y <0 or left_front_wheel.x >1000 or left_front_wheel.y >1000:
            return False
        if not self.map[int(left_front_wheel.y)][int(left_front_wheel.x)]:
            return False

        prev_points = self.model.calculateCorners(self.pos, self.theta1, self.theta2)
        prev_right_back_wheel = Point(prev_points[5][0], prev_points[5][1])
        prev_left_back_wheel = Point(prev_points[4][0], prev_points[4][1])
        #prev_right_front_wheel = Point(points[1][0], points[1][1])
        #prev_left_front_wheel = Point(points[0][0], points[0][1])

        #TODO: Could be really slow
        between = self.getPointsInBetween((right_back_wheel.x, right_back_wheel.y), (prev_right_back_wheel.x, prev_right_back_wheel.y), self.dt)


        #check right back wheel
        #TODO: Check left back wheel
        for (x,y) in between:
            if x <0 or y <0 or x >1000 or y >1000:
                return False
            if not self.map[y][x]:
                return False
        return True

    def creategraph(self, startPoint, endPoint, dt):

        start_time = time.time() #start a timer, to stop the really slow searches, some dt's are reallly slow, other are quick

        self.theta1 = radians(0) #start angle for header
        self.theta2 = radians(0) #start angle for trailer
        self.dt = dt #the delta time used for kinematic model, basicly the path step size
        self.paths = [] #list of solutions for current delta t
        self.ec = error_calc.errorCalc("optimal_path_rondell3.txt") #make new error calc every time to reset it and look from the beginning

        #initiate fields
        dd = self.speed * self.dt
        steering_angle_rad = radians(0)

        self.pos = startPoint
        (toPoint,strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)
        self.visited = set([]) #the points we have visited
        self.fromPoints ={(toPoint.x, toPoint.y) : ((startPoint.x , startPoint.y), self.theta1, self.theta2)} #a Map which hold the frompoints given a toPoint
        self.toVisit = [((toPoint.x,toPoint.y),self.theta1, self.theta2)] #a stack of points to visit,
        count =0 #for debugging

        while len(self.toVisit)>0:
            #loop until all possible nodes have been visited
            while True:
                if len(self.toVisit)==0:
                    print "reached End, no solution found"
                    return self.creategraph(startPoint, endPoint, self.dt-1)
                ((x,y),t1, t2) = self.toVisit.pop()
                if ((x,y),t1, t2) not in self.visited:
                    break
            #found new node to visit
            self.pos = Point(x, y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            #check if we have reached the end
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            if self.ec.isAboveEnd(Point(523, 377),Point(endPoint.x,endPoint.y), self.pos) and dist <5*self.dt: #is above a line the two last points make
                #reached end, gather the path
                path = self.gatherPath(startPoint, endPoint)
                self.paths = self.paths + [path]
            else:
                #we have not yet found a solution, search for new possible nodes
                goRightOrLeft = self.ec.calculateError(Point(self.pos.x,self.pos.y)) #check if we are left or right of the optimal path
                dd = self.speed * self.dt

                steering_angle_rad = radians(0)
                findPath = False

                (to_point_strait, strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)
                next_err = self.ec.calculateError(to_point_strait)
                if (next_err <0 and goRightOrLeft>0) or (next_err >0 and goRightOrLeft<0):
                    #steering angle passing over optimal pass
                    findPath = True


                #going right
                steering_angle_rad = radians(16) #max right angle

                (to_point_right,right_theta1,right_theta2) = self.calculateNextState(dd, steering_angle_rad)
                next_err = self.ec.calculateError(to_point_right)
                if (next_err <0 and goRightOrLeft>0) or (next_err >0 and goRightOrLeft<0):
                    #steering angle passing over optimal pass
                    findPath = True

                steering_angle_rad = radians(8) #middle right angle

                (to_point_right_narrow,right_theta_narrow1,right_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)

                #going left
                steering_angle_rad = radians(-21) #max left angle

                (to_point_left,left_theta1, left_theta2) = self.calculateNextState(dd, steering_angle_rad)
                next_err = self.ec.calculateError(to_point_left)
                if (next_err <0 and goRightOrLeft>0) or (next_err >0 and goRightOrLeft<0):
                    #steering angle passing over optimal pass
                    findPath = True


                steering_angle_rad = radians(-10) #middle left angle

                (to_point_left_narrow,left_theta_narrow1,left_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)

                #see if the endpoint is to the right/left/strait and go correct direction from there

                #if goRightOrLeft<=5 and goRightOrLeft>=-5:
                #if findPath:
                    #Go strait
                    #check if the vectors are within the allowed track
                #    if self.checkIfInTrack(to_point_left,left_theta1, left_theta2):
                #        self.addState(to_point_left, left_theta1, left_theta2)
                    #if self.checkIfInTrack(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2):
                    #    self.addState(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2)
                #    if self.checkIfInTrack(to_point_right, right_theta1, right_theta2):
                #        self.addState(to_point_right, right_theta1, right_theta2)
                    #if self.checkIfInTrack(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2):
                    #    self.addState(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2)
                    #if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                    #    self.addState(to_point_strait, strait_theta1, strait_theta2)
                #    (tp,tt1,tt2) = self.calculate_steering(radians(-21), radians(16.0), dd, 10)
                #    if self.checkIfInTrack(tp, tt1, tt2):
                #        self.addState(tp, tt1, tt2)

                if goRightOrLeft<0:
                    #Go right
                    #check if the vectors are within the allowed track
                    #if self.checkIfInTrack(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2):
                    #    self.addState(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2)
                    if self.checkIfInTrack(to_point_left, left_theta1, left_theta2):
                        self.addState(to_point_left, left_theta1, left_theta2)
                    if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                        self.addState(to_point_strait, strait_theta1, strait_theta2)
                    #if self.checkIfInTrack(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2):
                    #    self.addState(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2)
                    if self.checkIfInTrack(to_point_right, right_theta1, right_theta2):
                        self.addState(to_point_right, right_theta1, right_theta2)
                    (tp,tt1,tt2) = self.calculate_steering(radians(-21), radians(16.0), dd, 10)
                    if self.checkIfInTrack(tp, tt1, tt2):
                        self.addState(tp, tt1, tt2)


                else:
                    #Go left
                    #check if the vectors are within the allowed track
                    #if self.checkIfInTrack(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2):
                    #    self.addState(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2)
                    if self.checkIfInTrack(to_point_right, right_theta1, right_theta2):
                        self.addState(to_point_right, right_theta1, right_theta2)
                    if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                        self.addState(to_point_strait, strait_theta1, strait_theta2)
                    #if self.checkIfInTrack(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2):
                    #    self.addState(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2)
                    if self.checkIfInTrack(to_point_left, left_theta1, left_theta2):
                        self.addState(to_point_left, left_theta1, left_theta2)
                    (tp,tt1,tt2) = self.calculate_steering(radians(-21), radians(16), dd, 10)
                    if self.checkIfInTrack(tp, tt1, tt2):
                        self.addState(tp, tt1, tt2)


                #mark the previous vector as visited
                self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

                if time.time() - start_time > self.max_time:
                    return self.creategraph(startPoint, endPoint, self.dt-1)


                if len(self.all_paths)>=self.solutions:
                    sorted_list = sorted(self.all_paths, cmp=lambda (a,l1),(b,l2): cmp(a,b))
                    for (x,path) in sorted_list:
                        print x
                    print sorted_list[0]
                    return sorted_list[0]

                elif len(self.paths)>= self.steps:
                    #steps are amount of solution for every dt
                    print "dt in steps: ", self.dt
                    shortest = self.get_avarege_error(self.paths)
                    self.all_paths = self.all_paths + shortest
                    return self.creategraph(startPoint, endPoint, self.dt-1)

        if len(self.all_paths)>self.solutions:
            sorted_list = sorted(self.all_paths, cmp=lambda (a,l1),(b,l2): cmp(a,b))
            for (x,path) in sorted_list:
                print x
            print sorted_list[0]
            return sorted_list[0]

        if len(self.paths)>0:
            #found solution
            print "dt: ", self.dt
            shortest = self.get_avarege_error(self.paths)
            self.all_paths = self.all_paths + shortest
            return self.creategraph(startPoint, endPoint, self.dt-1)
        else:
            print "dt: ", self.dt
            return self.creategraph(startPoint, endPoint, self.dt-1)

    def calculate_steering(self, steering_min, steering_max, dd, iters):
        #Calculates a point within 1 unit of the optimal path
        steering_new = (steering_min + steering_max)/2
        (new_point, t1, t2) = self.calculateNextState(dd, steering_new)
        error = self.ec.calculateError(new_point)
        #print "error:", error
        if abs(error)<1 or iters==0:
            return (new_point, t1,t2)
        elif error<0:
            #search right
            return self.calculate_steering(steering_new, steering_max, dd, iters-1)
        else:
            return self.calculate_steering(steering_min, steering_new, dd, iters-1)

    def gatherPath(self, startPoint, endPoint):
        path = []
        #TODO: add correct theta's (if needed)
        path.append(((endPoint.x,endPoint.y), self.theta1, self.theta2))
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2)
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

    def getShortest(self, paths):
        distPath= []
        for path in paths:
            path.pop() #remove last item (start point)
            totDist = 0
            for ((x,y),th1,th2) in path:
                dist = self.calculateDist((x,y))
                totDist = totDist+dist
            distPath.append((totDist,path))
        return sorted(distPath,  cmp=lambda (a,l1),(b,l2): cmp(a,b))


    def calculateDist(self, (x,y)):
        fromx = self.fromPoints[(x,y)][0][0]
        fromy = self.fromPoints[(x,y)][0][1]
        dist = sqrt(((x -fromx)*(x -fromx)) + ((y -fromy)*(y -fromy)))
        return dist

    def getShortest_area(self, paths):
        areaPath= []
        for path in paths:
            workPath = [((100,400),radians(180),radians(180)) , ((100,100),radians(90),radians(90)), ((400,75),radians(90),radians(90))] + path
            xSum = 0
            ySum = 0
            for i in range(len(workPath)-1):
                ((x,y),th1,th2) = workPath[i]
                ((nx,ny),nth1,nth2) = workPath[i+1]
                xSum = xSum + (x*ny)
                ySum = ySum + (y*nx)
            areaPath.append(((ySum-xSum)/2,path))
        return sorted(areaPath,  cmp=lambda (a,l1),(b,l2): cmp(a,b))

    def get_avarege_error(self, paths):
        error_path= []
        for path in paths:
            totDist = 0
            for ((x,y),th1,th2) in path:
                ec = error_calc.errorCalc("optimal_path_rondell3.txt") #TODO: slow to read from file all the time?
                error = ec.calculateError(Point(x,y))
                if error>0:
                    totDist = totDist+abs(error)
                else:
                    totDist = totDist+abs(error)
            error_path.append((totDist/len(path),path))
        return sorted(error_path,  cmp=lambda (a,l1),(b,l2): cmp(a,b))



    def getPointsInBetween(self, p1, p2, n):
        p1x, p1y = p1
        p2x, p2y = p2

        dx = p2x - p1x
        dy = p2y - p1y

        stepX = dx/float(n+1)
        stepY = dy/float(n+1)

        points = []
        for i in range(1, n+1):
            x = int(round(p1x + i * stepX))
            y = int(round(p1y + i * stepY))
            points.append((x, y))

        return points



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
