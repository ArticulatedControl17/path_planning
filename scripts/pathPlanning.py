
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
    #TODO: make variables in start for "magic numbers" and extract to methods for better structure

    def __init__(self):
        self.speed = 1
        self.length_header = 27
        self.length_trailer = 62
        self.solutions = 5
        self.steps = 1
        self.max_time = 0.05

        self.all_paths = []
        self.model = model.truck() #model used to calculate error
        self.map = np.asarray(cv2.imread('rondell_3.png', 0), dtype=np.bool).tolist() #picture used to create a map with valid positions


    def checkIfInTrack(self, toPoint, th1, th2):
        #TODO: Make new class for this
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

        #TODO: affects performance quite a lot with dt amount of points
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
            if self.ec.isAboveEnd(Point(523, 377),Point(endPoint.x,endPoint.y), self.pos) and dist <5*self.dt: #checks if we are above a line of the two last points
                #reached end, gather the path
                path = self.gatherPath(startPoint, endPoint)
                self.paths = self.paths + [path]
            else:
                #we have not yet found a solution, search for new possible nodes
                goRightOrLeft = self.ec.calculateError(Point(self.pos.x,self.pos.y)) #check if we are left or right of the optimal path
                dd = self.speed * self.dt

                steering_angle_rad = radians(0)

                (to_point_strait, strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)

                #going right
                steering_angle_rad = radians(16) #max right angle

                (to_point_right,right_theta1,right_theta2) = self.calculateNextState(dd, steering_angle_rad)

                #going left
                steering_angle_rad = radians(-21) #max left angle

                (to_point_left,left_theta1, left_theta2) = self.calculateNextState(dd, steering_angle_rad)

                #prioritize nodes that are towards the optimal path
                if goRightOrLeft<0:
                    #Go right
                    #check if the nodes are within the allowed track
                    if self.checkIfInTrack(to_point_left, left_theta1, left_theta2):
                        self.addState(to_point_left, left_theta1, left_theta2)
                    if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                        self.addState(to_point_strait, strait_theta1, strait_theta2)
                    #find the optimal path, return max steering angle if we cant reach the optimal path
                    (tp,tt1,tt2) = self.calculate_steering(radians(-21), radians(16), dd, 10)
                    if self.checkIfInTrack(tp, tt1, tt2):
                        self.addState(tp, tt1, tt2)


                else:
                    #Go left
                    #check if the nodes are within the allowed track
                    if self.checkIfInTrack(to_point_right, right_theta1, right_theta2):
                        self.addState(to_point_right, right_theta1, right_theta2)
                    if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                        self.addState(to_point_strait, strait_theta1, strait_theta2)
                    #find the optimal path, return max steering angle if we cant reach the optimal path
                    (tp,tt1,tt2) = self.calculate_steering(radians(-21), radians(16), dd, 10)
                    if self.checkIfInTrack(tp, tt1, tt2):
                        self.addState(tp, tt1, tt2)

                #mark the previous node/state as visited
                self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

                if time.time() - start_time > self.max_time:
                    #cancel the current search and go to next dt
                    return self.creategraph(startPoint, endPoint, self.dt-1)

                elif len(self.paths)>= self.steps:
                    #steps are amount of solution for every dt, when we have found correct solutions for the current dt, go to next
                    print "dt in steps: ", self.dt
                    shortest = self.get_avarege_error(self.paths)
                    self.all_paths = self.all_paths + shortest
                    if len(self.all_paths)>=self.solutions:
                        #found enugh solutions, sort the results and return the one with the smallest error
                        sorted_list = sorted(self.all_paths, cmp=lambda (a,l1),(b,l2): cmp(a,b))
                        for (x,path) in sorted_list:
                            print x
                        print sorted_list[0]
                        return sorted_list[0]
                    #look for more solutions with lower dt
                    return self.creategraph(startPoint, endPoint, self.dt-1)


        print "dt: ", self.dt
        return self.creategraph(startPoint, endPoint, self.dt-1)

    def calculate_steering(self, steering_min, steering_max, dd, iters):
        #Calculates a point within 1 unit of the optimal path, return the closest possibility if we cant find the optimal path
        steering_new = (steering_min + steering_max)/2
        (new_point, t1, t2) = self.calculateNextState(dd, steering_new)
        error = self.ec.calculateError(new_point)
        if abs(error)<1 or iters==0:
            return (new_point, t1,t2)
        elif error<0:
            #search right
            return self.calculate_steering(steering_new, steering_max, dd, iters-1)
        else:
            #search left
            return self.calculate_steering(steering_min, steering_new, dd, iters-1)

    def gatherPath(self, startPoint, endPoint):
        path = []
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

    def get_avarege_error(self, paths):
        #TODO: add error in main loop when we already calculate it. Now we do it twice
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
