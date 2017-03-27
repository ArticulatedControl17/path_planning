
from math import *
import sys
import error_calc
import os
import time
import track_checker
import recalculatePath
from Point import Point
from vehicleState import vehicleState

class graphFinder:
    #TODO: add second to last point for error_calc
    #TODO: The absolute last point is not towrad the optimal path, could be maximum turn. FIX
    #TODO: fix "other-lane-padding"

    def __init__(self, mapp):
        self.speed = 1
        self.length_header = 27
        self.length_trailer = 62
        self.solutions = 1

        self.dt = 20 #the delta time used for kinematic model, basicly the path step size
        self.go_back_steps = 5
        self.padding_weight = 5

        self.trackChecker = track_checker.trackChecker(mapp)

    def getPath(self, vs, endPoint):

        self.recalculate_path = recalculatePath.recalculatePath(self.speed, self.length_header, self.length_trailer, self.trackChecker, self.padding_weight)
        self.on_optimal_path = True
        self.left_track_at = (-1,-1)
        self.could_left_at = []
        self.offset_treshold = 5

        self.theta1 = vs.theta1 #start angle for header
        self.theta2 = vs.theta2 #start angle for trailer
        self.ec = error_calc.errorCalc(self.optimal_path) #make new error calc every time to reset it and look from the beginning

        #initiate fields
        dd = self.speed * self.dt
        steering_angle_rad = radians(0)

        self.pos = Point(vs.x, vs.y)
        self.path = [] #list of solutions for current delta t
        self.fromPoints = {}
        self.toVisit = []
        #TODO: change so that first step dont allways go strait
        (toPoint,strait_theta1, strait_theta2, err) = self.calculateNextState(dd, steering_angle_rad)
        self.visited = set([]) #the points we have visited
        self.addState(toPoint, strait_theta1, strait_theta2, err)

        while len(self.toVisit)>0:
            #loop until all possible nodes have been visited
            while True:
                ((x,y),t1, t2, err) = self.toVisit.pop()
                if ((x,y),t1, t2, err) not in self.visited:
                    break
            #found new node to visit
            self.pos = Point(x, y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            #check if we have reached the end
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            #TODO: Add so that we can get the second to last point from error calc
            if self.ec.isAboveEnd(Point(330, 497),Point(endPoint.x,endPoint.y), self.pos) and dist <5*self.dt: #checks if we are above a line of the two last points
                #reached end, gather the path
                print "reached end, Gathering solution"
                self.path = self.gatherPath(Point(vs.x, vs.y), endPoint,self.theta1, self.theta2)
                return self.path
            else:
                #we have not yet found a solution, search for new possible nodes
                currentError = self.ec.calculateError(Point(self.pos.x,self.pos.y)) #check if we are left or right of the optimal path
                dd = self.speed * self.dt

                steering_angle_rad = radians(0)

                (to_point_strait, strait_theta1, strait_theta2, strait_error) = self.calculateNextState(dd, steering_angle_rad)

                #going right
                steering_angle_rad = radians(16) #max right angle

                (to_point_right,right_theta1,right_theta2, right_error) = self.calculateNextState(dd, steering_angle_rad)

                #going left
                steering_angle_rad = radians(-16) #max left angle

                (to_point_left,left_theta1, left_theta2, left_error) = self.calculateNextState(dd, steering_angle_rad)

                #finding optimal path
                (to_point_optimal, optimal_theta1, optimal_theta2, optimal_error) = self.calculate_steering(radians(-16), radians(16), dd, 10)
                added_optimal = False

                if abs(currentError) > self.offset_treshold and self.on_optimal_path:
                    self.on_optimal_path = False
                    self.left_track_at = self.pos

                #prioritize nodes that are towards the optimal path
                if currentError<0:
                    #Go right
                    #check if the nodes are within the allowed track
                    #Left
                    (inTrack, inPadding) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt, left_error)
                    if inPadding:
                        left_error = left_error * self.padding_weight
                    if inTrack:
                        self.addState(to_point_left, left_theta1, left_theta2, left_error)
                        if self.on_optimal_path and left_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))
                    #Strait
                    (inTrack, inPadding) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt, strait_error)
                    if inPadding:
                        strait_error = strait_error* self.padding_weight
                    if inTrack:
                        self.addState(to_point_strait, strait_theta1, strait_theta2, strait_error)
                        if self.on_optimal_path and strait_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))
                    #Optimal
                    (inTrack, inPadding) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt, optimal_error)
                    if inPadding:
                        optimal_error = optimal_error * self.padding_weight
                    if inTrack:
                        added_optimal = True
                        self.addState(to_point_optimal, optimal_theta1, optimal_theta2, optimal_error)
                        if self.on_optimal_path and optimal_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))

                else:
                    #Go left
                    #check if the nodes are within the allowed track
                    #Right
                    (inTrack, inPadding) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt, right_error)
                    if inPadding:
                        right_error = right_error * self.padding_weight
                    if inTrack:
                        self.addState(to_point_right, right_theta1, right_theta2, right_error)
                        if self.on_optimal_path and right_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))
                    #Strait
                    (inTrack, inPadding) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt, strait_error)
                    if inPadding:
                        strait_error = strait_error * self.padding_weight
                    if inTrack:
                        self.addState(to_point_strait, strait_theta1, strait_theta2, strait_error)
                        if self.on_optimal_path and strait_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))
                    #Optimal
                    (inTrack, inPadding) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt, optimal_error)
                    if inPadding:
                        optimal_error = optimal_error * self.padding_weight
                    if inTrack:
                        added_optimal = True
                        self.addState(to_point_optimal, optimal_theta1, optimal_theta2, optimal_error)
                        if self.on_optimal_path and optimal_error <= self.offset_treshold:
                            self.could_left_at.append((self.pos.x, self.pos.y))

                #check if we are off the optimal path and searching for it
                if self.left_track_at != (-1,-1):
                    (a,b), _, _, _ = self.fromPoints[(self.left_track_at.x, self.left_track_at.y)]
                if abs(optimal_error) <= self.offset_treshold and not self.on_optimal_path and self.ec.isAboveEnd(Point(a,b),self.left_track_at, to_point_optimal) and added_optimal:  #and abs(currentError)<= self.offset_treshold
                    self.on_optimal_path = True
                    #gather the path to make better, find startPoint
                    prex = self.left_track_at.x
                    prey = self.left_track_at.y
                    for r in range (0, self.go_back_steps): #go back two steps to be able to adapt the path
                        if prex == vs.x and prey == vs.y:
                            #we have reached the startPoint
                            break
                        ((nx,ny),nth1, nth2, nerr) = self.fromPoints[(prex,prey)]
                        prex=nx
                        prey=ny
                    #gather the toalError of the path that's off the optimal path
                    totError = self.gatherError(Point(prex,prey), to_point_optimal, Point(vs.x, vs.y)) + abs(optimal_error)
                    #create new errorCalc and take it to the correct point
                    new_ec = error_calc.errorCalc(self.optimal_path)
                    tempPath = self.gatherPathMiddle(Point(vs.x, vs.y), Point(nx,ny), nth1, nth2, nerr)
                    for (((a,b),_,_,_)) in list(reversed(tempPath)):
                        new_ec.calculateError(Point(a,b))
                    #Gather a new optimized path for the parts that go off the optimal path
                    (fromP, part) = self.recalculate_path.calculate_path(Point(prex,prey), self.pos, to_point_optimal, self.dt, nth1, nth2, totError, new_ec)
                    #reset the previous toVisit stack and extend the fromPoints to also include the adjusted path
                    if len(part)>0:
                        self.fromPoints.update(fromP)
                        #self.fromPoints[(part[0][0][0],part[0][0][1])] = self.fromPoints[(nx,ny)]
                        self.toVisit = [part[-1]]
                        self.could_left_at = []

                #mark the previous node/state as visited
                self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))
        print "no soluton found"
        return []

    def calculate_steering(self, steering_min, steering_max, dd, iters):
        #Calculates a point within 1 unit of the optimal path, return the closest possibility if we cant find the optimal path
        steering_new = (steering_min + steering_max)/2
        (new_point, t1, t2, error) = self.calculateNextState(dd, steering_new)
        if abs(error)<0.1 or iters==0:
            return (new_point, t1, t2, error)
        elif error<0:
            #search right
            return self.calculate_steering(steering_new, steering_max, dd, iters-1)
        else:
            #search left
            return self.calculate_steering(steering_min, steering_new, dd, iters-1)

    def gatherPath(self, startPoint, endPoint, end_theta1, end_theta2):
        path = []
        #path.append(vehicleState(endPoint.x, endPoint.y, end_theta1, end_theta2))
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2)
        prex = self.pos.x
        prey = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(vehicleState(prex,prey, pret1, pret2))
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
        return path

    def gatherPathMiddle(self, startPoint, endPoint, end_theta1, end_theta2, end_err):
        path = []
        prex = endPoint.x
        prey = endPoint.y
        pret1 = end_theta1
        pret2 = end_theta2
        prerr = end_err
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(((prex,prey), pret1, pret2, prerr))
            #TODO: Maybe add error to final path
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
            prerr = err
        return path

    def gatherError(self, startPoint, endPoint, firstPoint):
        print startPoint.x, startPoint.y
        prex = endPoint.x
        prey = endPoint.y
        totErr = 0
        while not (prex== startPoint.x and  prey == startPoint.y):
            if (prex,prey) in self.could_left_at:
                break
            ((nx,ny),_, _, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            totErr= totErr+ abs(err)
        if not prex == firstPoint.x and not prey == firstPoint.y:
            ((nx,ny),_, _, err) = self.fromPoints[prex,prey]
            totErr= totErr + abs(err)
        return totErr


    def calculateNextState(self, dd, steering_angle_rad):
        next_theta1 = self.theta1 + (dd * tan(steering_angle_rad)) / self.length_header
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2))/ self.length_trailer
        next_x = self.pos.x + dd * cos(next_theta1)
        next_y = self.pos.y - dd * sin(next_theta1)  # Subtracting instead of adding, since the y-axis is flipped
        error= self.ec.calculateError(Point(next_x, next_y))

        return (Point(next_x,next_y), next_theta1, next_theta2, error)

    def gatherFromPoints(self, startPoint, endPoint):
        prex = endPoint.x
        prey = endPoint.y
        fromPoints = []
        while not (prex== startPoint.x and  prey == startPoint.y):
            ((nx,ny), _, _, _) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            fromPoints.append((nx,ny))
        fromPoints.append((nx,ny))
        return fromPoints


    def addState(self, point, th1, th2, error):
        #add the vector as an adjacent vector to the previous vector in the graph
        self.fromPoints[(point.x, point.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, error)
        self.toVisit.append(((point.x,point.y), th1, th2, error))

    def setOptimalpath(self, path):
        self.optimal_path = path
        self.ec = error_calc.errorCalc(path)

    def setMap(self, mat):
        self.trackChecker.setMap(mat)
