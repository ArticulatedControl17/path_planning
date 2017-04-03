
from math import *
import sys
import error_calc
import os
import time
import track_checker
import recalculatePath
from Point import Point
from vehicleState import VehicleState
import model
import rospy
from custom_msgs.msg import Position
from helper_functions import *

from recalculatePath import w1, w2, w4, w5

class PathPlanner:
    #TODO: fix "other-lane-padding"
    #TODO: fix so that we dont stop before all of optimal path is visited.
    #TODO: allways return empty list

    def __init__(self, mapp):
        self.max_left_angle = -16
        self.speed = 1
        self.dt = 25 #the delta time used for kinematic model, basicly the path step size
        self.offset_treshold = 10
        self.trackChecker = track_checker.trackChecker(mapp)

        self.visited_pub = rospy.Publisher('visited_node', Position, queue_size=10)
        self.to_visit_pub = rospy.Publisher('to_visit_node', Position, queue_size=10)

    def getPath(self, vs, endPoint, secondEndPoint):

        endPoint = Point(*endPoint)
        secondEndPoint = Point(*secondEndPoint)

        self.recalculate_path = recalculatePath.recalculatePath(self.speed, LENGTH_HEADER, LENGTH_TRAILER, self.trackChecker, PADDING_WEIGHT)
        self.theta1 = vs.theta1 #start angle for header
        self.theta2 = vs.theta2 #start angle for trailer
        self.front_ec = error_calc.errorCalc(self.optimal_path) #make new error calc every time to reset it and look from the beginning
        self.back_ec = error_calc.errorCalc(self.optimal_path)
        self.pos = Point(vs.x, vs.y)
        self.fromPoints = {}
        self.toVisit = []
        self.visited = set([]) #the points we have visited

        count = 0

        self.addPossiblePathes(True)

        while len(self.toVisit)>0:
            count = count+1
            #loop until all possible nodes have been visited
            while True:
                if len(self.toVisit) == 0:
                    break
                ((x,y),t1, t2, err, new_front_ec, new_back_ec) = self.toVisit.pop()
                self.visited_pub.publish(Position(x,y))
                #round to not having to visit every mm, to make it faster
                round_x= round(x,0)
                round_y= round(y,0)
                round_theta1 = round(t1, 1)
                round_theta2 = round(t2, 1)
                if ((round_x,round_y),round_theta1, round_theta2) not in self.visited:
                    break
            #found new node to visit
            self.pos = Point(x, y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            self.front_ec= new_front_ec
            self.back_ec= new_back_ec

            #print count


            #check if we have reached the end
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            #TODO: Add so that we can get the second to last point from error calc
            if self.front_ec.isAboveEnd(secondEndPoint,endPoint, self.pos) and dist <1*self.dt: #checks if we are above a line of the two last points
                #reached end, gather the path
                print "reached end, Gathering solution"

                totError = self.gatherError(Point(vs.x, vs.y), self.pos, Point(vs.x, vs.y))
                #Gather a new optimized path for the parts that go off the optimal path
                ((nx,ny),_, _,_) = self.fromPoints[self.pos.x,self.pos.y]
                fromPoint = Point(nx,ny)
                #return self.gatherPath(Point(vs.x, vs.y), endPoint,self.theta1, self.theta2)
                (fromP, part, nn_ec) = self.recalculate_path.calculate_path(Point(vs.x, vs.y), secondEndPoint, endPoint, self.dt, vs.theta1, vs.theta2, totError, error_calc.errorCalc(self.optimal_path))
                #part = self.recalculate_path.calculate_path(Point(vs.x, vs.y), secondEndPoint, endPoint, self.dt, vs.theta1, vs.theta2, totError, error_calc.errorCalc(self.optimal_path))
                if part == []:
                    return self.gatherPath(Point(vs.x, vs.y), endPoint,self.theta1, self.theta2)
                return part

            else:
                #we have not yet found a solution, search for new possible nodes
                currentError = self.front_ec.calculateError(Point(self.pos.x,self.pos.y)) #check if we are left or right of the optimal path

                if currentError<0:
                    #Go right
                    #check if the nodes are within the allowed track
                    self.addPossiblePathes(False)
                else:
                    #Go left
                    #check if the nodes are within the allowed track
                    self.addPossiblePathes(True)

                #round to not having to visit every mm, for making it faster
                round_x= round(self.pos.x,0)
                round_y= round(self.pos.y,0)
                round_theta1 = round(self.theta1, 1)
                round_theta2 = round(self.theta2, 1)
                #mark the previous node/state as visited
                self.visited.add(((round_x, round_y),round_theta1, round_theta2))
        print "no soluton found"
        return []

    def addPossiblePathes(self, leftFirst):

        dd = self.speed * self.dt
        #add all possible pathes from self.pos, self.theta1 and self.theta2
        steering_angle_rad = radians(0)
        (to_point_strait, strait_theta1, strait_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)
        #going right
        steering_angle_rad = radians(self.max_left_angle) #max right angle
        (to_point_right,right_theta1,right_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)
        #going left
        steering_angle_rad = radians(16) #max left angle
        (to_point_left,left_theta1, left_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)
        #finding optimal path
        (to_point_optimal, optimal_theta1, optimal_theta2) = calculate_steering(radians(16), radians(self.max_left_angle), dd, 10, 0, self.pos, self.theta1, self.theta2, self.front_ec)
        #Optimal outside turn
        goingLeft = self.front_ec.is_next_Left()
        if goingLeft:
            (to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2) = calculate_steering(radians(16), radians(self.max_left_angle), dd, 10, 5, self.pos, self.theta1, self.theta2, self.front_ec)
        else:
            (to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2) = calculate_steering(radians(16), radians(self.max_left_angle), dd, 10, -5, self.pos, self.theta1, self.theta2, self.front_ec)

        #Strait
        (inTrack, tot_error) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt, self.front_ec, self.back_ec)
        if inTrack:
            self.addState(to_point_strait, strait_theta1, strait_theta2, tot_error)
        if leftFirst:
            #Right
            (inTrack, tot_error) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt, self.front_ec, self.back_ec)
            if inTrack:
                self.addState(to_point_right, right_theta1, right_theta2, tot_error)
            #Left
            (inTrack, tot_error) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt, self.front_ec, self.back_ec)
            if inTrack:
                self.addState(to_point_left, left_theta1, left_theta2, tot_error)
        else:
            #Left
            (inTrack, tot_error) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt, self.front_ec, self.back_ec)
            if inTrack:
                self.addState(to_point_left, left_theta1, left_theta2, tot_error)
            #Right
            (inTrack, tot_error) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt, self.front_ec, self.back_ec)
            if inTrack:
                self.addState(to_point_right, right_theta1, right_theta2, tot_error)
        #Optimal outside turn
        (inTrack, tot_error) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2, self.dt, self.front_ec, self.back_ec)
        if inTrack:
            self.addState(to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2, tot_error)
        #Optimal
        (inTrack, tot_error) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt, self.front_ec, self.back_ec)
        if inTrack:
            self.addState(to_point_optimal, optimal_theta1, optimal_theta2, tot_error)


    def gatherPath(self, startPoint, endPoint, end_theta1, end_theta2):
        path = []
        #path.append(vehicleState(endPoint.x, endPoint.y, end_theta1, end_theta2))
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2)
        prex = self.pos.x
        prey = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(VehicleState(prex,prey, pret1, pret2))
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
        return path[::-1]

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
            ((nx,ny),_, _, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
            totErr= totErr+ abs(err)
        if not prex == firstPoint.x and not prey == firstPoint.y:
            ((nx,ny),_, _, err) = self.fromPoints[prex,prey]
            totErr= totErr + abs(err)
        return totErr


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
        self.toVisit.append(((point.x,point.y), th1, th2, error, self.front_ec.getCopy(), self.back_ec.getCopy()))
        self.to_visit_pub.publish(Position(point.x, point.y))

    def setOptimalpath(self, path):
        print "setoptpath", path
        path = [Point(x,y) for x,y in path]
        self.optimal_path = path
        self.front_ec = error_calc.errorCalc(path)
        self.back_ec = error_calc.errorCalc(path)

    def setMap(self, mat):
        self.trackChecker.setMap(mat)

    def checkIfInTrack(self, vs):
        return self.trackChecker.checkIfInTrack2(Point(vs.x, vs.y), vs.theta1, vs.theta2)
