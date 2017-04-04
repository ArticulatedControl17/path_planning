from error_calc import errorCalc
from math import *
from Point import Point
from vehicleState import VehicleState
import rospy
from custom_msgs.msg import Path, Position
from helper_functions import *

class recalculatePath:

    def __init__(self, speed, trackChecker):
        self.speed = speed
        self.trackChecker = trackChecker

        self.path_pub = rospy.Publisher('possible_path', Path, queue_size=10)


    def calculate_path(self, startPoint, snd_to_last_end, endPoint, dt, theta1, theta2, totError, ec):

        self.start_theta1 = theta1
        self.start_theta2 = theta2
        self.errorList = {}

        self.theta1 = theta1 #start angle for header
        self.theta2 = theta2 #start angle for trailer
        self.dt = dt #the delta time used for kinematic model, basicly the path step size
        self.front_ec_i = 1
        self.back_ec_i = 1
        self.ec = ec
        self.path = ([],[], self.ec) #first list is the fromPoints, second list is the solution path

        self.lowest_error = totError
        self.totError = 0
        dd = self.speed * self.dt

        self.pos = startPoint
        self.fromPoints = {}
        self.toVisit = []
        self.visited = set([]) #the points we have visited
        #add all possible pathes for the first point before looping
        self.addPossiblePathes()

        print len(self.toVisit)
        count = 0
        while len(self.toVisit)>=0:
            count= count+1
            #loop until all possible nodes have been visited
            while True:
                if len(self.toVisit)==0:
                    print "reached End"
                    print "lowest error: ", self.lowest_error
                    print "start error: ", totError
                    print "starPoint: ", startPoint.x, startPoint.y
                    print "endPoint: ", endPoint.x, endPoint.y
                    return self.path
                ((x,y),t1, t2, err, toterr, new_front_ec_i, new_back_ec_i) = self.toVisit.pop()
                #round to make it faster, not having to visit as many nodes that are similar

                ((round_x, round_y), round_theta1, round_theta2) = rounding(x, y, t1, t2)
                prev_err = self.errorList[((round_x, round_y), round_theta1, round_theta2)]
                if ((round_x,round_y),round_theta1, round_theta2) not in self.visited: #or prev_err>toterr:
                    break
            #found new node to visit
            self.pos = Point(x,y) # get the toPoint
            #print x,y,err, toterr
            self.front_ec_i = new_front_ec_i
            self.back_ec_i = new_back_ec_i
            self.theta1= t1
            self.theta2= t2
            self.totError = toterr
            #check if we have reached the end
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            if self.ec.isAboveEnd(snd_to_last_end, endPoint, self.pos) and dist <1*self.dt and self.ec.isAtEnd(self.front_ec_i): #checks if we are above a line of the two last points
                #reached a sloution, gather the path if we are on the optimal path
                count = count+1
                print count
                #rospy.sleep(0.2)
                (new_point, nt1, nt2) = calculate_steering(radians(MAX_RIGHT_ANGLE), radians(MAX_LEFT_ANGLE), dd, 10, 0, self.pos, self.theta1, self.theta2, self.ec, self.front_ec_i)
                (nerror, n_i)= self.ec.calculateError(new_point, self.front_ec_i)
                self.path_pub.publish(Path(self.gather_x_y_path(startPoint, new_point, nt1, nt2, nerror)))
                if abs(nerror)< 1 and toterr < self.lowest_error:
                    print "found better solution"
                    self.path = self.gatherPath(startPoint, new_point, nt1, nt2, nerror, err)
                    self.lowest_error = toterr
                #mark visited
                self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

            else:
                #we have not yet found a solution, search for new possible nodes
                self.addPossiblePathes()

                ((round_x, round_y), round_th1, round_th2) = rounding(self.pos.x, self.pos.y, self.theta1, self.theta2)
                self.visited.add(((round_x, round_y),round_theta1, round_theta2))

    def addPossiblePathes(self):
        dd = self.speed * self.dt

        steering_angle_rad = radians(0) #strait

        (to_point_strait, strait_theta1, strait_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)

        #going right
        steering_angle_rad = radians(MAX_LEFT_ANGLE) #max right angle

        (to_point_right,right_theta1,right_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)

        #going left
        steering_angle_rad = radians(MAX_RIGHT_ANGLE) #max left angle

        (to_point_left,left_theta1, left_theta2) = calculateNextState(self.theta1, self.theta2, self.pos, dd, steering_angle_rad)

        #finding optimal path
        (to_point_optimal, optimal_theta1, optimal_theta2) = calculate_steering(radians(MAX_RIGHT_ANGLE), radians(MAX_LEFT_ANGLE), dd, 10, 0, self.pos, self.theta1, self.theta2, self.ec, self.front_ec_i)

        #finding optimal outside turn path
        goingLeft = self.ec.is_next_Left(self.front_ec_i)
        if goingLeft:
            (to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2) = calculate_steering(radians(MAX_RIGHT_ANGLE), radians(MAX_LEFT_ANGLE), dd, 10, OUTSIDE_TURN_ERROR, self.pos, self.theta1, self.theta2, self.ec, self.front_ec_i)
        else:
            (to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2) = calculate_steering(radians(MAX_RIGHT_ANGLE), radians(MAX_LEFT_ANGLE), dd, 10, -OUTSIDE_TURN_ERROR, self.pos, self.theta1, self.theta2, self.ec, self.front_ec_i)


        #check if the nodes are within the allowed track and we haven't reached the maximum error
        #Left
        (inTrack, truck_error, b_ec_i, f_ec_i) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt, self.ec, self.front_ec_i, self.back_ec_i)
        if inTrack and self.totError+truck_error <= self.lowest_error:
            self.addState(to_point_left, left_theta1, left_theta2, truck_error, b_ec_i, f_ec_i)
        #Strait
        (inTrack, truck_error, b_ec_i, f_ec_i) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt, self.ec, self.front_ec_i, self.back_ec_i)
        if inTrack and self.totError+truck_error <= self.lowest_error:
            self.addState(to_point_strait, strait_theta1, strait_theta2, truck_error, b_ec_i, f_ec_i)
        #Right
        (inTrack, truck_error, b_ec_i, f_ec_i) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt, self.ec, self.front_ec_i, self.back_ec_i)
        if inTrack and self.totError+truck_error <= self.lowest_error:
            self.addState(to_point_right, right_theta1, right_theta2, truck_error, b_ec_i, f_ec_i)
        #Optimal outside turn
        (inTrack, truck_error, b_ec_i, f_ec_i) =self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2, self.dt, self.ec, self.front_ec_i, self.back_ec_i)
        if inTrack:
            self.addState(to_point_optimal_outside, optimal_outside_theta1, optimal_outside_theta2, truck_error, b_ec_i, f_ec_i)
        #Optimal
        (inTrack, truck_error, b_ec_i, f_ec_i) = self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt, self.ec, self.front_ec_i, self.back_ec_i)
        if inTrack and self.totError+truck_error <= self.lowest_error:
            self.addState(to_point_optimal, optimal_theta1, optimal_theta2, truck_error, b_ec_i, f_ec_i)


    def gatherPath(self, startPoint, endPoint, end_theta1, end_theta2, end_err, err):
        path = []
        fromPoints = {}
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, err)
        fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, err)
        prex = self.pos.x
        prey = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        prerr = err
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(VehicleState(prex,prey, pret1, pret2))
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            fromPoints[(prex, prey)] = ((nx,ny),nt1, nt2, err)
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
            prerr = err
        return (fromPoints, list(reversed(path)), self.ec)

    def gather_x_y_path(self, startPoint, endPoint, end_theta1, end_theta2, err):
        path = []
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, err)
        prex = self.pos.x
        prey = self.pos.y
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(Position(prex,prey))
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            prex=nx
            prey=ny
        return list(reversed(path))


    def addState(self, point, th1, th2, error, b_ec_i, f_ec_i):
        #add the vector as an adjacent vector to the previous vector in the graph
        self.fromPoints[(point.x, point.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, error)
        self.toVisit.append(((point.x,point.y), th1, th2, error, self.totError + abs(error), b_ec_i, f_ec_i))
        ((round_x, round_y), round_th1, round_th2) = rounding(point.x, point.y, th1, th2)
        self.errorList[((round_x, round_y), round_th1, round_th2)] = self.totError + abs(error)
