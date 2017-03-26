from error_calc import errorCalc
from math import *
from Point import Point

class recalculatePath:

    def __init__(self, speed, length_header, length_trailer, trackChecker):
        self.speed = speed
        self.length_header = length_header
        self.length_trailer = length_trailer
        self.trackChecker = trackChecker


    def calculate_path(self, startPoint, snd_to_last_end, endPoint, dt, theta1, theta2, totError, errorC):


        self.start_theta1 = theta1
        self.start_theta2 = theta2
        self.start_err = errorC.calculateError(startPoint)

        self.theta1 = theta1 #start angle for header
        self.theta2 = theta2 #start angle for trailer
        self.dt = dt #the delta time used for kinematic model, basicly the path step size
        self.path = ([],[]) #first list is the fromPoints, second list is the solution path
        self.ec = errorC #make new error calc every time to reset it and look from the beginning

        self.lowest_error = totError
        self.totError = 0
        dd = self.speed * self.dt

        self.pos = startPoint
        self.fromPoints = {}
        self.toVisit = []
        self.visited = set([]) #the points we have visited
        #add all possible pathes for the first point before looping
        steering_angle_rad = radians(0)
        (to_point_strait, strait_theta1, strait_theta2, strait_error) = self.calculateNextState(dd, steering_angle_rad)
        #going right
        steering_angle_rad = radians(16) #max right angle
        (to_point_right,right_theta1,right_theta2, right_error) = self.calculateNextState(dd, steering_angle_rad)
        #going left
        steering_angle_rad = radians(-21) #max left angle
        (to_point_left,left_theta1, left_theta2, left_error) = self.calculateNextState(dd, steering_angle_rad)
        #finding optimal path
        (to_point_optimal, optimal_theta1, optimal_theta2, optimal_error) = self.calculate_steering(radians(-21), radians(16), dd, 10)
        #check if the nodes are within the allowed track and we haven't reached the maximum error
        if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt) and self.totError+left_error <= self.lowest_error:
            self.addState(to_point_left, left_theta1, left_theta2, left_error)
        if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt) and self.totError+strait_error <= self.lowest_error:
            self.addState(to_point_strait, strait_theta1, strait_theta2, strait_error)
        if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt) and self.totError+right_error <= self.lowest_error:
            self.addState(to_point_right, right_theta1, right_theta2, right_error)
        #find the optimal path, return max steering angle if we cant reach the optimal path
        if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt) and self.totError+optimal_error <= self.lowest_error:
            self.addState(to_point_optimal, optimal_theta1, optimal_theta2, optimal_error)

        print len(self.toVisit)
        count = 0
        while len(self.toVisit)>=0:
            count= count+1
            print "count: ", count
            #loop until all possible nodes have been visited
            while True:
                if len(self.toVisit)==0:
                    print "reached End"
                    print "lowest error: ", self.lowest_error
                    print "start error: ", totError
                    return self.path
                ((x,y),t1, t2, err, toterr) = self.toVisit.pop(0)
                #round to make it faster, not having to visit as many nodes that are similar
                round_x= round(x,0)
                round_y= round(y,0)
                round_theta1 = round(t1, 1)
                round_theta2 = round(t2, 1)
                if ((round_x,round_y),round_theta1, round_theta2) not in self.visited:
                    break
            #found new node to visit
            self.pos = Point(x,y) # get the toPoint
            self.theta1= t1
            self.theta2= t2
            self.totError = toterr
            #check if we have reached the end
            dist = sqrt( (endPoint.x - x)**2 + (endPoint.y - y)**2 )
            if self.ec.isAboveEnd(snd_to_last_end, endPoint, self.pos) and dist <5*self.dt: #checks if we are above a line of the two last points
                #reached a sloution, gather the path if we are on the optimal path
                (new_point, nt1, nt2, nerror) = self.calculate_steering(radians(-21), radians(16), dd, 10)
                if abs(nerror)< 1 and toterr < self.lowest_error:
                    self.path = self.gatherPath(startPoint, new_point, nt1, nt2, nerror, err)
                    self.lowest_error = toterr
                #mark visited
                self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

            else:
                #we have not yet found a solution, search for new possible nodes
                current_err = self.ec.calculateError(Point(self.pos.x,self.pos.y)) #check if we are left or right of the optimal path
                dd = self.speed * self.dt

                steering_angle_rad = radians(0) #strait

                (to_point_strait, strait_theta1, strait_theta2, strait_error) = self.calculateNextState(dd, steering_angle_rad)

                #going right
                steering_angle_rad = radians(16) #max right angle

                (to_point_right,right_theta1,right_theta2, right_error) = self.calculateNextState(dd, steering_angle_rad)

                #going left
                steering_angle_rad = radians(-21) #max left angle

                (to_point_left,left_theta1, left_theta2, left_error) = self.calculateNextState(dd, steering_angle_rad)

                #finding optimal path
                (to_point_optimal, optimal_theta1, optimal_theta2, optimal_error) = self.calculate_steering(radians(-21), radians(16), dd, 10)

                #check if the nodes are within the allowed track and we haven't reached the maximum error
                if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_left, left_theta1, left_theta2, self.dt) and self.totError+left_error <= self.lowest_error:
                    self.addState(to_point_left, left_theta1, left_theta2, left_error)
                if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_strait, strait_theta1, strait_theta2, self.dt) and self.totError+strait_error <= self.lowest_error:
                    self.addState(to_point_strait, strait_theta1, strait_theta2, strait_error)
                if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_right, right_theta1, right_theta2, self.dt) and self.totError+right_error <= self.lowest_error:
                    self.addState(to_point_right, right_theta1, right_theta2, right_error)
                #find the optimal path, return max steering angle if we cant reach the optimal path
                if self.trackChecker.checkIfInTrack(self.pos, self.theta1, self.theta2, to_point_optimal, optimal_theta1, optimal_theta2, self.dt) and self.totError+optimal_error <= self.lowest_error:
                    self.addState(to_point_optimal, optimal_theta1, optimal_theta2, optimal_error)

                round_x= round(self.pos.x,0)
                round_y= round(self.pos.y,0)
                round_theta1 = round(self.theta1, 1)
                round_theta2 = round(self.theta2, 1)
                self.visited.add(((round_x, round_y),round_theta1, round_theta2))

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

    def gatherPath(self, startPoint, endPoint, end_theta1, end_theta2, end_err, err):
        path = []
        fromPoints = {}
        path.append(((endPoint.x,endPoint.y), end_theta1, end_theta2, end_err))
        self.fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, err)
        fromPoints[(endPoint.x, endPoint.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, err)
        prex = self.pos.x
        prey = self.pos.y
        pret1 = self.theta1
        pret2 = self.theta2
        prerr = err
        while not (prex== startPoint.x and  prey == startPoint.y):
            path.append(((prex,prey), pret1, pret2, prerr))
            ((nx,ny),nt1, nt2, err) = self.fromPoints[prex,prey]
            fromPoints[(prex, prey)] = ((nx,ny),nt1, nt2, err)
            prex=nx
            prey=ny
            pret1 = nt1
            pret2 = nt2
            prerr = err
        path.append(((startPoint.x,startPoint.y), self.start_theta1, self.start_theta2, self.start_err))
        return (fromPoints, list(reversed(path)))

    def calculateNextState(self, dd, steering_angle_rad):
        next_theta1 = self.theta1 + (dd * tan(steering_angle_rad)) / self.length_header
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2))/ self.length_trailer
        next_x = self.pos.x + dd * cos(next_theta1)
        next_y = self.pos.y - dd * sin(next_theta1)  # Subtracting instead of adding, since the y-axis is flipped
        error = self.ec.calculateError(Point(next_x, next_y))

        return (Point(next_x,next_y), next_theta1, next_theta2, error)

    def addState(self, point, th1, th2, error):
        #add the vector as an adjacent vector to the previous vector in the graph
        self.fromPoints[(point.x, point.y)] = ((self.pos.x, self.pos.y),self.theta1, self.theta2, error)
        self.toVisit.append(((point.x,point.y), th1, th2, error, self.totError + abs(error)))
