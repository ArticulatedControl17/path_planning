
from math import *
import sys
import decimal
import model
import error_calc
import os

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
        self.solutions = 100

        self.all_paths = []
        self.model = model.truck()
        dirpath = os.path.dirname(os.path.abspath(__file__))
        f = open(dirpath+ '/optimal_path.txt', 'w')
        f.write('75 400\n400 75')
        f.close()
        self.ec = error_calc.errorCalc("optimal_path.txt")


    def checkIfInTrack(self, toPoint, th1, th2):

        p0 = Point(100,100) #corner position

        points = self.model.calculateCorners(toPoint, th1, th2)
        for (x,y) in points:
            if x <0 or y <0 or y >= 600 or x >=600 :
                #truck to the left of track
                return False
            if x>100 and y>100:
                return False

        right_wheel = Point(points[5][0], points[5][1])

        prev_points = self.model.calculateCorners(self.pos, self.theta1, self.theta2)
        prev_right_wheel = Point(prev_points[5][0], prev_points[5][1])

        #isLeft = ((right_wheel.x - self.prev_right_wheel.x)*(p0.y - self.prev_right_wheel.y) - (right_wheel.y - self.prev_right_wheel.y)*(p0.x - self.prev_right_wheel.x)) >=0

        #TODO: Really slow, optimize
        between = self.getPointsInBetween((right_wheel.x, right_wheel.y), (prev_right_wheel.x, prev_right_wheel.y), self.dt)

        for (x,y) in between:
            if x>100 and y>100:
                return False
        return True

    def creategraph(self, startPoint, endPoint, dt):

        self.theta1 = radians(90)
        self.theta2 = radians(90)
        self.dt = dt
        self.paths = []
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
            if dist < self.dt:
                #reached end, gather the path
                path = self.gatherPath(startPoint, endPoint)
                if self.findOptimalPath:
                    self.paths = self.paths + [path]
                else:
                    self.paths = self.paths + [path]


            #goin straight
            dd = self.speed * self.dt
            steering_angle_rad = radians(0)

            (to_point_strait, strait_theta1, strait_theta2) = self.calculateNextState(dd, steering_angle_rad)

            #going right
            steering_angle_rad = radians(16) #max right angle

            (to_point_right,right_theta1,right_theta2) = self.calculateNextState(dd, steering_angle_rad)

            steering_angle_rad = radians(8) #middle right angle

            (to_point_right_narrow,right_theta_narrow1,right_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)

            #going left
            steering_angle_rad = radians(-21) #max left angle

            (to_point_left,left_theta1, left_theta2) = self.calculateNextState(dd, steering_angle_rad)

            steering_angle_rad = radians(-10) #middle left angle

            (to_point_left_narrow,left_theta_narrow1,left_theta_narrow2) = self.calculateNextState(dd, steering_angle_rad)

            #see if the endpoint is to the right/left/strait and go correct direction from there

            goRightOrLeft = self.ec.calculateError(Point(self.pos.x,self.pos.y))
            #TODO: Never goes strait, change to following a line, and add stearing angle according to distance from line

            if goRightOrLeft<=5 and goRightOrLeft>=-5:
                #Go strait
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(to_point_left,left_theta1, left_theta2):
                    self.addState(to_point_left, left_theta1, left_theta2)
                #if self.checkIfInTrack(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2):
                #    self.addState(to_point_left_narrow, left_theta_narrow1, left_theta_narrow2)
                if self.checkIfInTrack(to_point_right, right_theta1, right_theta2):
                    self.addState(to_point_right, right_theta1, right_theta2)
                #if self.checkIfInTrack(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2):
                #    self.addState(to_point_right_narrow, right_theta_narrow1, right_theta_narrow2)
                if self.checkIfInTrack(to_point_strait, strait_theta1, strait_theta2):
                    self.addState(to_point_strait, strait_theta1, strait_theta2)

            elif goRightOrLeft<-5:
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

            #mark the previous vector as visited
            self.visited.add(((self.pos.x, self.pos.y),self.theta1, self.theta2))

            if len(self.paths)>100 and not self.findOptimalPath:
                shortest = self.getShortest_area(self.paths)
                sorted_list = sorted(shortest, cmp=lambda (a,l1),(b,l2): cmp(a,b))
                for (x,path) in sorted_list:
                    print x
                return sorted_list[0][1]

        if len(self.all_paths)>self.solutions:
            sorted_list = sorted(self.all_paths, cmp=lambda (a,l1),(b,l2): cmp(a,b))
            for (x,path) in sorted_list:
                print x
            self.findOptimalPath = False
            dirpath = os.path.dirname(os.path.abspath(__file__))
            f = open(dirpath+ '/optimal_path.txt', 'w')
            f.write(str(startPoint.x)+ ' ' +str(startPoint.y)+'\n')

            return sorted_list[0]


            for ((xa,ya),ta1, ta2) in reversed(sorted_list[0][1]):
                f.write(str(xa)+ ' ' +str(ya)+'\n')
            f.close()
            self.ec = error_calc.errorCalc("optimal_path.txt")
            return self.creategraph(startPoint, endPoint, 35)
            #return sorted_list[1]

        if len(self.paths)>0:
            #found solution
            print "dt: ", self.dt
            shortest = self.getShortest_area(self.paths)
            self.all_paths = self.all_paths + shortest
            return self.creategraph(startPoint, endPoint, self.dt-1)
        else:
            print "dt: ", self.dt
            return self.creategraph(startPoint, endPoint, self.dt-1)
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
        path.append(((startPoint.x,startPoint.y), radians(90), radians(90)))
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
        #TODO: make new getShortest that doesn't compare distance, maybe area?
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
