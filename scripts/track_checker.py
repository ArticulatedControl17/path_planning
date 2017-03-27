import model
import numpy as np
import cv2
from Point import Point

class trackChecker:

    def __init__(self, mapp):
        self.model = model.truck() #model used to calculate error
        self.map = mapp #map with allowed/not allowed areas
    
    def checkIfInTrack2(self, toPoint, th1, th2):

        #check the range of the matrix with the allowed positions, to avoid index error
        if toPoint.x <0 or toPoint.y <0 or toPoint.x >540 or toPoint.y >950:
            return False

        points = self.model.calculateCorners(toPoint, th1, th2)

        if self.map[int(toPoint.y)][int(toPoint.x)]==0:
            return False


        right_back_wheel = Point(points[5][0], points[5][1])
        left_back_wheel = Point(points[4][0], points[4][1])
        right_front_wheel = Point(points[1][0], points[1][1])
        left_front_wheel = Point(points[0][0], points[0][1])

        #check right front wheels
        if right_front_wheel.x <0 or right_front_wheel.y <0 or right_front_wheel.x >540 or right_front_wheel.y >950:
            return False
        if self.map[int(right_front_wheel.y)][int(right_front_wheel.x)] ==0:
            return False
        #check left front wheels
        if left_front_wheel.x <0 or left_front_wheel.y <0 or left_front_wheel.x >540 or left_front_wheel.y >950:
            return False
        if self.map[int(left_front_wheel.y)][int(left_front_wheel.x)] ==0:
            return False
        
        
        #check right front wheels
        if right_back_wheel.x <0 or right_back_wheel.y <0 or right_back_wheel.x >540 or right_back_wheel.y >950:
            return False
        if self.map[int(right_back_wheel.y)][int(right_back_wheel.x)] ==0:
            return False
        #check left front wheels
        if left_back_wheel.x <0 or left_back_wheel.y <0 or left_back_wheel.x >540 or left_back_wheel.y >950:
            return False
        if self.map[int(left_back_wheel.y)][int(left_back_wheel.x)] ==0:
            return False
            
        return True
            
                
            
    

    def checkIfInTrack(self, prevPoint, prevth1, prevth2, toPoint, th1, th2, dt, error):
        #TODO: Make between points for front and back header wheels
        #check if point and key wheels are in the track

        inPadding = False

        if error > 2* dt:
            return (False, True)

        #check the range of the matrix with the allowed positions, to avoid index error
        if toPoint.x <0 or toPoint.y <0 or toPoint.x >540 or toPoint.y >950:
            return (False, True)

        points = self.model.calculateCorners(toPoint, th1, th2)

        if self.map[int(toPoint.y)][int(toPoint.x)]==0:
            return (False, True)
        if self.map[int(toPoint.y)][int(toPoint.x)]==2:
            inPadding = True


        right_back_wheel = Point(points[5][0], points[5][1])
        left_back_wheel = Point(points[4][0], points[4][1])
        right_front_wheel = Point(points[1][0], points[1][1])
        left_front_wheel = Point(points[0][0], points[0][1])

        #check right front wheels
        if right_front_wheel.x <0 or right_front_wheel.y <0 or right_front_wheel.x >540 or right_front_wheel.y >950:
            return (False, True)
        if self.map[int(right_front_wheel.y)][int(right_front_wheel.x)] ==0:
            return (False, True)
        if self.map[int(right_front_wheel.y)][int(right_front_wheel.x)] ==2:
            inPadding = True
        #check left front wheels
        if left_front_wheel.x <0 or left_front_wheel.y <0 or left_front_wheel.x >540 or left_front_wheel.y >950:
            return (False, True)
        if self.map[int(left_front_wheel.y)][int(left_front_wheel.x)] ==0:
            return (False, True)
        if self.map[int(left_front_wheel.y)][int(left_front_wheel.x)] ==2:
            inPadding = True

        prev_points = self.model.calculateCorners(prevPoint, prevth1, prevth2)
        prev_right_back_wheel = Point(prev_points[5][0], prev_points[5][1])
        prev_left_back_wheel = Point(prev_points[4][0], prev_points[4][1])
        #prev_right_front_wheel = Point(points[1][0], points[1][1])
        #prev_left_front_wheel = Point(points[0][0], points[0][1])

        #TODO: affects performance quite a lot with dt amount of points
        between_right = self.getPointsInBetween((right_back_wheel.x, right_back_wheel.y), (prev_right_back_wheel.x, prev_right_back_wheel.y), dt/1)
        between_left = self.getPointsInBetween((left_back_wheel.x, left_back_wheel.y), (prev_left_back_wheel.x, prev_left_back_wheel.y), dt/1)


        #check right back wheel
        for (x,y) in between_right:
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x] ==0:
                return (False, True)
            if self.map[y][x] ==2:
                inPadding = True
        for (x,y) in between_left:
            #Check left back wheel
            if x <0 or y <0 or x >540 or y >950:
                return (False, True)
            if self.map[y][x]==0:
                return (False, True)
            if self.map[y][x]==2:
                inPadding = True
        return (True, inPadding)

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

    def setMap(self, mapp):
        self.map = mapp
