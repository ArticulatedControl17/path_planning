#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from custom_msgs.msg import *
from custom_msgs.srv import *
import time
import os
from math import *
import sys
from math import *
from vehicleState import *
from pathPlanning import *
import ref_path
from map_func import *
from Point import *

def hasPassedLine(p, (l1, l2)):

    if l1.x - l2.x !=0 and l1.y - l2.y !=0:
        slope = float(l1.y - l2.y) / float(l1.x - l2.x)
        prependularSlope = (-1)/slope
        prependularM = l2.y - l2.x*prependularSlope

        if l1.y < l2.y:
            #up
            return (p.x*prependularSlope + prependularM - p.y) < 0
        else:
            #down
            return (p.x*prependularSlope + prependularM - p.y) > 0

    elif l1.x - l2.x:
        #straight in x direction
        if l1.x < l2.x:
            #right
            return p.x > l2.x
        else:
            #left
            return p.x < l2.x

    else:
        #straight in y direction
        if l1.y < l2.y:
            #up
            return p.y > l2.y
        else:
            #down
            return p.y < l2.y




class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)

        self.i = 0

        self.map_obj = Map()
        self.ref_obj = ref_path.RefPath()

        self.map, self.scale = self.map_obj.getMapAndScale()
        self.pathplanner = PathPlanner(self.map)

        self.refpath = None

        self.wait_for_map_update = False

        self.latest_state = None

        self.current_start_state = None
        self.current_path = []

        self.active = False

        self.latest_ts = None

        self.path_append_publisher = rospy.Publisher('path_append', Path, queue_size=10)
        self.path_rework_publisher = rospy.Publisher('path_rework', Path, queue_size=10)
        self.refpath_publisher = rospy.Publisher('ref_path', Path, queue_size=10)


        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)
        rospy.Subscriber('truck_state', TruckState, self.truckStateHandler)

        self.path_request_srv = rospy.Service('request_path', RequestPath, self.requestPathHandler)


    def isVehicleStateOK(self, state):
        r = self.pathplanner.checkIfInTrack(state)
        if not r:
            print "not OK: ", state.x, state.y, state.theta1, state.theta2
        return r

    def isCurrentPlanOK(self):
        i = 0
        for state in self.current_path:
            if not self.isVehicleStateOK(state):
                return (False,i)
            i += 1

        return (True,0)

    def updateMap(self, obst):
        print "update map", obst
        add = self.map_obj.addObstacle(obst)
        if not add:
            rem = self.map_obj.removeObstacle(obst)
            if not rem:
                print "can't add or remove obstacle"

        self.map, _ = self.map_obj.getMapAndScale()


    def mapUpdateHandler(self, data):

        print "cur_path before"
        for c in self.current_path:
            print c.x, c.y

        self.wait_for_map_update = True

        self.updateMap(data.data)
        self.pathplanner.setMap(self.map)

        (ok, i) = self.isCurrentPlanOK()

        if ok:
            print "cur path ok"


        else:
            print "cur path NOT ok"
            if i-10 < 0:
                self.current_start_state = self.latest_state
                self.current_path = []
            else:
                self.current_start_state = self.current_path[i-10]
                self.current_path = self.current_path[:i-10+1]

            p = []
            for state in self.current_path:
                xx = round(state.x * self.scale)
                yy = round(state.y * self.scale)
                p.append(Position(xx,yy))

            self.path_rework_publisher.publish(Path(p))

            self.i = self.getClosestIndex(self.refpath, (self.current_start_state.x, self.current_start_state.y))

            self.active = True

        self.wait_for_map_update = False
        print "cur_path after"
        for c in self.current_path:
            print c.x, c.y
        print


    def getClosestIndex(self, refpath, (x,y)):
        i = 0
        minl = 9999
        minindex = 0
        for rx,ry in refpath:
            dx = rx - x
            dy = ry - y
            l = sqrt(dx**2 + dy**2)
            if l < minl:
                minl = l
                minindex = i
            i += 1
        return minindex

    def traverseCurrentPath(self):


        p = (self.latest_state.x, self.latest_state.y)

        path = list(self.current_path)


        if len(path) < 2:
            return

        l1 = path.pop(0)
        l2 = path.pop(0)



        while hasPassedLine(Point(*p), (Point(l1.x, l1.y), Point(l2.x, l2.y))):


            if len(path) == 0:
                self.current_path = [l2]
                return


            l1 = l2
            l2 = path.pop(0)

        self.current_path = [l1, l2] + path




    def truckStateHandler(self, data):
        self.latest_state = VehicleState(data.p.x / self.scale, data.p.y / self.scale, data.theta1, data.theta2)

        self.traverseCurrentPath()


    def requestPathHandler(self, data):

        s = data.state

        start = ref_path.VehicleState(s.p.x, s.p.y, degrees(-s.theta1), degrees(-s.theta2))
        goal = (data.goal.x / self.scale, data.goal.y / self.scale)

        response = RequestPathResponse()

        print
        rp = self.ref_obj.getRefPath(start)
        rp = [(x/ float(self.scale), y / float(self.scale)) for x,y in rp]
        #rp = [(1161, 7145), (1161, 6939), (1162, 6733), (1162, 6527), (1162, 6321), (1163, 6115), (1163, 5909), (1163, 5703), (1164, 5497), (1164, 5291), (1165, 5085), (1165, 4878), (1165, 4672), (1166, 4466), (1166, 4260), (1170, 4200), (1377, 4198), (1584, 4197), (1791, 4195), (1998, 4194), (2205, 4192), (2412, 4191), (2620, 4190), (2650, 4293), (2695, 4395), (2752, 4492), (2821, 4580), (2901, 4658), (2990, 4726), (3088, 4781), (3191, 4824), (3300, 4850), (3295, 5050), (3290, 5250), (3286, 5450)]
        if rp == []:
            response.success = False
            response.message = "Couldn't find reference path. Possible cause: start or goal way off"

        else:
            response.success = True

            self.i = 0

            self.current_start_state = VehicleState(s.p.x / self.scale, s.p.y / self.scale, -s.theta1, -s.theta2)

            self.refpath = rp
            self.pathplanner.setOptimalpath(rp)

            p = [Position(x*10,y*10) for x,y in self.refpath]
            self.refpath_publisher.publish(Path(p))

            self.wait_for_map_update = False


            c = self.current_start_state
            self.current_path = [VehicleState(c.x, c.y, c.theta1, c.theta2)]
            self.active = True


        return response





    def spin(self):

        while not rospy.is_shutdown():
            if self.active and not self.wait_for_map_update:

                done = False
                if self.i + 10 >= len(self.refpath) - 1:
                    g = self.refpath[-1]
                    g2 = self.refpath[-2]
                    done = True
                else:
                    g = self.refpath[self.i + 10]
                    g2 = self.refpath[self.i + 9]


                print self.current_start_state.x, self.current_start_state.y, self.current_start_state.theta1, self.current_start_state.theta2
                print g

                path = self.pathplanner.getPath(self.current_start_state, g, g2)
                if self.wait_for_map_update: #map updated while planning
                    continue
                print "path from planning"
                for pp in path:
                    print pp.x, pp.y, pp.theta1, pp.theta2
                print

                if path == []:
                    print "Can't find a path"
                    self.active = False
                    continue


                ti = int(ceil(len(path)/3.0))
                if done:
                    ti = len(path)-1
                    print "done"
                self.current_start_state = path[ti]
                print ti, len(path)
                app_path = path[:ti+1]
                self.current_path += app_path

                p = []
                for state in app_path:
                    xx = round(state.x * self.scale)
                    yy = round(state.y * self.scale)
                    p.append(Position(xx,yy))

                if done:
                    p.append(Position(-1, -1))

                self.path_append_publisher.publish(Path(p))

                if done:
                    self.active = False
                    continue

                self.i += 5






if __name__ == '__main__':
    p = PathPlanningNode()
    p.spin()
