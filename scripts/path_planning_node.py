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
from Point import *
import time

from geometry_msgs.msg import PoseWithCovarianceStamped

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *

import ref_path


def getPointsInBetween(p1, p2, n):
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
        self.ref_obj = ref_path.RefPath() #ref_path_no_gui
        
        self.map, self.scale = self.map_obj.getMapAndScale()
        self.scale = float(self.scale)
        self.pathplanner = PathPlanner(self.map)

        self.refpath = None
        
        self.wait_for_map_update = False
        
        self.latest_state = None

        self.current_start_state = None
        self.current_path = []
        
        self.active = False
                
        self.latest_ts = None
        
        
        
        self.startend_publisher = rospy.Publisher('alg_startend', Path, queue_size=10)
        self.path_append_publisher = rospy.Publisher('path_append', Path, queue_size=10)
        self.path_rework_publisher = rospy.Publisher('path_rework', Path, queue_size=10)
        self.refpath_publisher = rospy.Publisher('ref_path', Path, queue_size=10)
        self.long_path_publisher = rospy.Publisher('long_path', Path, queue_size=10)
        
        
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initPoseCallback)
        
        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)
        rospy.Subscriber('truck_state', TruckState, self.truckStateHandler)
        
        self.path_request_srv = rospy.Service('request_path', RequestPath, self.requestPathHandler)


    def initPoseCallback(self, data):
        self.current_path = self.current_path[:1]
    
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
            
        #self.wait_for_map_update = False
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
        
        start = ref_path.VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)
        goals = [(float(p.x)/self.scale, float(p.y)/self.scale) for p in data.goals.path]
        print "goals", goals
        response = RequestPathResponse()     
        
        print 
        st = time.time()
        rp = self.ref_obj.getRefPath(start, goals)
        print "time", time.time()-st
        #rp = [(x/ float(self.scale), y / float(self.scale)) for x,y in rp]
        #rp = [(1161, 7145), (1161, 6939), (1162, 6733), (1162, 6527), (1162, 6321), (1163, 6115), (1163, 5909), (1163, 5703), (1164, 5497), (1164, 5291), (1165, 5085), (1165, 4878), (1165, 4672), (1166, 4466), (1166, 4260), (1170, 4200), (1377, 4198), (1584, 4197), (1791, 4195), (1998, 4194), (2205, 4192), (2412, 4191), (2620, 4190), (2650, 4293), (2695, 4395), (2752, 4492), (2821, 4580), (2901, 4658), (2990, 4726), (3088, 4781), (3191, 4824), (3300, 4850), (3295, 5050), (3290, 5250), (3286, 5450)]
        if rp == []:
            response.success = False
            response.message = "Couldn't find reference path. Possible cause: start or goal way off"

        else:
            response.success = True
            
            self.i = 0
            
            self.current_start_state = VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)
            
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
            if self.active:# and not self.wait_for_map_update:
                
                sub_target = 45
                
                done = False
                if self.i + sub_target >= len(self.refpath) - 1:
                    g = self.refpath[-1]
                    g2 = self.refpath[-2]
                    done = True
                else:
                    g = self.refpath[self.i + sub_target]
                    g2 = self.refpath[self.i + sub_target-1]
                    
                    latest = None
                    for j in range(self.i, sub_target):
                        p1,p2 = self.refpath[j], self.refpath[j+1]
                        pts = getPointsInBetween(p1,p2,6)
                        for x,y in pts:
                            if self.map[y][x] in [0,2]:
                                latest = (x,y)
                    print latest
                    if latest != None:
                        lfg = sqrt((g[0] - latest[0])**2 + (g[1] - latest[1])**2 )
                        if lfg <= 200:
                            print "lfg too close"
                            self.i += 7
                            continue
                        
                    #loop through i -> sub_target
                    #add more points in between
                    #find last one on obst
                    #comp length from goal
                    # if length too small increase i and continue

                
                
                
                
                sp = Position(self.current_start_state.x * self.scale, self.current_start_state.y * self.scale)
                ep = Position(g[0] * self.scale, g[1] * self.scale)
                self.startend_publisher.publish(Path([sp, ep]))
                
                
                s = self.current_start_state
                print s.x, s.y, s.theta1, s.theta2
                self.wait_for_map_update = False
                ri = max(self.i - 1, 0)
                self.pathplanner.setOptimalpath(self.refpath[self.i:self.i + sub_target])
                
                path = self.pathplanner.getPath(self.current_start_state, g, g2)
                
                
                
                
                lp = []
                for state in path:
                    lx = round(state.x * self.scale)
                    ly = round(state.y * self.scale)
                    lp.append(Position(lx, ly))
                #self.wait_for_map_update = False
                self.long_path_publisher.publish(lp)
                
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


                ti = int(ceil(len(path)/4.5))
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

                self.i += 10
                
            
            



if __name__ == '__main__':
    p = PathPlanningNode()
    p.spin()
