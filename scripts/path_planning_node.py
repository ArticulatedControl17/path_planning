#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from custom_msgs.msg import *
from custom_msgs.srv import *
import time
import os
from math import *
import sys
from vehicleState import *
from pathPlanning import *
from Point import *
from helper_functions import *

from geometry_msgs.msg import PoseWithCovarianceStamped

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *

import ref_path


PATH_LENGTH_INDEX = 45
PATH_CUTOFF_RATIO = 2.5
OBSTACLE_BACKTRACK_INDEX_DISTANCE = 11
OBSTACLE_LOOKAHEAD_EULER_DISTANCE = 160.0
REFPATH_POINT_DENSITY = 20.0

FEISABLE_MAX_TIME = 4
FEISABLE_MOD_POINT = 6.0
FEISABLE_MOD_THETA = 0.5

FIRST_MOD_POINT = 6.0
FIRST_MOD_THETA = 0.5

MOD_POINT = 3.0
MOD_THETA = 0.3

RPI_FEISABLE_MAX_TIME = 8
RPI_FEISABLE_MOD_POINT = 9.0
RPI_FEISABLE_MOD_THETA = 0.6

RPI_FIRST_MOD_POINT = 8.0
RPI_FIRST_MOD_THETA = 0.6

RPI_MOD_POINT = 8.0
RPI_MOD_THETA = 0.6


MAX_TIME = 15



class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning', anonymous=False)

        
        if rospy.get_param('path_planning/rpi', False):
            print "Using RPI params"
            FEISABLE_MAX_TIME = RPI_FEISABLE_MAX_TIME
            FEISABLE_MAX_TIME = RPI_FEISABLE_MAX_TIME
            FEISABLE_MOD_POINT = RPI_FEISABLE_MOD_POINT
            FEISABLE_MOD_THETA = RPI_FEISABLE_MOD_THETA

            FIRST_MOD_POINT = RPI_FIRST_MOD_POINT
            FIRST_MOD_THETA = RPI_FIRST_MOD_THETA

            MOD_POINT = RPI_MOD_POINT
            MOD_THETA = RPI_MOD_THETA
        
        
        self.i = 0
        self.done = False

        self.goals = []
        self.gi = []


        self.first = False

        self.tp = []

        self.map_obj = Map()
        self.ref_obj = ref_path.RefPath()

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
        self.trailer_path_publisher = rospy.Publisher('trailer_path', Path, queue_size=10)


        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initPoseCallback)

        rospy.Subscriber('map_updated', Int8, self.mapUpdateHandler)
        rospy.Subscriber('truck_state', TruckState, self.truckStateHandler)

        self.path_request_srv = rospy.Service('request_path', RequestPath, self.requestPathHandler)


    def initPoseCallback(self, data):
        self.current_path = self.current_path[:1]

    def isVehicleStateOK(self, state):
        return self.pathplanner.checkIfInTrack(state)

    def isCurrentPlanOK(self):
        
        for i in range(len(self.current_path)-1):
            this_state = self.current_path[i]
            next_state = self.current_path[i+1]
            avg_x = (this_state.x + next_state.x) / 2
            avg_y = (this_state.y + next_state.y) / 2
            avg_t1 = (this_state.theta1 + next_state.theta1)/2
            avg_t2 = (this_state.theta2 + next_state.theta2)/2
            avg_state = VehicleState(avg_x, avg_y, avg_t1, avg_t2)
            
            if (not self.isVehicleStateOK(this_state)) or (not self.isVehicleStateOK(avg_state)) or (not self.isVehicleStateOK(next_state)):
                return (False,i)

        return (True,0)

    def updateMap(self, obst):
        print "update map", obst
        add = self.map_obj.addObstacle(obst)
        if not add:
            rem = self.map_obj.removeObstacle(obst)
            if not rem:
                print "can't add or remove obstacle"

        self.map, _ = self.map_obj.getMapAndScale()
        if add:
            return True
        else:
            return False


    def mapUpdateHandler(self, data):

        
        self.wait_for_map_update = True

        added = self.updateMap(data.data)
        self.pathplanner.setMap(self.map)

        if added:
            print "added obst"

            (ok, i) = self.isCurrentPlanOK()

            if ok:
                print "cur path ok"


            else:
                print "cur path NOT ok"
                if i-OBSTACLE_BACKTRACK_INDEX_DISTANCE < 0:
                    self.current_start_state = self.latest_state
                    self.current_path = []
                    self.tp = []
                else:
                    self.current_start_state = self.current_path[i-OBSTACLE_BACKTRACK_INDEX_DISTANCE]
                    self.current_path = self.current_path[:i-OBSTACLE_BACKTRACK_INDEX_DISTANCE+1]
                    self.tp = self.tp[:i-OBSTACLE_BACKTRACK_INDEX_DISTANCE+1]

                
                self.publishReworkPath(self.current_path)
                
                self.done = False
                self.i = getClosestIndex(self.refpath, (self.current_start_state.x, self.current_start_state.y))


                self.active = True

            

        else:
            if (not self.active) and (not self.done):

                self.path_rework_publisher.publish(Path([]))
                s = self.current_start_state = self.latest_state
                self.i = 0
                self.tp = []

                self.current_path = [VehicleState(s.x, s.y, s.theta1, s.theta2)]



                start = ref_path.VehicleState(s.x, s.y, s.theta1, s.theta2)


                ci = getClosestIndex(self.refpath, (s.x, s.y))

                x = len(filter(lambda f: f > ci, self.gi))

                rp, self.gi = self.ref_obj.getRefPath(start, self.goals[-x:])
                if rp == [] or rp == None:
                    print "Can't find a path"
                    self.active = False
                    return

                self.refpath = rp
                
                self.publishRefPath(self.refpath)
                self.active = True
                self.first = True

    
    def traverseCurrentPath(self, p, path):
        path = list(path)


        if len(path) < 2:
            return path

        l1 = path.pop(0)
        l2 = path.pop(0)


        while hasPassedLine(Point(*p), (Point(l1.x, l1.y), Point(l2.x, l2.y))):


            if len(path) == 0:
                return [l2]


            l1 = l2
            l2 = path.pop(0)

        return [l1, l2] + path




    def truckStateHandler(self, data):
        self.latest_state = VehicleState(data.p.x / self.scale, data.p.y / self.scale, data.theta1, data.theta2)
    
        self.current_path = self.traverseCurrentPath(self.latest_state.getPoint(), self.current_path)


    def requestPathHandler(self, data):

        self.tp = []
        s = data.state

        start = ref_path.VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)
        self.goals = [(float(p.x)/self.scale, float(p.y)/self.scale) for p in data.goals.path]

        response = RequestPathResponse()

        rp, self.gi = self.ref_obj.getRefPath(start, self.goals)
        if rp == []:
            response.success = False
            response.message = "Couldn't find reference path. Possible cause: start or goal way off"

        else:
            response.success = True
            self.done = False
            self.i = 0

            self.first = True

            self.current_start_state = VehicleState(s.p.x / self.scale, s.p.y / self.scale, s.theta1, s.theta2)

            self.refpath = rp

            self.publishRefPath(self.refpath)

            self.wait_for_map_update = False

            self.current_path = [self.current_start_state.copy()]
            self.active = True


        return response





    def spin(self):
        next_subadd = 0
        while not rospy.is_shutdown():

            if not self.active:
                time.sleep(0.05)
            else:

                sub_target = PATH_LENGTH_INDEX + next_subadd

                trying_done = False
                if self.i + sub_target>= len(self.refpath) - 1:
                    g2, g = self.refpath[-2:]
                    trying_done = True
                else:
                    nnn = self.i + sub_target
                    g, g2 = self.refpath[nnn], self.refpath[nnn-1]

                    
                    latest = None
                    for j in range(self.i, self.i + sub_target+1):
                        
                        p1,p2 = self.refpath[j], self.refpath[j+1]
                        pts = getPointsInBetween(p1,p2,6)
                        for x,y in pts:
                            if self.map[y][x] in [0]:
                                latest = (x,y)


                    if latest != None:
                        lfg = getDistance(g, latest)
                        if lfg <= OBSTACLE_LOOKAHEAD_EULER_DISTANCE:
                            added_space = int(round(-1/REFPATH_POINT_DENSITY * lfg + OBSTACLE_LOOKAHEAD_EULER_DISTANCE / REFPATH_POINT_DENSITY))
                            
                            next_subadd += max(added_space, 1)
                            continue


                next_subadd = 0

                self.publishStartEnd(self.current_start_state.getPoint(), g)


                self.wait_for_map_update = False
                self.pathplanner.setOptimalpath(self.refpath[self.i:self.i + sub_target])

                
                # ------------- GETTING THE PATH ---------------------
                path = self.getPath(self.current_start_state, g, g2, self.first)
                
                self.first = False



                if self.wait_for_map_update: #map updated while planning
                    continue

                if path == []:
                    ind = []
                    firsti = None
                    lasti = None

                    for c, gl in enumerate(self.gi):
                        if gl > self.i and gl < self.i + sub_target:
                            if firsti == None:
                                firsti = c
                            lasti = c

                    if firsti == None:
                        for i in range(len(self.gi)-1):
                            g1 = self.gi[i]
                            g_2 = self.gi[i+1]
                            if self.i >= g1 and self.i <= g_2:
                                (firsti, lasti) = (i, i+1)
                                break


                    else:
                        
                        if firsti != 0:
                            firsti -= 1

                        if lasti != len(self.gi) -1:
                            lasti += 1

                    alt_path_index = 1
                    while not rospy.is_shutdown():
                        ap = False
                        sol = False
                        for i in range(firsti, lasti)[::-1]:

                            starti = self.gi[i]
                            stopi = self.gi[i+1]

                            newref = self.ref_obj.getAltPath(self.refpath, starti, stopi, alt_path_index)
                            if newref == [] or newref == None :
                                continue

                            ap = True

                            self.publishRefPath(newref)


                            diff = len(newref) - len(self.refpath)
                            
                            newOpt = newref[self.i:stopi + diff+1]
                            if len(newOpt) < 2:
                                continue
                            
                            self.pathplanner.setOptimalpath(newOpt)

                            goal_i = stopi + diff
                            goal = newref[goal_i]
                            goal_2 = newref[goal_i -1]
                            
                            self.publishStartEnd(self.current_start_state.getPoint(), goal)
                            
                            self.wait_for_map_update = False
                            while 1:
                                newpath = self.checkIfPathFeisable(self.current_start_state, goal, goal_2)
                                if not self.wait_for_map_update:
                                    break
                            
                            if newpath:
                                sol = True

                                for k in range(i+1, len(self.gi)):
                                    self.gi[k] += diff

                                self.refpath = newref
                                break


                        if sol:
                            break
                        if not ap:
                            print "cant find a path"
                            self.active = False
                            break

                        alt_path_index += 1


                    if self.active == False:
                        continue
                    if sol:
                        continue





                if trying_done:
                    self.done = True
                    ti = len(path)-1
                else:
                    ti = min(len(path) -1, int(ceil(len(path)/PATH_CUTOFF_RATIO))+1)

                self.current_start_state = path[ti]
                app_path = path[:ti+1]
                self.current_path += app_path


                self.publishLongPath(path)


                self.publishAppendPath(app_path, self.done)
                self.publishTrailerPath(app_path)
                
                
                if self.done:
                    self.active = False
                    continue

                self.i += int(round(PATH_LENGTH_INDEX/PATH_CUTOFF_RATIO))


    def getPath(self, startstate, goal_1, goal_2, first):
        if self.checkIfPathFeisable(startstate, goal_1, goal_2):
            if first:
                mp, mt = FIRST_MOD_POINT, FIRST_MOD_THETA
            else:
                mp, mt = MOD_POINT, MOD_THETA
            
            return self.pathplanner.getPath(self.current_start_state, goal_1, goal_2, MAX_TIME, mp, mt)
        else:
            return []
    
    def checkIfPathFeisable(self, startstate, goal_1, goal_2):
        return self.pathplanner.getPath(startstate, goal_1, goal_2, FEISABLE_MAX_TIME, FEISABLE_MOD_POINT, FEISABLE_MOD_THETA, returnsIfFeisable=True)
    
    def publishAppendPath(self, statelist, done):
        p = []
        for state in statelist:
            xx = round(state.x * self.scale)
            yy = round(state.y * self.scale)

            p.append(Position(xx,yy))

        if done:
            p.append(Position(-1, -1))

        self.path_append_publisher.publish(Path(p))
        
    def publishLongPath(self, statelist):
        
        lp = []
        for state in statelist:
            lx = round(state.x * self.scale)
            ly = round(state.y * self.scale)
            lp.append(Position(lx, ly))
            
        self.long_path_publisher.publish(lp)

    def publishRefPath(self, path):
        p = [Position(x*self.scale,y*self.scale) for x,y in path]
        self.refpath_publisher.publish(Path(p))
    
    def publishTrailerPath(self, statelist):
        for state in statelist:
                    
            tx = state.x - HEADER_FRONTAXIS_TO_JOINT * cos(state.theta1) - (TRAILER_LENGTH + TL_BACK) * cos(state.theta2)
            ty = state.y - HEADER_FRONTAXIS_TO_JOINT * sin(state.theta1) - (TRAILER_LENGTH + TL_BACK) * sin(state.theta2)
            self.tp.append(Position(round(tx * self.scale),round(ty * self.scale)))

            
        self.trailer_path_publisher.publish(Path(self.tp))

    def publishStartEnd(self, p1, p2):
        sp = Position(p1[0] * self.scale, p1[1] * self.scale)
        ep = Position(p2[0] * self.scale, p2[1] * self.scale)
        self.startend_publisher.publish(Path([sp, ep]))
    
    def publishReworkPath(self, statelist):
        p = []
        for state in statelist:
            xx = round(state.x * self.scale)
            yy = round(state.y * self.scale)
            p.append(Position(xx,yy))
        self.path_rework_publisher.publish(Path(p))



if __name__ == '__main__':
    p = PathPlanningNode()
    p.spin()
