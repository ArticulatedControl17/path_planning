#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import Int64
from truck_map.msg import Map, Row
from truck_map.srv import GetMap
from map_func import readFileToMatrix


MAP_PATH = "/map.png"

PUBLISH_TOPIC = "map_updated"
MAP_SERVICE = "get_map"


class MapNode:

    def __init__(self):
        self.map = matrixToMap(readFileToMatrix(MAP_PATH))

        #self.pub = rospy.Publisher(PUBLISH_TOPIC, Int64, queue_size=10)
        serv1 = rospy.Service(MAP_SERVICE, GetMap, self.handleGetMap)

        rospy.loginfo("Waiting for request on service '%s'", MAP_SERVICE)


    def handleGetMap(self, req):
        rospy.loginfo("Returning map after request on service '%s'", MAP_SERVICE)
        return self.map


# Takes a matrix representing a Map
#
# Returns the map as Map.msg-object
def matrixToMap(matrix):
    rows = []

    # Converting all rows to Row.msg-objects
    for row in matrix:
        rows.append(Row(elem=row))

    return Map(rows=rows)


if __name__ == "__main__":
    rospy.init_node("map", anonymous=True)
    MapNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




# rosrun truck_map map_node.py
# rosservice call /get_map
