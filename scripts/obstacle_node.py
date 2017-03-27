#!/usr/bin/env python
# license removed for brevity
from map_func import *

import rospy
from std_msgs.msg import Int8
from os.path import dirname, abspath
from PIL import Image


PUBLISH_TOPIC = "map_updated"
IMG_PATH = "/map.png"

SCALE = 10  # Map img is in scale 1:10


class ObstacleHandler:

    def __init__(self, map_obj):
        dirpath = dirname(abspath(__file__))

        self.map_img = Image.open(dirpath + IMG_PATH)
        self.map = map_obj

        # To handle user input
        self.ax = None

        # Ros topic
        self.pub = rospy.Publisher(PUBLISH_TOPIC, Int8, queue_size=10)
        rospy.loginfo("Publishing on topic '%s'", PUBLISH_TOPIC)


    # Handler for 'key_press_event'
    def onKeyPress(self, event):
        if event.key.isdigit():
            if event.key == "0":
                index = 9
            else:
                index = int(event.key)-1

            # Checking if there is an obstacle in 'obstacles' that corresponds with given number,
            # only proceeding if there is
            try:
                obstacle = self.map.obstacles[index]
            except IndexError:
                print "There is no obstacle with identifier '%s'" % (index+1)
                return

            # If the obstacle is active: Deactivating it
            if obstacle.active:
                # Removing obstacle from the map matrix
                self.map.removeObstacle(index)
                print "Obstacle '%s' was deactivated" % (index+1)
                self.pub.publish(index)

                # Updating obstacle plot
                obstacle.plot.remove()
                obstacle.text.remove()
                obstacle.plot = self.ax.add_patch(obstacle.deactivated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(index+1),
                                             verticalalignment="top", color="blue", fontweight="bold")
                plt.draw()

            # If the obstacle is inactive: Activating it
            else:
                # Adding obstacle to the map matrix
                self.map.addObstacle(index)
                print "Obstacle '%s' was activated" % (index+1)
                self.pub.publish(index)
                #rospy.loginfo("Published on topic '%s'", PUBLISH_TOPIC)

                # Updating obstacle plot
                obstacle.plot.remove()
                obstacle.text.remove()
                obstacle.plot = self.ax.add_patch(obstacle.activated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(index+1),
                                             verticalalignment="top", color="red", fontweight="bold")
                plt.draw()


    # Lets user activate/deactivate obstacles on the 'map' of this handler
    def handleObstacles(self):
        xlim = self.map_img.size[0]
        ylim = self.map_img.size[1]

        fig = plt.figure()
        self.ax = plt.axes()

        # Graph settings
        plt.axis("scaled")
        plt.xlim( (0, xlim) )
        plt.ylim( (ylim, 0) )
        plt.xlabel("x-axis")
        plt.ylabel("y-axis")
        plt.title("handleObstacles()")

        # Displaying map image
        img_plot = plt.imshow(self.map_img)
        # Displaying all obstacles
        for i, obstacle in enumerate(self.map.obstacles):
            if obstacle.active:
                if obstacle.plot:
                    obstacle.plot.remove()
                obstacle.plot = self.ax.add_patch(obstacle.activated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(i+1),
                                             verticalalignment="top", color="red", fontweight="bold")
            else:
                if obstacle.plot:
                    obstacle.plot.remove()
                obstacle.plot = self.ax.add_patch(obstacle.deactivated_patch)
                obstacle.text = self.ax.text(obstacle.text_x, obstacle.text_y, str(i+1),
                                             verticalalignment="top", color="blue", fontweight="bold")

        print ("=====\nRed obstacles are activated, Blue obstacles are deactivated\n" +
               "Press the corresponding number key, to activate/deactivate an obstacle\n=====")

        fig.canvas.mpl_connect("key_press_event", self.onKeyPress)

        plt.show()


if __name__ == "__main__":
    rospy.init_node("obstacles", anonymous=True)
    map_obj = Map()
    handler_obj = ObstacleHandler(map_obj)

    try:
        handler_obj.handleObstacles()
    except rospy.ROSInterruptException:
        pass


# rosrun truck_map obstacle_node.py
# rostopic echo map_updated
