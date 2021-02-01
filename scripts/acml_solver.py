#!/usr/bin/env python
import rospy
import ros_numpy
import std_msgs.msg
import numpy as np
import rospkg
import subprocess
import os
# we have to save it like :: rosrun map_server map_saver -f map
#known pump geometry
class MapSaver:
    def __init__(self):
        rospy.init_node('amcl_solver_node')
        rospy.loginfo('amcl_solver_node node started')
        self.rospack = rospkg.RosPack()
        self.sb_map = rospy.Subscriber("/map", OccupancyGrid, self.process_map)
        # self.sb_map_meta = rospy.Subscriber("/map_metadata", MapMetaData, self.process_map_meta)
        rospy.spin()

    def process_map(self, message):        
        print("map received", type(message))
        
        # os.system("rosrun map_server map_saver -f " + self.save_folder)
        test = subprocess.Popen("rosrun map_server map_saver -f " + self.save_folder, stdout=subprocess.PIPE, shell=True)
        output = test.communicate()[0]
        # print(message.header)
        # print(message.info)
        # print(len(message.data))

    # def process_map_meta(self, message):
    #     print("map_meta received", type(message))
        
if __name__ =='__main__':
    map_saver = MapSaver()
