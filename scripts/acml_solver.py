#!/usr/bin/env python
import rospy
import ros_numpy
import std_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rospkg
import subprocess
import os
from  std_srvs.srv import Empty
# we have to save it like :: rosrun map_server map_saver -f map
#known pump geometry
class LocalizationSolver:
    def __init__(self):
        rospy.init_node('amcl_solver_node')
        rospy.loginfo('amcl_solver_node node started')
        self.rospack = rospkg.RosPack()
        self.sb_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.process_pose)
        self.serive_global_localization = rospy.ServiceProxy('/global_localization', Empty)
        rospy.spin()

    def process_pose(self, message):        
        # print("pose received", type(message.pose.covariance))
        cov = np.array(message.pose.covariance)
        
        # if(np.any(cov> 0.65)):
        #     # invoke global_localization, with current covariance abd std
        #     self.serive_global_localization()
        #     print(np.max(cov))
        #     pass


if __name__ =='__main__':
    map_saver = LocalizationSolver()
