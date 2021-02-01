#! /usr/bin/env python
import tensorflow as tf
import numpy
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)


