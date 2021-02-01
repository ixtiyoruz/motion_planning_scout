#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point32
import ros_numpy
import std_msgs.msg
import numpy as np



#known pump geometry
class lidar_filter:
    def __init__(self):
        rospy.init_node('lidar_process')
        rospy.loginfo('lidar_process node started')
        self.sb = rospy.Subscriber("/velodyne_points", PointCloud2, self.process_points)
        self.pub_lidar = rospy.Publisher('lidar_points', PointCloud2, queue_size=0)
        
        self.x1 = -0.1
        self.x2 = 0.1
        self.z1 = 0
        self.z2 = 2
        self.y1= 0
        self.y2= 2
        self.received_msgs = []
        
        self.loop()

    def loop(self):
        r = rospy.Rate(10)
        while(True):
            if(len(self.received_msgs) > 0):
               
                msg = self.received_msgs[-1]
                array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
                pcfields = msg.fields 
                pcheader = msg.header
                # print(pcfields)
                new_data = self.filter_points(array)
                # print(len(new_data))
                new_pc2 = pc2.create_cloud(pcheader, pcfields, new_data)                
                self.pub_lidar.publish(new_pc2)
            r.sleep()
   
    def process_points(self, msg):
       self.received_msgs.append(msg)
       if(len(self.received_msgs) >=2):
           del self.received_msgs[:-1]
    def filter_points(self, array):
        res = []
        dt=np.dtype('float,float,float,float,int')
        array = np.array(array, dtype=dt)
        # print(array['f0'][0], '----')
        del_argsx1 = np.where(array['f0'] < self.x1)[0]
        del_argsx2 = np.where(array['f0'] > self.x2)[0]     
        del_argsy1 = np.where(array['f1'] < self.y1)[0]
        del_argsy2 = np.where( array['f1'] > self.y2)[0]       
        del_argsz1 = np.where(array['f2'] < self.z1 )[0]
        del_argsz2 = np.where(array['f2'] > self.z2)[0]
        all_del_args = np.append(del_argsx1, del_argsx2)
        all_del_args = np.append(all_del_args,del_argsy1)
        all_del_args = np.append(all_del_args,del_argsy2)
        all_del_args = np.append(all_del_args,del_argsz1)
        all_del_args = np.append(all_del_args,del_argsz2)

        # print(len(all_del_args), len(np.unique(all_del_args)))
        new_arr = np.delete(array, np.unique(all_del_args))
        # x = new_arr['f0']
        # y = np.round(new_arr['f2'] , 2)
        # intensity = new_arr['f3']

        # x_y_z = []

        # ys = np.unique(y)
        # for i in range(len(ys)):
        #     # find all unique y indexes
        #     y_ind = np.where(y == ys[i])[0]
            
        #     # find maximal intensity
        #     max_intensity = np.max(intensity[y_ind])
        #     # find maximal intensity index
        #     max_index_0 = np.where(intensity[y_ind] == max_intensity)[0]
            
        #     # take first found maximal index
        #     max_index_0 = max_index_0[0]
        #     max_y_ind = y_ind[max_index_0]
        #     # print(new_arr[max_index_0],'-------------')
        #     x_y_z.append(new_arr[max_y_ind])
        # print(x_y_z)

        #   // Create the filtering object: downsample the dataset using a leaf size of 1cm
        

        return new_arr

if __name__ == '__main__':
    try:
        l_p = lidar_filter()
    except rospy.ROSInterruptException:
        pass

