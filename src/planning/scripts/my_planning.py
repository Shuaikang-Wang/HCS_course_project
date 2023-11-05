import rospy
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
print(f"Entering vie!!!!!!!!!!!!!!!!!!!!!!1")
import sys

import numpy as np
import random
import math
import copy
import open3d as o3d
import time
import rospy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointField

import pcl
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.laserSub = rospy.Subscriber("/limo/scan", LaserScan, self.laserCallback)
        self.my_pointcloud2 = None
        # self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1) 
    def laserCallback(self,data):

        cloud_out = self.laserProj.projectLaser(data)
        self.my_pointcloud2 = cloud_out
        # self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("my_planning")
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        l2pc = Laser2PC()
        print(l2pc.my_pointcloud2)
    
    rospy.spin()