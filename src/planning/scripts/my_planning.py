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

from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.laserSub = rospy.Subscriber("/limo/scan", LaserScan, self.laserCallback)
        self.slam_cloud = None
        # self.pcPub = rospy.Publisher("/laserPointCloud", PointCloud2, queue_size=1)

    def laserCallback(self,msg):
        self.slam_cloud = msg
     
    def generate_pointcloud2(self):
        if self.slam_cloud == None:
            viewpoint_pointcloud2 = None
        else:
            cloud_out = self.laserProj.projectLaser(self.slam_cloud)
            viewpoint_pointcloud2 = cloud_out
        # self.pcPub.publish(cloud_out)
        return viewpoint_pointcloud2

def msg2np(msg: PointCloud2, fileds=('x', 'y', 'z', 'intensity')):
    """
    激光雷达不同, msg 字节编码不同
    Args:
        msg:
        fileds_names:
    Returns: np.array, Nx3 或者 Nx4

    """

    def find_filed(filed):
        # 顺序查找
        for f in msg.fields:
            if f.name == filed:
                return f

    data_types_size = [None,
                    {'name': 'int8', 'size': 1},
                    {'name': 'uint8', 'size': 1},
                    {'name': 'int16', 'size': 2},
                    {'name': 'uint16', 'size': 2},
                    {'name': 'int32', 'size': 4},
                    {'name': 'uint32', 'size': 4},
                    {'name': 'float32', 'size': 4},
                    {'name': 'float64', 'size': 8}]

    dtypes_list = [None, np.int8, np.uint8, np.int16, np.uint16,
                np.int32, np.uint32, np.float32, np.float64]  # PointCloud2 中有说明

    np_list = []
    for filed in fileds:
        f = find_filed(filed)

        dtype_size = data_types_size[f.datatype]['size']
        msg_total_type = msg.point_step

        item = np.frombuffer(msg.data, dtype=dtypes_list[f.datatype]).reshape(-1, int(
            msg_total_type / dtype_size))[:, int(f.offset / dtype_size)].astype(np.float32)
        np_list.append(item)

    points = np.array(np_list).T
    return points


if __name__ == '__main__':
    rospy.init_node("my_planning")
    rate = rospy.Rate(1)
    l2pc = Laser2PC()
    while not rospy.is_shutdown():
        # 将 laserscan格式 转换为pointcloud2格式
        pcd2 = l2pc.generate_pointcloud2()
        print(pcd2)
        # 将pointcloud2转换为 numpy点格式
        if pcd2 != None:
            point_set = msg2np(pcd2) #point_set为nunmpy类型
            print(point_set)
            # np.savetxt("data.txt",point_set,delimiter=',')


            #*********************************************************************************
            # you can use your method here 




            #*********************************************************************************


    rospy.spin()