import rospy
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
print(f"Entering choose point!!!!!!!!!!!!!!!!!!!!!!1")
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


class Choose_Point():
    def __init__(self):
        self.frontier_sub = rospy.Subscriber("")


