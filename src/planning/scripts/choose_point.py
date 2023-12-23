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
from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry
from actionlib.action_client import GoalManager
import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

RESCUE_THRESHOLD = 0.3
FRONTIER_THRESHOLD = 0.2


class GETPOSE():

    def __init__(self) -> None:
        self.pose = rospy.wait_for_message("odom", Odometry)
        self.robot_pose = np.array([self.pose.pose.pose.position.x,self.pose.pose.pose.position.y,self.pose.pose.pose.position.z])

class Choose_Point():
    def __init__(self):
        self.frontier_sub = rospy.Subscriber("/frontier_point",PointStamped, self.frontier_callback)
        self.rescue_sub = rospy.Subscriber("/object_detect_pose", PointStamped, self.rescue_callback)
        self.rescue_sub_msg = None
        self.frontier_sub_msg = None

        self.flag = 1

        self.rescue_list = []
        self.frontier_goal = []
        # object position dict:
        self.object_position={}

    def update_object_position(self,point_world):
        x,y,z= point_world.point.x,point_world.point.y,point_world.point.z
        # get the path of current sricpts
        import os
        path=os.path.abspath(os.path.dirname(__file__))
        # open a txt to record the position
        print("-------------------------------------------------")
        print(path+"/object_position.txt")
        print("-------------------------------------------------")
        f = open(path+"/object_position.txt","a")
        Is_Recorded_Before=False
        # check if the object has been recorded
        for obj_key in self.object_position.keys():
            old_position=self.object_position[obj_key]
            if abs(x-old_position[0])<0.2 and abs(y-old_position[1])<0.2:
                new_position=np.array(old_position)*0.8+np.array([x,y,z])*0.2
                new_position=new_position.tolist()
                self.object_position[obj_key]=new_position
                Is_Recorded_Before=True
                break
        if Is_Recorded_Before==False:
            if np.isnan(x)==False and np.isnan(y)==False:
                object_num=len(list(self.object_position.keys()))
                self.object_position[object_num+1]=[x,y,z]   
        # get current time
        import time
        time_now=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        f.write("--------------------------------------\n")
        f.write("Time:"+str(time_now))
        f.write("\n")     
        for obj_key in self.object_position.keys():
            x,y,z=self.object_position[obj_key]
            f.write("object index:"+str(obj_key)+":\t x:"+str(x)+"\t y:"+str(y)+"\t z:"+str(z)+"\n")
        # f.write(str(x)+","+str(y)+"\n")
        f.write("--------------------------------------\n")
        f.close()

    def frontier_callback(self,msg):
        self.frontier_sub_msg = msg
        point =[self.frontier_sub_msg.point.x,self.frontier_sub_msg.point.x,self.frontier_sub_msg.point.z]
        wrong_point = np.array([2.0,2.0])
        if np.linalg.norm(wrong_point-np.array(point[:2]))>0.1:
            self.frontier_goal = point
        import os
        import sys
        path=os.path.abspath(os.path.dirname(__file__))
        # path=sys.path[0]
        # open a txt to record the position
        f = open(path+"/best_frontier_points.txt","w")
        # get current time
        import time
        time_now=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        f.write("--------------------------------------\n")
        f.write("Time:"+str(time_now))
        f.write("\n")     
        f.write(f"best_frontier_points: x: {point[0]},y: {point[1]},z: {point[2]},\n")
        # f.write(str(x)+","+str(y)+"\n")
        f.write("--------------------------------------\n")
        f.close()
     

    def rescue_callback(self, msg):
        self.rescue_sub_msg = msg
        self.update_object_position(self.rescue_sub_msg)
        # point =[self.rescue_sub_msg.point.x,self.rescue_sub_msg.point.x,self.rescue_sub_msg.point.z]
        # self.rescue_list.append(point)


    def plan(self):
        '''
        Function: choose which point to go 
        '''
        rescue_position = [0,0,0]

        #判断位姿 与 rescue_point 和frontier_point的距离，决定是否删除rescue_point点以及，选用下一个
        if len(self.object_position) !=0:
            min_key=min(list(self.object_position.keys()))
            rescue_position=self.object_position[min_key]

        robot = GETPOSE()
        # robot.robot_pose
        if len(self.object_position) != 0:
            distance_to_rescue = np.linalg.norm(np.array(robot.robot_pose)-np.array(rescue_position))
            if distance_to_rescue < RESCUE_THRESHOLD:
                del self.object_position[min_key]
                self.flag = 1
                
        if self.frontier_goal != []:
            distance_to_frontier = np.linalg.norm(np.array(robot.robot_pose)-np.array(self.frontier_goal[0]))
            if distance_to_frontier < FRONTIER_THRESHOLD:
                del self.frontier_goal[0]
                self.flag = 1
        
        if len(self.object_position) != 0:
            min_key=min(list(self.object_position.keys()))
            rescue_position=self.object_position[min_key]
            #若 救援点 非空，则优先选择救援点作为目标点发布
            point = rescue_position
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

            client.wait_for_server()
            #定义四个发送目标点的对象
            goal0 = MoveBaseGoal()
            # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
            goal0.target_pose.pose.position.x = point[0]
            goal0.target_pose.pose.position.y = point[1]
            # goal0.target_pose.pose.orientation.z = 0
            # goal0.target_pose.pose.orientation.w = 1

            goal0.target_pose.header.frame_id = "map"
            goal0.target_pose.header.stamp = rospy.Time.now()
            import os
            import sys
            #path=os.path.abspath(os.path.dirname(__file__))
            path=sys.path[0]
            # open a txt to record the position
            f = open(path+"/object_position.txt","w")
            # get current time
            import time
            time_now=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            f.write("--------------------------------------\n")
            f.write("Time:"+str(time_now))
            f.write("\n")     
            f.write(f"target object position: x: {goal0.target_pose.pose.position.x} y: {goal0.target_pose.pose.position.y} \n")
            # f.write(str(x)+","+str(y)+"\n")
            f.write("--------------------------------------\n")
            f.close()
            client.send_goal(goal0)
            print("--------------------RESCUEING----------------------")
        elif self.frontier_goal != []:
            point = self.frontier_goal
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

            client.wait_for_server()
            #定义四个发送目标点的对象
            goal0 = MoveBaseGoal()
            # 初始化四个目标点在 map 坐标系下的坐标,数据来源于《采集的目标点.docx》
            goal0.target_pose.pose.position.x = point[0]
            goal0.target_pose.pose.position.y = point[1]
            # goal0.target_pose.pose.orientation.z = 0
            goal0.target_pose.pose.orientation.w = 1

            goal0.target_pose.header.frame_id = "map"
            goal0.target_pose.header.stamp = rospy.Time.now()
            if self.flag == 1:
                client.send_goal(goal0)
                import os
                path=os.path.abspath(os.path.dirname(__file__))
                # open a txt to record the position
                f = open(path+"/frontier_position.txt","a")
                import time
                time_now=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                f.write("--------------------------------------\n")
                f.write("Time:"+str(time_now))
                f.write("\n")     
                f.write(f"goal frontier point: {goal0.target_pose.pose.position.x},{goal0.target_pose.pose.position.y}\n")
                # f.write(str(x)+","+str(y)+"\n")
                f.write("--------------------------------------\n")
                f.close()
                self.flag = 0
            print("--------------------EXPLORING----------------------")

if __name__ == "__main__":
    rospy.init_node("decision_planner")
    rospy.loginfo("Starting planning")
    my_plan = Choose_Point()
    while not rospy.is_shutdown():
        my_plan.plan()
    rospy.spin()

            









