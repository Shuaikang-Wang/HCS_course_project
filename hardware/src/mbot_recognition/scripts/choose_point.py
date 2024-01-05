import rospy
from sensor_msgs.msg import LaserScan

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

import numpy as np
import random
import math
import copy
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

import os 
import sys
import time

RESCUE_THRESHOLD = 0.3
FRONTIER_THRESHOLD = 0.8


class GETPOSE():

    def __init__(self):
        self.pose = rospy.wait_for_message("odom", Odometry)
        self.robot_pose = np.array([self.pose.pose.pose.position.x,self.pose.pose.pose.position.y,self.pose.pose.pose.position.z])

class Choose_Point():
    def __init__(self):
        self.frontier_sub = rospy.Subscriber("/limo_explore/frontier_point",PointStamped, self.frontier_callback)
        self.rescue_sub = rospy.Subscriber("/object_detect_pose", PointStamped, self.rescue_callback)
        self.rescue_sub_msg = None
        self.frontier_sub_msg = None
	
	self.start_time = None
	
	self.last_rescue_id = None

        #self.flag = 1
        self.If_Arrived_Frontier=True
        self.last_frontier_goal_update_time=0
        self.rescue_list = []
        self.frontier_goal = []
        # object position dict:
        self.object_position={}
	self.last_obj = -1
        import sys
        self.path=sys.path[0]
        # clear the recording file
        f = open(self.path+"/object_positions_dict.txt","w")
        f.close()
        f = open(self.path+"/target_object_position.txt","w")
        f.close()
        f = open(self.path+"/frontier_points.txt","w")
        f.close()
        f = open(self.path+"/target_frontier_position.txt","w")
        f.close()


    def update_object_position(self,point_world):
        x,y,z= point_world.point.x,point_world.point.y,point_world.point.z

	if abs(x) >= 1000 or abs(y) >=1000 or abs(z) >= 500:
            return
	
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
        	if self.last_obj == None:
			self.last_obj = [x,y,z]
		else:
			if np.linalg.norm(np.array(self.last_obj)-np.array([x,y,z])) < 0.5:
				return 
                self.object_position[object_num+1]=[x,y,z]   
        import time
        # open a txt to record the position
        f = open(self.path+"/object_positions_dict.txt","a")
        f.write("--------------------------------------\n")
        f.write("Time:"+str(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
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
        # open a txt to record the position
        import time
        f = open(self.path+"/frontier_points.txt","a")
        f.write("--------------------------------------\n")
        f.write("Time:"+str(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
        f.write("\n")     
        #f.write(f"best_frontier_points: x: {point[0]},y: {point[1]},z: {point[2]},\n")
        # f.write(str(x)+","+str(y)+"\n")
        f.write("--------------------------------------\n")
        f.close()

        wrong_point = np.array([2.0,2.0])
        
        if np.linalg.norm(wrong_point-np.array(point[:2]))>0.1:
            import time
            # only update the frontier goal when the robot has arrived the previous frontier goal or the time is over 20s
            if self.If_Arrived_Frontier==True or time.time()-self.last_frontier_goal_update_time>20:
                self.frontier_goal = point
                self.If_Arrived_Frontier=False
                self.last_frontier_goal_update_time=time.time()
                f = open(self.path+"/target_frontier_position.txt","a")
                import time
                time_now=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                f.write("--------------------------------------\n")
                f.write("Time:"+str(time_now)+"\n")
                #f.write(f"goal frontier point: {point[0]},{point[1]}\n")
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

        robot = GETPOSE()
        # robot.robot_pose
        if len(self.object_position) != 0:
            min_key=min(list(self.object_position.keys()))
            rescue_position=self.object_position[min_key]
            distance_to_rescue = np.linalg.norm(np.array(robot.robot_pose)-np.array(rescue_position))
            if distance_to_rescue < RESCUE_THRESHOLD:
                del self.object_position[min_key]
                self.If_Arrived_Frontier = True
                
        if self.frontier_goal != []:
            distance_to_frontier = np.linalg.norm(np.array(robot.robot_pose)-np.array(self.frontier_goal[0]))
            if distance_to_frontier < FRONTIER_THRESHOLD:
                del self.frontier_goal[0]
                self.If_Arrived_Frontier = True
        
        if len(self.object_position) != 0:
            min_key=min(list(self.object_position.keys()))
            rescue_position=self.object_position[min_key]
            
            if min_key != self.last_rescue_id:
		import time
            	self.start_time = time.time()
            	self.last_rescue_id = min_key
            else:
            	import time
            	end_time = time.time()
		use_time = end_time - self.start_time
		if use_time >= 10:
                	del self.object_position[min_key]
                	self.If_Arrived_Frontier = True



            point = rescue_position
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

            client.wait_for_server()
          
            goal0 = MoveBaseGoal()
        
            goal0.target_pose.pose.position.x = point[0]
            goal0.target_pose.pose.position.y = point[1]
            goal0.target_pose.pose.orientation.z = 0.15
            goal0.target_pose.pose.orientation.w = 1

            goal0.target_pose.header.frame_id = "map"
            goal0.target_pose.header.stamp = rospy.Time.now()
            import time
            # open a txt to record the position
            f = open(self.path+"/target_object_position.txt","a")
            f.write("--------------------------------------\n")
            f.write("Time:"+str(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
            f.write("\n")     
            #f.write(f"target object position: x: {goal0.target_pose.pose.position.x} y: {goal0.target_pose.pose.position.y} \n")
            f.write("--------------------------------------\n")
            f.close()

            client.send_goal(goal0)
            print("--------------------RESCUEING----------------------")

        elif self.frontier_goal != []:
            point = self.frontier_goal
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

            client.wait_for_server()
          
            goal0 = MoveBaseGoal()
         
            goal0.target_pose.pose.position.x = point[0]
            goal0.target_pose.pose.position.y = point[1]
            goal0.target_pose.pose.orientation.z = 0.15
            goal0.target_pose.pose.orientation.w = 1
            import time
            goal0.target_pose.header.frame_id = "map"
            goal0.target_pose.header.stamp = rospy.Time.now()
            
            client.send_goal(goal0)
            print("--------------------EXPLORING----------------------")

if __name__ == "__main__":
    rospy.init_node("decision_planner")
    rospy.loginfo("Starting planning")
    my_plan = Choose_Point()
    while not rospy.is_shutdown():
        my_plan.plan()
    rospy.spin()

            









