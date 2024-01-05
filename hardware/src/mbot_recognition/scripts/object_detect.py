#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from math import *
from geometry_msgs.msg import Pose,PointStamped
from figure_analyze import get_mask
import tf2_ros
import tf2_geometry_msgs

HUE_LOW   = 0
HUE_HIGH  = 10
SATURATION_LOW  = 120
SATURATION_HIGH = 255
VALUE_LOW    = 70
VALUE_HIGH   = 255
class GETPOSE():

    def __init__(self) :
        self.pose = rospy.wait_for_message("odom", Odometry)
        self.robot_pose = np.array([self.pose.pose.pose.position.x,self.pose.pose.pose.position.y,self.pose.pose.pose.position.z])
class image_converter:
    def __init__(self):    
    # 创建cv_bridge，声明图像的发布者和订阅者
      self.bridge=CvBridge()	#ROS图像和OpenCV图像信息的转换
    #   self.image_sub=rospy.Subscriber("/camera/image_raw", Image, self.callback)	#订阅Image，Camera的话题
      self.image_sub=rospy.Subscriber("/camera/rgb/image_raw", Image, self.visual_callback)	#订阅Image，Camera的话题
      self.depth_sub=rospy.Subscriber("/camera/depth/image_raw",Image,self.depth_callback)    # 获取深度信息
      self.camera_info_sub = rospy.Subscriber("/camera/rgb/camera_info",CameraInfo,self.camera_info_callback)   # 获取相机内参
      self.image_pub=rospy.Publisher("object_detect_image", Image, queue_size=1)	#发布识别结果
      self.target_pub=rospy.Publisher("/object_detect_pose", PointStamped, queue_size=1)	#发布target的Pose信息
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
      import sys
      import os
      self.path=os.path.split(os.path.realpath(__file__))[0]
      self.last_depth=None

    def visual_callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        print("---------RECEIVED_RGB_MSG------------")
        try:
            image_input = self.bridge.imgmsg_to_cv2(data, "bgr8")	#将ROS中拿到的数据转换成OpenCV能够使用的数据
        except CvBridgeError as e:
            print(e)
        print("----------START_IMAGE_PROCESSING--------------")
        bgr_array=np.asarray(image_input)
        K1=-40
        K2=50
        C=0
	print("----------START_IMAGE_PROCESSING--------------")
        
	
	
	shape0=bgr_array.shape[0]
	shape1=bgr_array.shape[1]
	scale_factor=4
	scaled_bgr_array=bgr_array[0:shape0:scale_factor,0:shape1:scale_factor,:]
	rgb_array=np.zeros(scaled_bgr_array.shape)
	rgb_array[:,:,2]=scaled_bgr_array[:,:,0]
	rgb_array[:,:,1]=scaled_bgr_array[:,:,1]
	rgb_array[:,:,0]=scaled_bgr_array[:,:,2]
	mask= np.zeros(rgb_array.shape,np.int8)
	import time
	start_time=time.time()
	mask[:,:,0]=np.where(C + K1 * rgb_array[:,:,0] + K2 * rgb_array[:,:,1] > rgb_array[:,:,2],0,255)
	# condition=C + K1 * rgb_array[:,:,0] + K2 * rgb_array[:,:,1] < rgb_array[:,:,2]
	# mask[condition]=0
	# mask[condition]=255
	end_time=time.time() 
	print("SOLVE TIME:",end_time-start_time)
	# 计算条件，生成布尔数组
        # condition = C + K1 * rgb_array[:,:,0] + K2 * rgb_array[:,:,1] > rgb_array[:,:,2]
        # 根据条件设置 mask 的值
        #mask[condition] = 255
        #mask[~condition] = 0
        #mask=mask[:,:,0]
	mask=mask[:,:,0]
        mask = np.uint8(mask)
	print(mask)
        print("NEW MASK SHAPE",mask.shape)
        print("---------------GENERATE MASK---------------------")
        # mask = cv2.cvtColor(mask, cv2.COLOR_RGB2BGR)
        print("---------------GENERATE MASK---------------------")
	print("-------------NEW:",type(mask),"--------------------")
        cnts,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # loop over the contours
        for c in cnts:  # 每个c是一个轮廓的轮廓点组成的数组
            # compute the center of the contour
            M = cv2.moments(c)

            if int(M["m00"]) not in range(10, 307200):	#M["m00"]是面积
                continue
            
            cX = int(M["m10"] / M["m00"])*scale_factor   # M["m10"]是一阶矩，用来计算质心位置
            cY = int(M["m01"] / M["m00"])*scale_factor   # M["m10"]是一阶矩，用来计算质心位置
            print("C POINT TYPE:",type(c[0]))
            c=scale_factor*c
            # for c_ind in range(len(c)):
		# c[c_ind][0]=c[c_ind][0]*scale_factor
		# c[c_ind][1]=c[c_ind][1]*scale_factor
            # 在图像上显示识别出的物体轮廓和质心
            cv2.drawContours(image_input, [c], -1, (255, 68, 0), 2)
            cv2.circle(image_input, (cX, cY), 1, (0, 255, 230), -1)

            # 获取对应质心的深度信息
            print("depth_image_size:",self.depth_image.shape)
            range_x1=max(int(cX*400/480)-50,0)
            range_x2=min(int(cX*400/480)+50,400-1)
            range_y1=max(cY-50,0)
            range_y2=min(cY+50,640-1)
            range_array=self.depth_image[range_x1:range_x2,range_y1:range_y2]
            thre=50
            if len(range_array[range_array>50])==0:
            	if self.last_depth==None:
            		depth=10000
            	else:
            		depth=self.last_depth
            else:
            	depth=0.001*range_array[range_array>50].min()
            # depth = 0.01*self.depth_image[int(cX*400/480),cY]
            print("DEPTH:",depth)
            # rospy.loginfo("Robot detecting: get a target with depth "+str(depth)+"m")

            # 计算物体在相机坐标系中的位置,像素坐标系-->相机坐标系
            point_camera = PointStamped()
            point_camera.header.frame_id = data.header.frame_id
            point_camera.point.x = (cX - self.camera_info.K[2]) * depth / self.camera_info.K[0]
            point_camera.point.y = (cY - self.camera_info.K[5]) * depth / self.camera_info.K[4]
            point_camera.point.z = depth
            
            # 使用tf2将点从相机坐标系转换到世界坐标系
            point_world = self.tf_buffer.transform(point_camera,"odom")
            if np.isnan(point_world.point.x)!=True and np.isnan(point_world.point.y)!=True:
                np_image = np.asarray(image_input) #将cv_image转化为numpy数组
                # get current time
                import time
                time_now = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
                # save the numpy array into file
                # np.save(self.path+"/camera_record/camera"+str(time_now)+".npy",np_image)
            self.target_pub.publish(point_world)
	print("RECEIVED RGB DATA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # 显示Opencv格式的图像
        cv2.imshow("Image window", image_input)
        # cv2.imshow("Image window", mask)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        # try:
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        # except CvBridgeError as e:
            # print(e)

    def depth_callback(self,data):
        print("RECEIVED DEPTH DATA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        self.depth_image = self.bridge.imgmsg_to_cv2(data,"32FC1")  # "32FC1"表示32位浮点数单通道，常用于表示深度图像

    def camera_info_callback(self,data):
        self.camera_info = data



if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("object_detect")
        rospy.loginfo("Starting detect object")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down object_detect node.")
        cv2.destroyAllWindows()
