#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>   // 引用move_base的信息
#include <actionlib/client/simple_action_client.h>   // 引用actionlib库
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <signal.h>  // 引用signal头文件，为了做节点退出操作
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
// 定义一个SimpleActionClient，用来给move_base一个目标点：
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;



void DoShutdown(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown(); 
    exit(sig); //为了更完整，处理一下退出的signal
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "first_bot_goal"); //初始化ros节点的常规操作
    ros::NodeHandle nh;
    ros::NodeHandle nh_global;
	  ros::NodeHandle nh_local("~");

    tf2::Quaternion myQuaternion;
    
    
    // 声明一个SimpleActionClient：
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    double first_tb3_x_pos,first_tb3_y_pos,first_tb3_z_pos,first_tb3_yaw;
    // ros::param::get("init_first_tb3_x_pos", first_tb3_x_pos);
    // ros::param::get("init_first_tb3_x_pos", first_tb3_y_pos);
    // ros::param::get("init_first_tb3_x_pos", first_tb3_z_pos);
    // ros::param::get("init_first_tb3_x_pos", first_tb3_yaw);
    first_tb3_x_pos = nh_local.param("first_tb3_x_pos", 0.0);
    first_tb3_y_pos = nh_local.param("first_tb3_y_pos", 0.0);
    first_tb3_z_pos = nh_local.param("first_tb3_z_pos", 0.0);
    first_tb3_yaw = nh_local.param("first_tb3_yaw", 0.0);
    myQuaternion.setRPY( 0, 0, first_tb3_yaw );
    
    double x = myQuaternion.getX();
    double y = myQuaternion.getY();
    double z = myQuaternion.getZ();
    double w = myQuaternion.getW();
    cout << "111"<<first_tb3_x_pos<<endl;

  //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // 声明一个目标点goal，注意MoveBaseGoal的格式：
    move_base_msgs::MoveBaseGoal goal;
    
    signal(SIGINT, DoShutdown);
    
    ros::NodeHandle n;
    ros::Rate loop_rate(100); 
    
    while (ros::ok())
    {
        /*goal.target_pose.header.frame_id = "map" specifies the reference frame for that location. In this example, it is specified as the map frame, which simply means that the coordinates will be considered in the global reference frame related to the map itself. In other words, it is the absolute position on the map. In case where the reference frame is set with respect to the robot, namely goal.target_pose.header.frame_id = "base_link" (like in this tutorial), the coordinate will have a completely other meaning. In fact, in this case the coordinate will represent the (x,y) coordinate with respect to robot base frame attached to the robot, so it is a relative position rather than a absolute position as in the case of using the map frame.
        //goal.target_pose.header.frame_id = "base_link";*/
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
		// 以下是一个随意取的二维目标点：
        // goal.target_pose.pose.position.x = -8.0;
        goal.target_pose.pose.position.x = first_tb3_x_pos;
        goal.target_pose.pose.position.y = first_tb3_y_pos;
        goal.target_pose.pose.orientation.x = x;
        goal.target_pose.pose.orientation.y = y;
        goal.target_pose.pose.orientation.z = z;
        goal.target_pose.pose.orientation.w = w;

        ROS_INFO("Sending goal");
        // 定义好了goal，就可以调用SimpleActionClient的现成方法sendGoal()，非常方便：
        ac.sendGoal(goal);

        ac.waitForResult();
        // 判断是否执行成功：
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Hooray, the base moved to the goal");
            cout << "tb3_0 finished the goal!" <<endl;
            ros::shutdown();
          }
        else
          ROS_INFO("The base failed to move for some reason");
      
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

