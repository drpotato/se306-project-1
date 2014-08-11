#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageOdom_callback(nav_msgs::Odometry msg)
{
    //This is the call back function to process odometry messages coming from Stage.
    px = 5 + msg.pose.pose.position.x;
    py =10 + msg.pose.pose.position.y;
    ROS_INFO("Current x position is: %f", px);
    ROS_INFO("Current y position is: %f", py);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
    //This is the callback function to process laser scan messages
    //you can access the range data from msg.ranges[i]. i = sample number
    
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char **argv)
{
    
    //initialize robot parameters
    //Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
    theta = M_PI/2.0;
    px = 10;
    py = 20;
    
    //Initial velocity
    linear_x = 0.2;
    angular_z = 0.2;
    
    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
    ros::init(argc, argv, "RobotNode0");
    
    //NodeHandle is the main access point to communicate with ros.
    ros::NodeHandle n;
    
    //advertise() function will tell ROS that you want to publish on a given topic_
    //to stage
    ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
    
    //subscribe to listen to messages coming from stage
    ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
    ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);
    
    ros::Rate loop_rate(10);
    
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
            ros::Time::now(),"robot_0/base_link", "robo_0/base_laser"));

    
    
    //a count of howmany messages we have sent
    int count = 0;
    
    ////messages
    //velocity of this RobotNode
    geometry_msgs::Twist RobotNode_cmdvel;
    
    
     
    
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("robot_0/move_base", true);
    
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    
    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "robot_0/base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    
    while (ros::ok())
    {
        
        //messages to stage
        //RobotNode_cmdvel.linear.x = linear_x;
        //RobotNode_cmdvel.angular.z = angular_z;
        
        //publish the message
        RobotNode_stage_pub.publish(RobotNode_cmdvel);
        
        ros::spinOnce();
        
        loop_rate.sleep();
        ++count;
    }
    
    return 0;
    
}