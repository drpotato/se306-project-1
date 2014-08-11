#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include "R0.h"

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

 void R0::initialSetup(int argc, char **argv) {
  //initialize robot parameters
  //Initial pose. This is same as the pose that you used in the world file to set the robot pose.
  theta = M_PI/2.0;
  px = 10;
  py = 20;
  
  //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
  ros::init(argc, argv, "RobotNode0");
  
  //NodeHandle is the main access point to communicate with ros.
  ros::NodeHandle n;
}

void R0::executeInfiniteLoopHook() {
  
}
