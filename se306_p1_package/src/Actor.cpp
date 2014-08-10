#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#ifndef ACTORMAIN
#define ACTORMAIN

class Actor
{

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

	virtual int mainHook(int argc, char **argv){}

	int main(int argc, char **argv)
	{
    	//TODO: Decide whether the following code is needed in the Actor class, or can be implemented in the mainHook method by other classes


	    //initialize robot parameters
	    //Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	    //theta = M_PI/2.0;
	    //px = 10;
	    //py = 20;
	    
	    //Initial velocity
	    linear_x = 0;
	    angular_z = 0;
	    
	    //You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	    //ros::init(argc, argv, "RobotNode0");
	    
	    //NodeHandle is the main access point to communicate with ros.
	    //ros::NodeHandle n;
	    
	    //advertise() function will tell ROS that you want to publish on a given topic_
	    //to stage
	    //ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
	    
	    //subscribe to listen to messages coming from stage
	    //ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
	    //ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);
	    
	    //ros::Rate loop_rate(10);
	    
	    //a count of howmany messages we have sent
	    //int count = 0;
	    
	    ////messages
	    //velocity of this RobotNode
	    //geometry_msgs::Twist RobotNode_cmdvel;
	    
	    //while (ros::ok())
	    //{
	        //messages to stage
	        //RobotNode_cmdvel.linear.x = linear_x;
	        //RobotNode_cmdvel.angular.z = angular_z;
	        
	        //publish the message
	        //RobotNode_stage_pub.publish(RobotNode_cmdvel);
	        
	        //ros::spinOnce();
	        
	        //loop_rate.sleep();
	        //++count;
	    //}

	    //Call hook in child classes
	    mainHook(argc, argv);
	    
	    return 0;
    
	}




};


#endif 