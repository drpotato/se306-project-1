#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <msg_pkg/Location.h>

#include <string.h>

#include "Actor.h"
#include "ActorSpawner.h"

namespace
{
	std::string generateNodeName(unsigned int ID);
	std::string generateStageName(unsigned int ID);
}

Actor::Actor():
	velLinear(0.0),
	velRotational(0.0),
	px(0.0),
	py(0.0),
	theta(0.0),
	
	// Zero out these pointers in case someone accidentally dereferences them too soon
	nodeHandle(0),
	loopRate(0)
{
}

Actor::~Actor()
{
	delete loopRate;
	delete nodeHandle;
}



void Actor::initialSetup(unsigned int robotID)
{
	rosName = generateNodeName(robotID);
	stageName = generateStageName(robotID);
	
	// ros::init needs L-values, so we can't just directly pass (0, ...)
	int fakeArgC = 0;
	ros::init(fakeArgC, 0, rosName.c_str());
	
	nodeHandle = new ros::NodeHandle();
	loopRate = new ros::Rate(10);

	//publisherLocation = n.advertise<msg_pkg::Location>("location", 1000);
	//subscriberLocation = n.subscribe("location", 1000, ((Actor*)this)->Actor::locationCallback);

	publisherLocation = nodeHandle->advertise<msg_pkg::Location>("location", 1000);

	
	// Put custom init stuff here (or make a method and call it from here)
	initialSetupStage();
	doInitialSetup();
}

bool Actor::executeLoop()
{
	if (ros::ok())
	{
		executeLoopStageSubscription();
		// Put custom loop stuff here (or make a method and call it from here)
		
		//Create a location message to publish
		
		msg_pkg::Location location_message;
		//Assign current x and y values to message
		location_message.xpos = px;
		location_message.ypos = py;
		location_message.id = rosName;

		publisherLocation.publish(location_message);

		doExecuteLoop();
		executeLoopStagePublication();
		
		ros::spinOnce();
		loopRate->sleep();
		return true;
	}
	
	return false;
}

void Actor::initialSetupStage()
{
	publisherStageVelocity = nodeHandle->advertise<geometry_msgs::Twist>((stageName + "/cmd_vel").c_str(), 1000);
	subscriberLocation = nodeHandle->subscribe<msg_pkg::Location>((stageName+ "/location").c_str(), 1000, Actor::locationCallback);	
	subscriberStageOdometry  = nodeHandle->subscribe<nav_msgs::Odometry>((stageName + "/odom").c_str(), 1000, 
Actor::StageOdom_callback);
	// subscriberStageLaserScan = nodeHandle->subscribe<sensor_msgs::LaserScan>((stageName + "/base_scan").c_str(), 1000, StageLaser_callback);
}

void Actor::StageOdom_callback(nav_msgs::Odometry msg)
{
  //Grab x and y coordinates from the Odometry message and assign to px and py
  ActorSpawner::getInstance().getActor("jenna")->px = msg.pose.pose.position.x;
  ActorSpawner::getInstance().getActor("jenna")->py = msg.pose.pose.position.y;
  ActorSpawner::getInstance().getActor("j")->robotidentification = msg.child_frame_id;
}

void Actor::locationCallback(msg_pkg::Location msg)
{
 ROS_INFO("Received test message from robot: []");
}


void Actor::executeLoopStageSubscription()
{
	
}

void Actor::executeLoopStagePublication()
{
	// Create and publish a velocity command.
	geometry_msgs::Twist commandVelocity;
	commandVelocity.linear.x  = velLinear;
	commandVelocity.angular.z = velRotational;
	publisherStageVelocity.publish(commandVelocity);
}

namespace
{
	std::string generateNodeName(unsigned int ID)
	{
		char *buffer = new char[128];
		sprintf(buffer, "RobotNode%u", ID);
		
		std::string nodeName(buffer);
		delete[] buffer;
		
		return nodeName;
	}
	
	std::string generateStageName(unsigned int ID)
	{
		char *buffer = new char[128];
		sprintf(buffer, "robot_%u", ID);
		
		std::string nodeName(buffer);
		delete[] buffer;
		
		return nodeName;
	}
}
