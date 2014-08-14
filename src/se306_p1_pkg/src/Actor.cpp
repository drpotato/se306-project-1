#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/lexical_cast.hpp>
#include <ros/console.h>
#include <tf/tf.h>
//#include <msg_pkg/Location.h>
#include <math.h>
#include <string>
#include <msg_pkg/Location.h>

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
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
    //Create path planner and setup nodes
    this->pathPlanner = PathPlanner();
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
		
		publishLocation();

		doExecuteLoop();
		executeLoopStagePublication();
        ROS_DEBUG("loop");
		ros::spinOnce();
		loopRate->sleep();
		return true;
	}
	
	return false;
}

void Actor::initialSetupStage()
{
	publisherStageVelocity = nodeHandle->advertise<geometry_msgs::Twist>((stageName + "/cmd_vel").c_str(), 1000);
	subscriberLocation = nodeHandle->subscribe("location", 1000, Actor::locationCallback);	
	subscriberStageOdometry  = nodeHandle->subscribe<nav_msgs::Odometry>((stageName + "/odom").c_str(), 1000, 
Actor::StageOdom_callback);

}

void Actor::StageOdom_callback(nav_msgs::Odometry msg)
{
  //TODO: FIX THIS SHIT
  //Grab x and y coordinates from the Odometry message and assign to px and py
  ActorSpawner::getInstance().getActor("kurt fix this shit")->px = msg.pose.pose.position.x;
  ActorSpawner::getInstance().getActor("kurt fix this shit")->py = msg.pose.pose.position.y;
ActorSpawner::getInstance().getActor("kurt fix this shit")->theta = tf::getYaw(msg.pose.pose.orientation);
  // std::stringstream ss;
  // ss << ActorSpawner::getInstance().getActor("")->px;
  // ROS_INFO("%s", ss.str().c_str());
}

void Actor::locationCallback(msg_pkg::Location msg)
{
 
}

void Actor::publishLocation()
{
	//Create a location message to publish
	msg_pkg::Location locationMessage;
	//Assign current x and y values to message
	locationMessage.xpos = px;
	locationMessage.ypos = py;
	//Assign id to rosName
	locationMessage.id = rosName;
	//Publish the message
	publisherLocation.publish(locationMessage);
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


double Actor::faceDirection(double x,double y){
    
    double vx = x-this->px;
    double vy = x-this->py;
    
    double ax = cos(this->theta)*vx + sin(this->theta)*vy;
    double ay = cos(this->theta)*vy - sin(this->theta)*vx;
    
    double angle = atan2(ay,ax);
    //Calculate target angle
    
    //Set velocity to face the angle using PID
    
    this->velRotational = (angle)*1;
    return abs(angle);
}

bool Actor::gotoPosition(double x,double y){
    //Face the node
    if (faceDirection(x,y) < 0.1){
        double distance = sqrt((x-this->px)*(x-this->px) + (y-this->py)*(y-this->py));
        if (distance > 0.1){
            faceDirection(x,y);
            this->velLinear = distance*1;
            distance = sqrt((x-this->px)*(x-this->px) + (y-this->py)*(y-this->py));
            return true;
        }else{
            this->velLinear = 0;
            return false;
        }
    }else{
        this->velLinear = 0;
        return true;
    }
    
}

void Actor::goToNode(vector<PathPlannerNode*> &path){
    //Get the node
        ROS_INFO("there are  %d node",path.size());
    
    for (int i=0;i<path.size();i++){
        if (!path[i]->visited){
            if (!this->gotoPosition(path[i]->px,path[i]->py)){
                path[i]->visited = true;
            }else{
                ROS_INFO("Going to node %d",i);
            }
        }
    }
}
