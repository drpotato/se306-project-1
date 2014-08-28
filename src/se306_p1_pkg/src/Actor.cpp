#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/lexical_cast.hpp>
#include <ros/console.h>
#include <tf/tf.h>

#include <math.h>
#include <string>
#include <vector>
#include <msg_pkg/Location.h>
#include <msg_pkg/Interaction.h>

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

#include "keyinput/KeyboardListener.hpp"

namespace
{
	std::string generateNodeName(unsigned int ID);
	std::string generateStageName(unsigned int ID);
}

Actor::Actor():
    // Zero out these pointers initially in case someone accidentally dereferences them too soon
	velLinear(0.0),
	velRotational(0.0),
	pxInitial(0.0),
	pyInitial(0.0),
	thetaInitial(0.0),
	px(0.0),
	py(0.0),
	theta(0.0),
	nodeHandle(0),
	loopRate(0)
{ }

Actor::~Actor()
{
	delete loopRate;
	delete nodeHandle;
}

void Actor::initialSetup(unsigned int robotID, double px, double py, double theta)
{
	rosName = generateNodeName(robotID);
	stageName = generateStageName(robotID);
	pxInitial = px;
	pyInitial = py;
	thetaInitial = theta;

	// ros::init needs L-values, so we can't just directly pass (0, ...)
	int fakeArgC = 0;
	ros::init(fakeArgC, 0, rosName.c_str());

	nodeHandle = new ros::NodeHandle();
	loopRate = new ros::Rate(LOOP_RATE);

	publisherLocation = nodeHandle->advertise<msg_pkg::Location>("location", 1000);

	publisherInteraction = nodeHandle->advertise<msg_pkg::Interaction>("interaction", 1000);


	//PathPlanner
	this->currentNode = pathPlanner.getClosestNode(this->px, this->py)->getName();
	this->currentNodeIndex = 0;
	// Put custom init stuff here (or make a method and call it from here)
	KeyboardListener::init();
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
	subscriberStageOdometry  = nodeHandle->subscribe<nav_msgs::Odometry>((stageName + "/odom").c_str(), 1000,
Actor::StageOdom_callback);

}

void Actor::StageOdom_callback(nav_msgs::Odometry msg)
{
  //Grab x and y coordinates from the Odometry message and assign to px and py
  ActorSpawner &actorSpawner = ActorSpawner::getInstance();
  Actor *actorPtr = ActorSpawner::getInstance().getActor();

  actorPtr->px = actorPtr->pxInitial + msg.pose.pose.position.x;
  actorPtr->py = actorPtr->pyInitial + msg.pose.pose.position.y;
  actorPtr->theta = actorPtr->thetaInitial + tf::getYaw(msg.pose.pose.orientation);
}

// Publish a message containing own x and y coordinates to the 'location' topic.
void Actor::publishLocation()
{
	// Create a location message to publish.
	msg_pkg::Location locationMessage;
	// Assign current x and y values to message.
	locationMessage.xpos = px;
	locationMessage.ypos = py;
	// Assign id to rosName.
	locationMessage.id = rosName;
	// Publish the message.
	publisherLocation.publish(locationMessage);
}

// Process messages coming from Stage.
void Actor::executeLoopStageSubscription()
{

}

// Publish any changes in linear and rotational velocity to Stage.
void Actor::executeLoopStagePublication()
{
	// Create and publish a velocity command.
	geometry_msgs::Twist commandVelocity;
	commandVelocity.linear.x  = velLinear;
	commandVelocity.angular.z = velRotational;
	publisherStageVelocity.publish(commandVelocity);
}

void Actor::doResponse(const char *attribute)
{
	msg_pkg::Interaction interaction;
	interaction.attribute = attribute;
	interaction.amount = 1;
	publisherInteraction.publish(interaction);

	ROS_INFO("%s (%s) is performing \"%s\"", rosName.c_str(), stageName.c_str(), attribute);

	// Spin for visual feedback
	velRotational = 1.0; // fmod(ros::Time::now().toSec(), 1.0) >= 0.5 ? 1.0 : -1.0
	velLinear = 0.0;
}

void Actor::stopResponse(const char *attribute)
{
  // TODO maybe do something with the attribute

  // Stop moving TODO Kurt, this could possibly be threaded and be delayed on a new thread
  velRotational = 0.0;
  velLinear = 0.0;
}

double Actor::faceDirection(double x,double y){
    // Calculate target angle.
    double vx = x-this->px;
    double vy = y-this->py;

    double ax = cos(this->theta)*vx + sin(this->theta)*vy;
    double ay = cos(this->theta)*vy - sin(this->theta)*vx;

    double angle = atan2(ay,ax);

    // Set velocity to face the angle using PID.
    this->velRotational = (angle)*1;
    return abs(angle);
}

// Moves the Actor in a straight line towards the given x and y coordinates.
// Returns true while moving/rotating, and false when it has arrived at its location and stopped.
bool Actor::gotoPosition(double x,double y) {
    // Face the node
    if (faceDirection(x,y) < 0.1) {
        double distance = sqrt((x-this->px)*(x-this->px) + (y-this->py)*(y-this->py));

        ROS_DEBUG("Distance is %f",distance);

        if (distance > 0.01) {
            faceDirection(x,y);
            this->velLinear = distance*1;
            return true;
        } else {
            this->velLinear = 0;
            return false;
        }
    } else {
        ROS_DEBUG("Target: %f",faceDirection(x,y));
        this->velLinear = 0;
        return true;
    }
}

// Returns false when it has arrived at the target node, and true when in transit.
bool Actor::goToNode(string nodeName) {
	//Update the graph before doing anything else
	pathPlanner.update(rosName);

    ROS_INFO("current position %f %f", px, py);

    string closest = pathPlanner.getClosestNode(this->px, this->py)->getName();
    ROS_INFO("Name of closest node is %s", closest.c_str());

    vector <PathPlannerNode*> path = pathPlanner.pathToNode(rosName, nodeName);


    if (currentNodeIndex < path.size()-1) {

				PathPlannerNode* nextNode = pathPlanner.getNode(currentNode);

				if (!this->gotoPosition(nextNode->px, nextNode->py)) {
            // We have arrived at the next node.
            ROS_INFO_STREAM("We have arrived at a node on the path");
            currentNode = path[currentNodeIndex+1]->getName();
						currentNodeIndex++;
            return true;
        } else {
						ROS_INFO_STREAM("We are travelling between nodes");
            return true;
				}
    } else {
        ROS_INFO_STREAM("We have arrived at our final destination!");
				currentNode = nodeName;
        currentNodeIndex = 0;
        return false;
    }
}

ros::NodeHandle &Actor::getNodeHandle() const
{
  return *nodeHandle;
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
