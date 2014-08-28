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
#include <msg_pkg/RequestLock.h>
#include <msg_pkg/Unlock.h>

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"
#include "ActorLocation.h"

#include "keyinput/KeyboardListener.hpp"

namespace
{
	std::string generateNodeName(unsigned int ID, string nodeName);
	std::string generateStageName(unsigned int ID, string nodeName);
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

{
    // Setup navigation waypoint nodes.
    this->targetNode = 0;

    // Next to the Resident.
    node1Name = "testnode1";

    node2Name = "testnode2";
    node3Name = "testnode3";
    node4Name = "testnode4";

    // Next to the Entertainment Robot.
    node5Name = "testnode5";
    
    // At the door to the house
    nodeDoorName = "nodeDoorName";

    node1 = PathPlannerNode(&node1Name,-2.5,3);
    node2 = PathPlannerNode(&node2Name,-2.5,-0);
    node3 = PathPlannerNode(&node3Name,3,0);
    node4 = PathPlannerNode(&node4Name,3,3);
    node5 = PathPlannerNode(&node5Name, -2.5, -3);
    nodeDoor = PathPlannerNode(&nodeDoorName,2.8,5);

    // Specify which nodes have a clear line of sight to each other.
    node1.addNeighbour(&node2);
    node1.addNeighbour(&node5);

    node2.addNeighbour(&node1);
    node2.addNeighbour(&node3);
    node2.addNeighbour(&node5);

    node3.addNeighbour(&node2);
    node3.addNeighbour(&node4);
    node3.addNeighbour(&nodeDoor);

    node4.addNeighbour(&node3);
    node4.addNeighbour(&nodeDoor);

    node5.addNeighbour(&node2);
    node5.addNeighbour(&node1);
    
    nodeDoor.addNeighbour(&node3);
    nodeDoor.addNeighbour(&node4);

    // Add the nodes to the path planner's graph of nodes and connections.
    PathPlanner::addNode(&node1);
    PathPlanner::addNode(&node2);
    PathPlanner::addNode(&node3);
    PathPlanner::addNode(&node4);
    PathPlanner::addNode(&node5);
    PathPlanner::addNode(&nodeDoor);

    this->activeNode = &node1;

    this->movingToResident = false;
}

Actor::~Actor()
{
	delete loopRate;
	delete nodeHandle;
}

void Actor::initialSetup(unsigned int robotID, double px, double py, double theta)
{
	rosName = generateNodeName(robotID, getActorName());
	stageName = generateStageName(robotID, getActorName());
	pxInitial = px;
	pyInitial = py;
	thetaInitial = theta;
    haveLock = false;
    deniedLock = false;
    otherUnlocked = false;

	// ros::init needs L-values, so we can't just directly pass (0, ...)
	int fakeArgC = 0;
	ros::init(fakeArgC, 0, rosName.c_str());

	nodeHandle = new ros::NodeHandle();
	loopRate = new ros::Rate(LOOP_RATE);

	publisherLocation = nodeHandle->advertise<msg_pkg::Location>("location", 1000);

	publisherInteraction = nodeHandle->advertise<msg_pkg::Interaction>("interaction", 1000);

    publisherRequestLock = nodeHandle->advertise<msg_pkg::RequestLock>("requestLock", 1000);
    publisherUnlock = nodeHandle->advertise<msg_pkg::Unlock>("unlock", 1000);

    subscriberLockStatus = nodeHandle->subscribe("lockStatus", 1000, Actor::lockStatusCallback);
    subscriberUnlock = nodeHandle->subscribe("unlock", 1000, Actor::unlockCallback);

	// Put custom init stuff here (or make a method and call it from here)
	KeyboardListener::init();
	initialSetupStage();
	doInitialSetup();

    RCmode = "";
    
}

bool Actor::executeLoop()
{
	if (ros::ok())
	{
		executeLoopStageSubscription();
		// Put custom loop stuff here (or make a method and call it from here)

		publishLocation();

        checkKeyboardPress();

        moveToResident();

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

// Process messages publish to the 'location' topic.
// These messages each contain the current location of a single Actor.
void Actor::locationCallback(msg_pkg::Location msg)
{

}

void Actor::lockStatusCallback(msg_pkg::LockStatus msg)
{
    Actor *actorPtr = ActorSpawner::getInstance().getActor();
    if (0 == (strcmp(msg.robot_id.c_str(), actorPtr->rosName.c_str())) && (msg.has_lock == true))
    {
        if (msg.has_lock)
        {
            actorPtr->deniedLock = false;
            actorPtr->haveLock=true;
            ROS_INFO("I HAVE THE LOCK %s",actorPtr->rosName.c_str() );
        } else
        {
            actorPtr->haveLock = false;
            actorPtr->deniedLock = true;
            ROS_INFO("I WAS DENIED THE LOCK %s",actorPtr->rosName.c_str() );
        }
    }

}

void Actor::unlockCallback(msg_pkg::Unlock msg)
{
    Actor *actorPtr = ActorSpawner::getInstance().getActor();
    if (0 == (strcmp(msg.robot_id.c_str(), actorPtr->rosName.c_str())))
    {
        actorPtr->haveLock=false;
        actorPtr->deniedLock = false;
        ROS_INFO("I LOST THE LOCK %s", actorPtr->rosName.c_str());
    } else if (actorPtr->deniedLock)
    {
        actorPtr->otherUnlocked = true;
    }

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

// Check to see if any keys are pressed and set the mode appropriately
void Actor::checkKeyboardPress()
{
    KeyboardListener &keyboardListener = KeyboardListener::getInstance();
    if (keyboardListener.isKeyTapped(ups::KEY_D_CODE))
    {
        //doctor
        toggleMode("doctor");
        ROS_INFO("hello doctor");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_N_CODE) && !(keyboardListener.isKeyPressed(ups::KEY_SPACE_CODE)))
    {
        //nurse1
        toggleMode("nurse1");
    }
    else if ((keyboardListener.isKeyTapped(ups::KEY_N_CODE)) && (keyboardListener.isKeyTapped(ups::KEY_SPACE_CODE)))
    {
        //nurse2
        toggleMode("nurse2");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_F_CODE) && !(keyboardListener.isKeyPressed(ups::KEY_SPACE_CODE)))
    {
        //friend1
        toggleMode("friend1");
    }
    else if ((keyboardListener.isKeyTapped(ups::KEY_F_CODE)) && (keyboardListener.isKeyTapped(ups::KEY_SPACE_CODE)))
    {
        //friend2
        toggleMode("friend2");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_R_CODE) && !(keyboardListener.isKeyPressed(ups::KEY_SPACE_CODE)))
    {
        //relative1
        toggleMode("relative1");
    }
    else if ((keyboardListener.isKeyTapped(ups::KEY_R_CODE)) && (keyboardListener.isKeyTapped(ups::KEY_SPACE_CODE)))
    {
        //relative2
        toggleMode("relative2");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_C_CODE) && !(keyboardListener.isKeyPressed(ups::KEY_SPACE_CODE)))
    {
        //caregiver1
        toggleMode("caregiver1");
    }
    else if ((keyboardListener.isKeyTapped(ups::KEY_C_CODE)) && (keyboardListener.isKeyTapped(ups::KEY_SPACE_CODE)))
    {
        //caregiver2
        toggleMode("caregiver2");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_M_CODE))
    {
        //medicationRobot
        toggleMode("medicationRobot");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_E_CODE))
    {
        //entertainmentRobot
        toggleMode("entertainmentRobot");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_T_CODE))
    {
        //companionRobot
        toggleMode("companionRobot");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_B_CODE))
    {
        //cookingRobot
        toggleMode("cookingRobot");
    }
    else if (keyboardListener.isKeyTapped(ups::KEY_ENTER_CODE))
    {
        //resident
        toggleMode("resident");
    }
}

// Checks if there is any mode currently set
bool Actor::modeSet()
{
    if (RCmode == "")
    {
        return false;
    }
    return true;
}

// Checks whether we are in a certain mode or not
bool Actor::inMode(string mode)
{
    if (RCmode == mode)
    {
        return true;
    }
    return false;
}

// Turns a mode on or off
void Actor::toggleMode(string mode)
{
    // If we are in this mode, turn the mode off
    if (inMode(mode)) 
    {
        RCmode = "";
    }
    // If there is no current mode, we are free to put it in this mode
    else if (!modeSet())
    {
        RCmode = mode;
    }
    // Cannot set the mode on or off if it is currently in another mode
}

// Only call this method from the subclass IF it is in your corresponding mode (RCmode)
void Actor::controlRobot()
{
    KeyboardListener &keyboardListener = KeyboardListener::getInstance();
    velRotational = 0.0;
    velLinear = 0.0;
    
    if (keyboardListener.isKeyPressed(ups::KEY_UP_CODE))
    {
        velLinear += 1.0;
    }
    
    if (keyboardListener.isKeyPressed(ups::KEY_DOWN_CODE))
    {
        velLinear -= 1.0;
    }
    
    if (keyboardListener.isKeyPressed(ups::KEY_LEFT_CODE))
    {
        velRotational += 1.0;
    }
    
    if (keyboardListener.isKeyPressed(ups::KEY_RIGHT_CODE))
    {
        velRotational -= 1.0;
    }
}

// Request the lock for the resident
void Actor::requestLock(std::string actor_name)
{
    msg_pkg::RequestLock request;
    request.robot_id = rosName;
    request.actor_name = actor_name;
    publisherRequestLock.publish(request);
}
// Give up the resident lock
void Actor::unlock()
{
    msg_pkg::Unlock unlock;
    unlock.robot_id = rosName;
    publisherUnlock.publish(unlock);
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

void Actor::startMovingToResident() {
    this->movingToResident = true;
}

bool Actor::moveToResident() {
    // if (this->movingToResident) {
    // 	//ROS_INFO("MOVING TO RESIDENT");
    //     PathPlannerNode *target = this->pathPlanner.getNode(&node1Name);
    //     vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    //     if ( this->goToNode(path))
    //     {
    //     	ROS_INFO("CHANGED MOVING TO RESIDENT");
    //     	this->movingToResident = false;
    //     }
    // }
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
bool Actor::gotoPosition(double x,double y){
    // Face the node
    if (faceDirection(x,y) < 0.1){
        double distance = sqrt((x-this->px)*(x-this->px) + (y-this->py)*(y-this->py));

        //ROS_DEBUG("Distance is %f",distance);

        if (distance > 0.01){
            faceDirection(x,y);
            this->velLinear = distance*1;
            return true;
        }else{
            this->velLinear = 0;
            return false;
        }
    }else{
        //ROS_DEBUG("Target: %f",faceDirection(x,y));
        this->velLinear = 0;
        return true;
    }
}

bool Actor::goToNode(string* nodeName) {
    activeNode = getActiveNode();

    targetNode = PathPlanner::getNode(nodeName);
                //TODO: REWIRITE THIS TO USE NEW SYSTEM ################################################################################################
    // goingToNode = pathPlanner.getClosestNode(goingToX, goingToY);

    // vector <PathPlannerNodepath = pathPlanner.pathToNode(this->activeNode, goingToNode);

    // //Get the node
    // if (targetNode >= path.size()){
    //     //We have arrived at the last node
    //     this->velLinear = 0;
        
    //     return true;
    // }
    // if (!this->gotoPosition(path[targetNode]->px,path[targetNode]->py)){
    //     this->activeNode = path[targetNode];
    //     targetNode++;
    // }else{
    //     ROS_DEBUG("current position %f %f",px,py);
    // }
    // return false;
}

// Find the closest waypoint node to this Actor's current position.
PathPlannerNode* Actor::getActiveNode() {
    return PathPlanner::getClosestNode(this->px, this->py);
}

ros::NodeHandle &Actor::getNodeHandle() const
{
  return *nodeHandle;
}

namespace
{
	std::string generateNodeName(unsigned int ID, string nodeName)
	{
          ostringstream os;
          os << nodeName << ID;
          string s = os.str();
          return s;
	}

	std::string generateStageName(unsigned int ID, string nodeName)
	{
          // TODO Jamie, why the fuck does this have to mess with the behaviour of robots?
          ostringstream os;
          os << "robot_" << ID;
          string s = os.str();
          return s;
	}
}
