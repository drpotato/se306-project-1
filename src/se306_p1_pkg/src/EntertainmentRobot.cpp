#include "EntertainmentRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"



void EntertainmentRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	entertainednessLevel = 1;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberEntertainedness = nodeHandle->subscribe("entertainedness", 1000, EntertainmentRobot::entertainednessCallback);	

}

void EntertainmentRobot::doExecuteLoop()
{
	
	if (checkEntertainmentLevel())
	{
		ROS_INFO("Nothing to do here");
	} else {
		entertaining = true;
		//Call method to do the entertaining
		PathPlannerNode *target = this->pathPlanner.getNode(&residentName);
    	vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    	this->goToNode(path);
		EntertainmentRobot::doResponse("entertaining");
		//After finished entertaining set entertaining to flase
		//entertaining = false;;
	}
}


void EntertainmentRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor("kurt fix this shit"));

 	temp->entertainednessLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool EntertainmentRobot::checkEntertainmentLevel()
{
	if (entertainednessLevel>=2 | entertaining){
		return true;
	}
	return false;
}