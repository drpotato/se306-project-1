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
	entertainednessLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberEntertainedness = nodeHandle->subscribe("entertainedness", 1000, EntertainmentRobot::entertainednessCallback);	
	y = 0;
	x = 0;
	first = true;

}

void EntertainmentRobot::doExecuteLoop()
{
	
	if (!entertaining)
	{
		if (checkEntertainmentLevel())
		{
			//ROS_INFO("Nothing to do here");
		} else {
			
			//Call method to do the entertaining
			PathPlannerNode *target = this->pathPlanner.getNode(&node2Name);
	    	vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);

	    	//The or in this case is just for the alpha, remove once the robot is capable of reaching the resident
	    	if ((this->goToNode(path)) | ((y>=30) && first)){
	    		//EntertainmentRobot::doResponse("entertaining");
	    		entertaining=true;
	    		first = false;
	    	}
	    	y=y+1;
			//After finished entertaining set entertaining to flase
			
		}
	} else {
		if (entertainednessLevel == 5)
		{
			entertaining = false;
			y=0;

		} else if ((50 % y) == 0)
		{
			EntertainmentRobot::doResponse("entertaining");
		} else {}
		y=y+1;
		
	}
}

void EntertainmentRobot::entertainForPeriod()
{
	int x = 0;
	while(x<10)
	{
		EntertainmentRobot::doResponse("entertaining");
		x=x+1;
	}
	entertaining=false;
}

void EntertainmentRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor("kurt fix this shit"));

 	temp->entertainednessLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool EntertainmentRobot::checkEntertainmentLevel()
{
	if (entertainednessLevel>=2 ){
		return true;
	}
	return false;
}