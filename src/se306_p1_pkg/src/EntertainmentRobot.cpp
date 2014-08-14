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
	first_call = true;

    //this->activeNode = &node4;
    //this->startMovingToResident();

}

void EntertainmentRobot::doExecuteLoop()
{

	if (!entertaining)
	{
		if (checkEntertainmentLevel())
		{
			//ROS_INFO("Nothing to do here");
		} else {
			if (first_call)
			{
				this->activeNode = &node4;
				this->startMovingToResident();
				first_call = false;
			}
			//Call method to do the entertaining
			//PathPlannerNode *target = this->pathPlanner.getNode(&node2Name);
	    	//vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
	    	
    		

	    	//The or in this case is just for the alpha, remove once the robot is capable of reaching the resident
	    	if (!(this->movingToResident) ){
	    		//EntertainmentRobot::doResponse("entertaining");
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		entertaining=true;
	    		first = false;
	    	}
	    	
			//After finished entertaining set entertaining to flase

		}
	} else {
		if (entertainednessLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			EntertainmentRobot::stopResponse("entertaining");
			entertaining = false;
			

		} else 
		{
			//TODO: KURT FIX THIS SHIT
			if (y=10000){
				EntertainmentRobot::doResponse("entertaining");
				y=0;
			} else 
			{
				y++;
			}

			
			
		} 
		
	}
}


void EntertainmentRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor());

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
