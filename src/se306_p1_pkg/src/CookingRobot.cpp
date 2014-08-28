#include "CookingRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

string CookingRobot::getActorName()
{
  return "CookingRobot";
}

// A Robot to cook meals for the Resident. Meals happen according to a regular schedule and can[not] be interrupted.
void CookingRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	hungerLevel = 5;
	cooking = false;
	residentName = "RobotNode2";
	subscriberHunger = nodeHandle->subscribe("hunger", 1000, CookingRobot::hungerCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void CookingRobot::doExecuteLoop()
{
	if (RCmode == "cookingRobot")
  	{
    	CookingRobot::controlRobot();
  	}
	if (returningHome){
		//ROS_INFO("MOVEING TO HOME");

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}

        return;

}

	if (!cooking)
	{
		if (!checkHungerLevel())
		{
			if (first_call)
			{
				//this->activeNode = &node5;
				this->startMovingToResident();
				first_call = false;
			}
			//Call method to do the cooking
			//PathPlannerNode *target = this->pathPlanner.getNode(&node2Name);
	    	//vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);



	    	//The or in this case is just for the alpha, remove once the robot is capable of reaching the resident
	    	if (!(this->movingToResident) )
	    	{
	    		//CookingRobot::doResponse("cooking");
	    		ROS_INFO("CHANGED TO cooking");
	    		cooking=true;
	    		first = false;
	    	}
			//After finished cooking set cooking to false

		}
	}
	else
	{
		if (hungerLevel == 5)
		{
			//Add do last response call that kurt implimented
			CookingRobot::stopResponse("cooking");
			cooking = false;
			returningHome = true;

		}
		else
		{
			if (y == 40)
			{
				CookingRobot::doResponse("cooking");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}

void CookingRobot::hungerCallback(msg_pkg::Hunger msg)
{
 	CookingRobot* temp = dynamic_cast<CookingRobot*>( ActorSpawner::getInstance().getActor());

 	temp->hungerLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool CookingRobot::checkHungerLevel()
{
	if (hungerLevel>=2 )
	{
		return true;
	}
	return false;
}
