#include "CookingRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"


// A Robot to cook meals for the Resident. Meals happen according to a regular schedule and can[not] be interrupted.
void CookingRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, CookingRobot::moraleCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void CookingRobot::doExecuteLoop()
{
	if (returningHome){
		//ROS_INFO("MOVEING TO HOME");

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}

        return;

	}

	if (!entertaining)
	{
		if (!checkCookingLevel())
		{
			if (first_call)
			{
				//this->activeNode = &node5;
				//this->startMovingToResident();
				first_call = false;
			}
	    	if (!(this->movingToResident) )
	    	{
	    		//CookingRobot::doResponse("entertaining");
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		entertaining=true;
	    		first = false;
	    	}
		}
	}
	else
	{
		if (moraleLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			CookingRobot::stopResponse("entertaining");
			entertaining = false;
			returningHome = true;

		}
		else
		{

			if (y == 40)
			{
				CookingRobot::doResponse("entertaining");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}

// TODO: SHOULD BE COOKING/FOOD ########################################################################################################################
void CookingRobot::moraleCallback(msg_pkg::Morale msg)
{
 	CookingRobot* temp = dynamic_cast<CookingRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool CookingRobot::checkCookingLevel()
{
	if (moraleLevel>=2 )
	{
		return true;
	}
	return false;
}
