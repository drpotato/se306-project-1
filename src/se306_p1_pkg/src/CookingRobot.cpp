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
	cooking = false;
	residentName = "Resident0";
	subscriberHunger = nodeHandle->subscribe("hunger", 1000, CookingRobot::hungerCallback);
	subscriberTime = nodeHandle->subscribe("time", 1000, CookingRobot::timeCallback);
	first = true;
	first_call = true;
	x = 0;
	returningHome = false;
	returningHome_first = true;
	moving_to_stove = false;
}

void CookingRobot::doExecuteLoop()
{
	if (RCmode == "cookingRobot")
  	{
    	CookingRobot::controlRobot();	
  	}
	else if (returningHome){
		if(!(CookingRobot::goToNode("nodeLivingRoomByCouch")))
		{
			returningHome = false;
		}
	}
	else if (moving_to_stove)
	{
		if(!(CookingRobot::goToNode("nodeKitchenStove")))
		{
    		ROS_INFO("CHANGED TO cooking");
    		moving_to_stove = false;
    		cooking=true;
    		first = false;
		}
	}
	else if (cooking)
	{
		if (x == 5)
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
				x += 1;
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
 	if (!msg.level >= CRITICAL_LEVEL)
 	{
 		//COOK
 		temp->moving_to_stove = true;
 	}
}
void CookingRobot::timeCallback(msg_pkg::Time msg)
{
	CookingRobot* temp = dynamic_cast<CookingRobot*>( ActorSpawner::getInstance().getActor());
	if ((msg.hour == temp->LUNCH_TIME) || (msg.hour == temp->BREAKFAST_TIME) || (msg.hour == temp->DINNER_TIME))
	{
		temp->moving_to_stove = true;
	}
}