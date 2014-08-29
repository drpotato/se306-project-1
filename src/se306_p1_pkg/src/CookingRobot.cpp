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
	givingFood = false;
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
  	else if (givingFood)
  	{
  		if(!(CookingRobot::goToNode("Resident")))
		{
			givingFood = false;
			returningHome = true;
			velRotational = 0.0;
  			velLinear = 0.0;
		}
  	}
	else if (returningHome){
		if(!(CookingRobot::goToNode("nodeLivingRoomByCouch")))
		{
			returningHome = false;
			velLinear = 0.0;
			velRotational = 0.0;
		}
	}
	else if (moving_to_stove)
	{
		if(!(CookingRobot::goToNode("nodeKitchenStove")))
		{
    		moving_to_stove = false;
    		cooking=true;
    		first = false;
    		velRotational = 0.0;
  			velLinear = 0.0;
		}
	}
	else if (cooking)
	{
		if (x == 5)
		{
			//Add do last response call that kurt implimented
			cooking = false;
			givingFood = true;
			velRotational = 0.0;
  			velLinear = 0.0;
		}
		else
		{
			if (y == 40)
			{
				y=0;
				x = x + 1;
				ROS_INFO("Cooking robot is cooking up a feast!");
				// Spin for visual feedback
				velRotational = 1.0;
				velLinear = 0.0;
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
	if ((msg.hour == temp->LUNCH_TIME-1) || (msg.hour == temp->BREAKFAST_TIME-1) || (msg.hour == temp->DINNER_TIME-1))
	{
		temp->moving_to_stove = true;
	}
}