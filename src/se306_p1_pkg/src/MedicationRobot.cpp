#include "MedicationRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

// A Robot which gives the Resident his medication.
void MedicationRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	healthLevel = 5;
	healing = false;
	residentName = "RobotNode2";
	subscriberHealth = nodeHandle->subscribe("health", 1000, MedicationRobot::healthCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void MedicationRobot::doExecuteLoop()
{
	if (RCmode == "medicationRobot")
  	{
    	MedicationRobot::controlRobot();
  	}
	if (returningHome){

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}

        return;

	}

	if (!healing)
	{
		if (checkHealthLevel())
		{
			//ROS_INFO("Nothing to do here");
		}
		else
		{
			if (first_call)
			{
				this->startMovingToResident();
				first_call = false;
			}

	    	if (!(this->movingToResident) )
	    	{
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		healing=true;
	    		first = false;
	    	}

		}
	}
	else
	{
		if (healthLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			MedicationRobot::stopResponse("entertaining");
			healing = false;
			returningHome = true;

		}
		else
		{

			if (y == 40)
			{
				MedicationRobot::doResponse("entertaining");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}


void MedicationRobot::healthCallback(msg_pkg::Health msg)
{
 	MedicationRobot* temp = dynamic_cast<MedicationRobot*>( ActorSpawner::getInstance().getActor());

 	temp->healthLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool MedicationRobot::checkHealthLevel()
{
	if (healthLevel>=2 )
	{
		return true;
	}
	return false;
}
