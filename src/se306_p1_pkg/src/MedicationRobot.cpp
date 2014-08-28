#include "MedicationRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

string MedicationRobot::getActorName()
{
  return "MedicationRobot";
}

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

	if (travellingToResident)
	{
		//This is here so that it will compile. Get rid of when uncommenting goToNode()
		bool temp;
		//bool temp = goToNode("RosNode0");
		if (!temp)
		{
			travellingToResident = false;
			healing = true;
			first = true;
		}
		return;
	}

	if (healing)
	{
		//Send message to patient to rise health stats
		if (first)
		{
			requestLock("MedicationRobot");
			first = false;
		}

		if (haveLock)
		{
			//Treat resident
			if (!(healing >= 99))
			{
				doResponse("mend");
			} else 
			{
				returningHome = true;
				healing = false;
			}
		} else if (deniedLock)
		{
			if (otherUnlocked)
			{
				requestLock("MedicationRobot");
				deniedLock = false;
				otherUnlocked = false;
			}
		}
	}

	if (returningHome){

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}

        return;

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
