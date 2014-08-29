#include "CompanionRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

string CompanionRobot::getActorName()
{
  return "CompanionRobot";
}

// A robot who provides the Resident with companionship (could be a robotic dog, a video call system, sex bot)
void CompanionRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	giving_morale = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, CompanionRobot::moraleCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	active = false;
	returningHome = false;
	returningHome_first = true;
}

void CompanionRobot::doExecuteLoop()
{
	if (RCmode == "companionRobot")
  	{
    	CompanionRobot::controlRobot();
  	} else if (active) {

  		if (first)
  		{
  			requestLock("companionRobot");
			first = false;
  		}

  		if (haveLock)
  		{
  			if (travellingToResident) {

				if(CompanionRobot::goToNode("nodeBedroomCentre")) {
						travellingToResident = false;
						giving_morale = true;
					}
			 } else if (returningHome) {

					if(CompanionRobot::goToNode("nodeMedicineRobotHome")) {
						returningHome = false;
						first = true;
						active = false;
					}
			 } else if (giving_morale) {
					if (moraleLevel < 100) {
						CompanionRobot::doResponse("morale");
						giving_morale = false;
					} else {
						CompanionRobot::stopResponse("morale");
					}
				}
		} else if (deniedLock) {

			if (otherUnlocked)
			{
				requestLock("companionRobot");
				deniedLock = false;
				otherUnlocked = false;
			}
		}	
	}
}

// TODO: SHOULD BE COMPANIONSHIP/LONELINESS ########################################################################################################
void CompanionRobot::moraleCallback(msg_pkg::Morale msg)
{

 	CompanionRobot* temp = dynamic_cast<CompanionRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;

 	if (msg.level < 70) {
 		temp->active = true;
 		temp->travellingToResident = true;
 	}

 	//ROS_INFO("Changed value");
}

bool CompanionRobot::checkMoraleLevel()
{
	if (moraleLevel>=2 )
	{
		return true;
	}
	return false;
}
