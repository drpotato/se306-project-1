#include "EntertainmentRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

string EntertainmentRobot::getActorName()
{
  return "EntertainmentRobot";
}

// A Robot that provides the Resident with entertainment (possibly TV)
void EntertainmentRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, EntertainmentRobot::moraleCallback);
	//subscriberLockStatus = nodeHandle->subscribe("lockStatus", 1000, EntertainmentRobot::lockStatusCallback);
	y = 0;
	x = 0;
	first = true;
	returningHome = false;
	returningHome_first = true;
	waiting_to_entertain = false;
	has_lock = false;
}

void EntertainmentRobot::doExecuteLoop()
{
	goToNode("Resident0");

	// if (RCmode == "entertainmentRobot")
 //  	{
 //    	EntertainmentRobot::controlRobot();
 //  	}

	// if (returningHome){
	// 	//ROS_INFO("MOVING TO HOME");

	// 	if (returningHome_first){
	// 		returningHome_first = false;
	// 		//TODO: Matt fix this shit (Target node reset upon reach destination)
	// 		//targetNode = 0;
	// 	}
        
 //        return;

	// }

	// // If we have finished moving to the resident and we need to entertain:
	// if ((!(this->movingToResident)) && (waiting_to_entertain) && first)
	// {
	// 	// Request the lock
	// 	ROS_INFO("Requesting lock...");
	// 	first = false;
	//     EntertainmentRobot::requestLock("Robot");
	// }
	// // If it has the lock:
	// else if (haveLock)
	// {
	// 	waiting_to_entertain = false;
	// 	entertaining=true;
	// 	// If it has reached the maximum level
	// 	if (moraleLevel == 5)
	// 	{
	// 		// TODO: Add do last desponse call that kurt implimented
	// 		// Stop entertaining and unlock the resident
	// 		EntertainmentRobot::stopResponse("entertaining");
	// 		EntertainmentRobot::unlock();
	// 		entertaining = false;
	// 		first = false;
	// 		returningHome = true;
	// 	} 
	// 	else
	// 	{
	// 		if (y == 40)
	// 		{
	// 			EntertainmentRobot::doResponse("entertaining");
	// 			y=0;
	// 		} 
	// 		else 
	// 		{
	// 			y++;
	// 		}	
	// 	}
	// }
	// else if (deniedLock)
	// {
	// 	if (otherUnlocked)
	// 	{
	// 		EntertainmentRobot::requestLock("Robot");
	// 		//Set back to false so only requests again once
	// 		deniedLock = false;
	// 		otherUnlocked = false;
	// 	}
	// }
}

// Upon receiving a message published to the 'entertainedness' topic, respond appropriately.
void EntertainmentRobot::moraleCallback(msg_pkg::Morale msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
 	// If it has reached level 1:
 	if (!temp->checkMoraleLevel())
 	{
 		temp->startMovingToResident();
 		temp->waiting_to_entertain = true;
 	}
}

bool EntertainmentRobot::checkMoraleLevel()
{
	if (moraleLevel>= CRITICAL_LEVEL )
	{
		return true;
	}
	return false;
}
