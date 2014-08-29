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
	residentName = "Resident0";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, EntertainmentRobot::moraleCallback);

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
<<<<<<< HEAD
	if (y++ >= 200)
	{
		goToNode("nodeKitchenStove");
	}

	// if (RCmode == "entertainmentRobot")
 //  	{
 //    	EntertainmentRobot::controlRobot();
 //        return;
 //  	}
=======
	// if (y++ >= 200)
	// {
	// 	goToNode("nodeKitchenStove");
	// }
	
	if (RCmode == "entertainmentRobot")
  	{
    	EntertainmentRobot::controlRobot();
        return;
  	}
>>>>>>> 8c89235e3e584920d7e5a32ebc52a074af378262

	if (returningHome){
		//ROS_INFO("MOVING TO HOME");

<<<<<<< HEAD
	// 	if (returningHome_first){
	// 		returningHome_first = false;
	// 		//TODO: Matt fix this shit (Target node reset upon reach destination)
	// 		//targetNode = 0;
	// 	}

 //        return;
=======
		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}
        
        return;
>>>>>>> 8c89235e3e584920d7e5a32ebc52a074af378262

	}

<<<<<<< HEAD
	// // If we have finished moving to the resident and we need to entertain:
	// if ((!(goToNode("Resident0"))) && (waiting_to_entertain) && first)
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
=======
	// If we have finished moving to the resident and we need to entertain:
	if ((!(goToNode("Resident0"))) && (waiting_to_entertain) && first)
	{
		// Request the lock
		ROS_INFO("Requesting lock...");
		first = false;
	    EntertainmentRobot::requestLock("Robot");
	}
	// If it has the lock:
	else if (haveLock)
	{
		waiting_to_entertain = false;
		entertaining=true;
		// If it has reached the maximum level
		if (moraleLevel == 5)
		{
			// TODO: Add do last desponse call that kurt implimented
			// Stop entertaining and unlock the resident
			EntertainmentRobot::stopResponse("entertaining");
			EntertainmentRobot::unlock();
			entertaining = false;
			first = false;
			returningHome = true;
		} 
		else
		{
			if (y == 40)
			{
				EntertainmentRobot::doResponse("entertaining");
				y=0;
			} 
			else 
			{
				y++;
			}	
		}
	}
	else if (deniedLock)
	{
		if (otherUnlocked)
		{
			EntertainmentRobot::requestLock("Robot");
			//Set back to false so only requests again once
			deniedLock = false;
			otherUnlocked = false;
		}
	}
>>>>>>> 8c89235e3e584920d7e5a32ebc52a074af378262
	}

// Upon receiving a message published to the 'entertainedness' topic, respond appropriately.
void EntertainmentRobot::moraleCallback(msg_pkg::Morale msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
 	// If it has reached level 1:
 	if (!temp->checkMoraleLevel())
 	{
 		//TODO: Go to resident
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
