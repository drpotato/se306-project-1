#include "Resident.h"
#include <string.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Entertainedness.h>
#include "Actor.h"
#include "ActorSpawner.h"

void Resident::doInitialSetup()
{
  velLinear = 0;
  velRotational = 0.0;

  // Initially the resident is not locked
  lock_ = false;

  // Set levels to maximum initially
  entertainedness_level_ = 5;
  socialness_level_ = 5;

  entertainment_count_ = 0;
  socialness_count_ = 0;
  e_dropped_ = false;
  e_replenished_ = false;

  // Set up a publishers
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherEntertainedness = nodeHandle->advertise<msg_pkg::Entertainedness>("entertainedness", 1000);

  // Set up subscribers
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
}

void Resident::doExecuteLoop()
{
	//PathPlannerNode *target = this->pathPlanner.getNode(&node4Name);
    //vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    //this->goToNode(path);
    
	if (entertainment_count_ >= WAIT_TIME && !e_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->entertainedness_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			e_dropped_ = true;
			//ROS_INFO("e_dropped changed");
		}
		else 
		{
			// reduce the level every 1000 counts
			residentInstance->entertainedness_level_--;
			//Create a socialness message to publish
			msg_pkg::Entertainedness entertainednessMessage;
			//Assign current socialness level to the message
			entertainednessMessage.level = residentInstance->entertainedness_level_;
			//Publish the message
			residentInstance->publisherEntertainedness.publish(entertainednessMessage);

		}
		entertainment_count_ = 0;
	}
	else if (entertainment_count_ < WAIT_TIME && !e_dropped_)
	{
		entertainment_count_++;
	}
	else if (e_replenished_ && (socialness_count_ >= WAIT_TIME) && !s_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->socialness_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			s_dropped_ = true;
		}
		else 
		{
			// reduce the level every 1000 counts
			residentInstance->socialness_level_--;
			//Create a socialness message to publish
			msg_pkg::Socialness socialnessMessage;
			//Assign current socialness level to the message
			socialnessMessage.level = residentInstance->socialness_level_;
			//Publish the message
			residentInstance->publisherSocialness.publish(socialnessMessage);
		}
		socialness_count_ = 0;
	}
	else if (e_replenished_ && (socialness_count_ < WAIT_TIME) && !s_dropped_)
	{
		socialness_count_++;
	}
}

/*
 * A robot should not be able to interact with the resident if the resident is currently locked.
 */
bool Resident::isLocked()
{
  return lock_;
}

/*
 * Robots should only lock the resident if the resident is currently not locked
 */
void Resident::lock()
{
  lock_ = true;
}

/*
 * Robots should unlock the resident after interation * so that another robot may interact with the resident.
 */
void Resident::unlock()
{
  lock_ = false;
}

void Resident::interactionCallback(msg_pkg::Interaction msg)
{

  std::string attribute = msg.attribute;
  int amount = msg.amount;

  // Get the class instance
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 1.0; // Rotate to show is being interacted with

  if (attribute == "socialising")
  {
  	// Get new level
  	int newLevel = getNewLevel(amount, residentInstance->socialness_level_);
	// Update the residents socialness level
	residentInstance->socialness_level_ = newLevel;
	//Create a socialness message to publish
	msg_pkg::Socialness socialnessMessage;
	//Assign current socialness level to the message
	socialnessMessage.level = newLevel;

	if (newLevel == 5)
	{
		residentInstance->stopRobotSpinning();
	}

	//Publish the message
	residentInstance->publisherSocialness.publish(socialnessMessage);
  } 
  else if (attribute == "entertaining")
  {
	// Get new level
	int newLevel = getNewLevel(amount, residentInstance->entertainedness_level_);
	// Update the residents socialness level
	residentInstance->entertainedness_level_ = newLevel;
	//Create a socialness message to publish
	msg_pkg::Entertainedness entertainednessMessage;
	//Assign current socialness level to the message
	entertainednessMessage.level = newLevel;

	if (newLevel == 5)
	{
		residentInstance->stopRobotSpinning();
		residentInstance->e_replenished_ = true;
	}

	//Publish the message
	residentInstance->publisherEntertainedness.publish(entertainednessMessage);
  }
  // TODO: put others in when implemented
}

int Resident::getNewLevel(int amount, int oldLevel)
{
	int newLevel = std::min(amount + oldLevel, 5); // Can only have a maximum level of 5
	// Code to check it doesn't go below 1... just incase interactions can reduce levels at some point
	if (newLevel < 1)
	{
		newLevel = 1;
	}
	return newLevel;
}

void Resident::stopRobotSpinning()
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 0.0; // Stop rotation to show interaction finished
}