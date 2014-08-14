#include "Resident.h"
#include <string.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Entertainedness.h>
#include "Actor.h"
#include "ActorSpawner.h"

void Resident::doInitialSetup()
{
  // Initially the resident is not locked
  lock_ = false;

  // Set levels to maximum initially
  entertainedness_level_ = 5;
  socialness_level_ = 5;

  // Set up a publishers
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherEntertainedness = nodeHandle->advertise<msg_pkg::Entertainedness>("entertainedness", 1000);

  // Set up subscribers
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
}

void Resident::doExecuteLoop()
{
    
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
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor("kurt fix this shit"));

  if (attribute == "socialness")
  {
  	// Get new level
  	int newLevel = getNewLevel(amount, residentInstance->socialness_level_);

	// Update the residents socialness level
	residentInstance->socialness_level_ = newLevel;

	//Create a socialness message to publish
	msg_pkg::Socialness socialnessMessage;
	//Assign current socialness level to the message
	socialnessMessage.level = newLevel;
	//Publish the message
	residentInstance->publisherSocialness.publish(socialnessMessage);
  } 
  else if (attribute == "entertainedness")
  {
	// Get new level
	int newLevel = getNewLevel(amount, residentInstance->entertainedness_level_);

	// Update the residents socialness level
	residentInstance->entertainedness_level_ = newLevel;

	//Create a socialness message to publish
	msg_pkg::Entertainedness entertainednessMessage;
	//Assign current socialness level to the message
	entertainednessMessage.level = newLevel;
	//Publish the message
	residentInstance->publisherEntertainedness.publish(entertainednessMessage);
  }
	

  	//put others in when implemented
  
}

int getNewLevel(int amount, int oldLevel)
{
	int newLevel = std::min(amount + oldLevel, 5); // Can only have a maximum level of 5
	// Code to check it doesn't go below 1... just incase interactions can reduce levels at some point
	if (newLevel < 1)
	{
		newLevel = 1;
	}
	return newLevel;
}

void Resident::setEntertainedness(int newLevel)
{
    entertainedness_level_ = newLevel;
    //Every time the entertainedness attribute changes, publish a message containing the new level.
    this->publishEntertainedness();
    //Spin in a circle?
}

void Resident::publishEntertainedness()
{
    msg_pkg::Entertainedness entertainednessMessage;
    entertainednessMessage.level = entertainedness_level_;
    
    publisherEntertainedness.publish(entertainednessMessage);
}