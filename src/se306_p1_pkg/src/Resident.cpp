#include "Resident.h"
#include <string.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Entertainedness.h>

void Resident::doInitialSetup()
{
  // Initially the resident is not locked
  lock_ = false;

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

  if (attribute == "socialness")
  {
	// Get the class instance
	Resident* temp = dynamic_cast<Resident*>( ActorSpawner::getInstance().getActor("kurt fix this shit"));
	// Update the residents socialness level
	temp->socialness_level_ = max(msg.amount + temp->socialness_level_, 5); // Can only have a maximum level of 5

	//Create a socialness message to publish
	msg_pkg::Socialness socialnessMessage;
	//Assign current socialness level to the message
	socialnessMessage.level = temp->socialness_level_;
	//Publish the message
	publisherSocialness.publish(socialnessMessage);
  } 
  else if (attribute == "entertainedness")
  {
  	//some stuff
  }
	

  	//put others in when implemented
  
}
