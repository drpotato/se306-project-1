#include "Resident.h"
#include <string.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include "Actor.h"
#include "ActorSpawner.h"
#include <ctime>
#include <time.h>

// The person living in our house. 
// Has various attributes representing his needs/wants, which degrade over time.
// When they reach a certain level, messages are published to his assistant Robots and the VisitorController, requesting various services.
void Resident::doInitialSetup()
{
  velLinear = 0;
  velRotational = 0.0;

  // Initially the resident is not locked.
  // Any other Actor can interact with him and acquire the lock.
  lock_ = false;

  // Initialise time of day to current time
  time_of_day = std::time(0);
  // Get number of seconds to add on each loop
  seconds_to_add = second_increase_per_loop();

  // Set levels to maximum initially.
  morale_level_ = 5;
  socialness_level_ = 5;

  morale_count_ = 0;
  socialness_count_ = 0;
  m_dropped_ = false;
  m_replenished_ = false;

  // Set up publishers.
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherMorale = nodeHandle->advertise<msg_pkg::Morale>("morale", 1000);

  // Set up subscriptions.
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
}

void Resident::doExecuteLoop()
{    
	// Increment the time of day
	//time_of_day += 12;
	time_of_day += 3600;
	//ROS_INFO("%s", ctime(&time_of_day));

	time_of_day_values = gmtime(&time_of_day);
	//ROS_INFO("%d", time_of_day_values->tm_hour);

	if (morale_count_ >= WAIT_TIME && !m_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->morale_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			m_dropped_ = true;
		}
		else 
		{
			// Reduce the level every 1000 counts
			residentInstance->morale_level_--;
			// Create a socialness message to publish
			msg_pkg::Morale moraleMessage;
			// Assign current socialness level to the message
			moraleMessage.level = residentInstance->morale_level_;
			// Publish the message
			residentInstance->publisherMorale.publish(moraleMessage);
		}
		morale_count_ = 0;
	}
	else if (morale_count_ < WAIT_TIME && !m_dropped_) {
		morale_count_++;
	}

	else if (m_replenished_ && (socialness_count_ >= WAIT_TIME) && !s_dropped_) {
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->socialness_level_ <= 1) {
			// don't drop the value any more, it's being tended to or has been already
			s_dropped_ = true;
		}

		else {
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
	else if (m_replenished_ && (socialness_count_ < WAIT_TIME) && !s_dropped_)
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
 * Robots should unlock the resident after interation so that another robot may interact with the resident.
 */
void Resident::unlock()
{
  lock_ = false;
}

/*
 * Upon receiving a message published to the 'interaction' topic, respond appropriately.
 */
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
	int newLevel = getNewLevel(amount, residentInstance->morale_level_);
	// Update the residents socialness level
	residentInstance->morale_level_ = newLevel;
	//Create a socialness message to publish
	msg_pkg::Morale moraleMessage;
	//Assign current socialness level to the message
	moraleMessage.level = newLevel;

	if (newLevel == 5)
	{
		residentInstance->stopRobotSpinning();
		residentInstance->m_replenished_ = true;
	}

	//Publish the message
	residentInstance->publisherMorale.publish(moraleMessage);
  }
  // TODO: put others in when implemented ##################################################################################################################
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

int Resident::second_increase_per_loop()
{
	Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
	int loop_rate = residentInstance->LOOP_RATE;

	// 1 hr (Ultron world) = 30 seconds (real world)
	// Number of loops needed to pass 30 seconds in the real world: (loop_rate is number loops per second)
	double num_loops_for_real_30_seconds = loop_rate * 30; //e.g. 300

	// Number of loops needed to pass one minute in the Ultron world
	double num_loops_per_ultron_minute = num_loops_for_real_30_seconds / 60; //e.g. 5

	// Number of seconds to add to the time_of_day on each loop in order to pass one Ultron hour in real world 30 seconds
	double num_ultron_seconds_per_loop = 60 / num_loops_per_ultron_minute; //e.g. 12

	return num_ultron_seconds_per_loop;
}