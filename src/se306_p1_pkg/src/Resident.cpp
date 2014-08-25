#include "Resident.h"
#include <string.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/RequestLock.h>
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

  // Initialise statuses
  has_eaten_breakfast_ = false;
  has_eaten_lunch_ = false;
  has_eaten_dinner_ = false;
  has_woken_ = hasWoken();
  has_gone_to_bed_ = !hasWoken();

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
  publisherLockStatus = nodeHandle->advertise<msg_pkg::LockStatus>("lockStatus", 1000);

  // Set up subscriptions.
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Resident::timeCallback);
  subscriberRequestLock = nodeHandle->subscribe("requestLock", 1000, Resident::requestLockCallback);
}

void Resident::doExecuteLoop()
{    

  //TODO: REMOVE THIS WHEN RANDOMNESS AND DAY LOGIC IS IMPLEMENTED##################################################################################
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
  //###################################################################################################################################################
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
void Resident::lock(ActorType type)
{
  lock_ = true;
  lock_type_ = type;
}

/*
 * Robots should unlock the resident after interation so that another robot may interact with the resident.
 */
void Resident::unlock()
{
  lock_ = false;
  lock_type_ = NULL;
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

void Resident::timeCallback(msg_pkg::Time msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

  // Check if its currently any event times
  if ((msg.hour == residentInstance->WAKE_TIME) && (!residentInstance->has_woken_))
  {
    // WAKE THE FK UP
    ROS_INFO("It is %d:00. Wake up!", msg.hour);
    residentInstance->wakeUp();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
  else if ( ((msg.hour == residentInstance->BREAKFAST_TIME) && (!residentInstance->has_eaten_breakfast_)) || ((msg.hour == residentInstance->LUNCH_TIME) && (!residentInstance->has_eaten_lunch_)) || ((msg.hour == residentInstance->DINNER_TIME) && (!residentInstance->has_eaten_dinner_)))
  {
    // Here have some food
    ROS_INFO("It is %d:00 - time to eat!", msg.hour);
    residentInstance->eat(msg.hour);
  }
  else if ((msg.hour == residentInstance->SLEEP_TIME) && (!residentInstance->has_gone_to_bed_))
  {
    // Go to sleep yo
    ROS_INFO("It is %d:00. Sleep time!", msg.hour);
    residentInstance->goToSleep();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
}

void Resident::requestLockCallback(msg_pkg::RequestLock msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  if (residentInstance->isLocked())
  {
    ActorType type = residentInstance->getActorTypeFromString(msg.actor_name);
    if ((!lock_type_) && (type > lock_type_))
    {
      // The robot requesting the lock has a higher priority than the current one. You can have the lock!
      
    }
    else
    {
      // The robot with the lock has the same or higher priority than the one requesting it, you cannot have the lock m8

    }
  }
  else
  {
    residentInstance->lock(residentInstance->getActorTypeFromString(msg.actor_name));
  }
}

int Resident::getNewLevel(int amount, int oldLevel)
{
  int newLevel = std::min(amount + oldLevel, 5); // Can only have a maximum level of 5
  return newLevel < 1 ? 1 : newLevel; // Should not be below 1
}

void Resident::stopRobotSpinning()
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 0.0; // Stop rotation to show interaction finished
}

void Resident::wakeUp()
{
  // Reset sleep value for the day
  has_gone_to_bed_ = false;

  has_woken_ = true;
}
void Resident::eat(int hour)
{
  if (hour == BREAKFAST_TIME)
  {
    has_eaten_breakfast_ = true;
  }
  else if (hour == LUNCH_TIME)
  {
    has_eaten_lunch_ = true;
  }
    else if (hour == DINNER_TIME)
  {
    has_eaten_dinner_ = true;
  }
}
void Resident::goToSleep()
{
  // Reset values for next day
  has_eaten_breakfast_ = false;
  has_eaten_breakfast_ = false;
  has_eaten_lunch_ = false;
  has_eaten_dinner_ = false;
  has_woken_ = false;

  has_gone_to_bed_ = true;
}
bool Resident::hasWoken()
{
  std::time_t time_of_day = std::time(0);
  int hour = gmtime(&time_of_day)->tm_hour;
  // Hours are inverted by 12 hours, so an hour value of 11 corresponds to 11pm while an hour value of 23 corresponds to 11am
  if ((hour < 11) && (hour > 6))
  {
    return true;
  }
  return false;
}
Resident::ActorType Resident::getActorTypeFromString(string actorType)
{
  if (actorType == "Doctor")
  {
    return Doctor;
  }
  else if (actorType == "Nurse")
  {
    return Nurse;
  }
  else if (actorType == "Caregiver")
  {
    return Caregiver;
  }
  else if (actorType == "Visitor")
  {
    return Visitor;
  }
  else if (actorType == "Robot")
  {
    return Robot;
  }
}
string Resident::getStringFromActorType(ActorType actorType)
{
  if (actorType == Doctor)
  {
    return "Doctor";
  }
  else if (actorType == Nurse)
  {
    return "Nurse";
  }
  else if (actorType == Caregiver)
  {
    return "Caregiver";
  }
  else if (actorType == Visitor)
  {
    return "Visitor";
  }
  else if (actorType == Robot)
  {
    return "Robot";
  }
}