#include "Friend.h"

Friend* Friend::getFriendInstance()
{
  return dynamic_cast<Friend*>( ActorSpawner::getInstance().getActor());
}

string Friend::getActorName()
{
  return "Friend";
}

/* Hooks */
void Friend::doInitialSetup()
{
  /* Subscriptions */
  subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Friend::socialnessCallback);
  subscriberMorale = nodeHandle->subscribe("morale", 1000, Friend::moraleCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Friend::timeCallback);
  subscribeTelephone = nodeHandle->subscribe("telephone", 1000, Friend::telephoneCallback);

  velLinear = 0.0;
  velRotational = 0.0;
  socialnessLevel = 5;
  socialising = false;

  y = 0;
  x = 0;
  first = true;
  first_call = true;
  returningHome = false;
  returningHome_first = true;

  waiting_to_socialise = false;
  called_by_resident_ = false;
}

void Friend::doExecuteLoop()
{

  if (RCmode == "Friend1")
    {
      Friend::controlRobot();
        return;
    }
  if (returningHome)
  {
    if (returningHome_first)
    {
      returningHome_first = false;
    }
    return;
  }

  // If socialness is CRITICAL_LEVEL and has been called by resident should probably socialise with resident
  if (socialnessLevel <= CRITICAL_LEVEL && called_by_resident_)
  {
    //TODO: go to resident
    waiting_to_socialise = true;
    called_by_resident_ = false;
  }

  // If we have finished moving to the resident and we need to socialise:
  if ((!(this->movingToResident)) && (waiting_to_socialise) && first)
  {
    // Request the lock
    ROS_INFO("Requesting lock...");
    first = false;
    Friend::requestLock("Friend");
  }
  // If it has the lock:
  else if (haveLock)
  {
    // If it has reached the maximum level
    if (socialnessLevel == 5)
    {
      // Stop socialising and unlock the resident
      Friend::stopResponse("socialising");
      Friend::unlock();
      socialising = false;
      first = false;
      returningHome = true;
    }
    else
    {
      if (y == 40)
      {
        // Called ever 40 cycles
        Friend::doResponse("socialising");
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
      Friend::requestLock("Visitor");
      //Set back to false so only requests again once
      deniedLock = false;
      otherUnlocked = false;
    }
  }
}

/* Callbacks */
void Friend::socialnessCallback(msg_pkg::Socialness msg)
{
  Friend *temp = Friend::getFriendInstance();
  temp->socialnessLevel = msg.level;

}

void Friend::telephoneCallback(msg_pkg::Telephone msg)
{

  Friend *temp = Friend::getFriendInstance();

  if (msg.contact.compare("friend") == 0)
  {
    
    temp->called_by_resident_ = true;
  }
}

void Friend::moraleCallback(msg_pkg::Morale msg)
{

}

void Friend::timeCallback(msg_pkg::Time msg)
{
  //ROS_DEBUG_STREAM("Friend timeCallback with time " << (int)msg.hour << ":" << (int)msg.minutes << ":" << (int)msg.seconds);
}
