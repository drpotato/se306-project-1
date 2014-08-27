#include "Friend.h"

Friend* Friend::getFriendInstance()
{
  return dynamic_cast<Friend*>( ActorSpawner::getInstance().getActor());
}

/* Hooks */
void Friend::doInitialSetup()
{
  /* Subscriptions */
  subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Friend::socialnessCallback);
  subscriberMorale = nodeHandle->subscribe("morale", 1000, Friend::moraleCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Friend::timeCallback);
  subscriberLockStatus = nodeHandle->subscribe("lockStatus", 1000, Friend::lockStatusCallback);
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
}

void Friend::doExecuteLoop()
{
  if (returningHome)
  {
    if (returningHome_first)
    {
      returningHome_first = false;
    }
    return;
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
  else if (this->has_lock)
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
}

/* Callbacks */ 
void Friend::socialnessCallback(msg_pkg::Socialness msg)
{
  Friend *temp = Friend::getFriendInstance();
  temp->socialnessLevel = msg.level;
  ROS_DEBUG_STREAM("Friend socialnessCallback with level " << (int)msg.level);
}

void Friend::telephoneCallback(msg_pkg::Telephone msg)
{
  ROS_DEBUG_STREAM("Friend telephoneCallback with contact " << msg.contact);
  Friend *temp = Friend::getFriendInstance();
  
  if (msg.contact.compare("friend") == 0)
  {
    ROS_DEBUG_STREAM("Friend telephoneCallback contact is friend!!!");
    // If it has reached level 1 and resident has called friend should probably socialise with resident
    if (temp->socialnessLevel <= 1)
    {
      temp->startMovingToResident();
      temp->waiting_to_socialise = true;
    }
  }
}

void Friend::moraleCallback(msg_pkg::Morale msg)
{

}

void Friend::timeCallback(msg_pkg::Time msg)
{
  //ROS_DEBUG_STREAM("Friend timeCallback with time " << (int)msg.hour << ":" << (int)msg.minutes << ":" << (int)msg.seconds);
}

void Friend::lockStatusCallback(msg_pkg::LockStatus msg)
{
        Friend* temp = Friend::getFriendInstance();
        if ((msg.has_lock) && (msg.robot_id == temp->rosName))
        {
                ROS_INFO("Friend has the lock");
                ROS_INFO("CHANGED TO SOCIALISING");
                //temp->waiting_to_entertain = false;
                //temp->entertaining=true;
                temp->has_lock = true;
        }
        else if ((!(msg.has_lock)) && (msg.robot_id == temp->rosName))
        {
                ROS_INFO("Friend does not have the lock");
                temp->has_lock = false;
        }
}
