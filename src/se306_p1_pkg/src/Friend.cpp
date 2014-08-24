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

  if (!socialising)
  {
    if (socialnessLevel < 2) // If the socialness level is too low
    {
      if (first_call)
      {
        this->startMovingToResident();
        first_call = false;
      }
      if (!(this->movingToResident) )
      {
        //Relative::doResponse("socialising");
        ROS_INFO("CHANGED TO SOCIALISING");
        socialising=true;
        first = false;
      }
    }
  }
  else
  {
    if (socialnessLevel == 5)
    {
      //Add do last desponse call that kurt implimented
      Friend::stopResponse("socialising");
      socialising = false;
      returningHome = true;
    }
    else
    {
      if (y == 40)
      {
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

void Friend::moraleCallback(msg_pkg::Morale msg)
{
  ROS_DEBUG_STREAM("Friend moraleCallback with level " << (int)msg.level);
}

void Friend::timeCallback(msg_pkg::Time msg)
{
  //ROS_DEBUG_STREAM("Friend timeCallback with time " << (int)msg.hour << ":" << (int)msg.minutes << ":" << (int)msg.seconds);
}
