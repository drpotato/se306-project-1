#include "Caregiver.h"
#include <stdlib.h>
#include <time.h>
#include <string>

#include "PathPlanner.h"
#include "PathPlannerNode.h"

string Caregiver::getActorName()
{
  return "Caregiver";
}

void Caregiver::doInitialSetup()
{
  velLinear = 0.0;
  velRotational = 0.0;
  nodename = ros::this_node::getName();

  std::size_t found11 = nodename.find_first_of("11");
  std::size_t found12 = nodename.find_first_of("12");

  if (found11 != std::string::npos)
  {
    caregiverId = 1;
  }
  else if (found12 != std::string::npos)
  {
    caregiverId = 2;
  }
  else
  {
    //YOU BROKE THE BUILD!
    ROS_INFO("THIS IS BROKEN! CAREGIVERS SHOULD BE KEPT TO ROSNODE12 AND ROSNODE13");
    ROS_INFO("MY NAME IS %s", nodename.c_str());
    caregiverId = 0;
    exit(EXIT_FAILURE);
  }

  subscriberFitness = nodeHandle->subscribe("fiteness", 1000, Caregiver::fitnessCallback);
  subscriberHunger = nodeHandle->subscribe("hunger", 1000, Caregiver::hungerCallback);
  subscriberHygiene = nodeHandle->subscribe("hygiene", 1000, Caregiver::hygieneCallback);
  subscriberMorale = nodeHandle->subscribe("morale", 1000, Caregiver::moraleCallback);
  subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Caregiver::socialnessCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Caregiver::timeCallback);
  
  homeVisit = false;
  movingToResident = false;

  hour = 0;
  y = 0;

  fitnessLevel = LEVEL_MAX;
  hungerLevel = LEVEL_MAX;
  hygieneLevel = LEVEL_MAX;
  moraleLevel = LEVEL_MAX;
  socialnessLevel = LEVEL_MAX;

  exercising = false;
  eating = false;
  showering = false;
  entertaining = false;
  socialising = false;
  caring = 0;

  first = true;
}

void Caregiver::doExecuteLoop()
{
    if(caregiverId == 1)
    {
       if (RCmode == "caregiver1")
        {
            controlRobot();
            return;
        } 
        if (odd)
        {
            if ((hour > 6 && hour < 22) || (caring))
            {
              if (!homeVisit)
              {
                homeVisit = true;
                movingToResident = true;
                // Move to resident            
              }
              
              if (!goToNode("Resident0"))
              {
                lookafter();
              }
            }
        }
    }

  else if(caregiverId == 2)
  {
     if (RCmode == "caregiver2")
      {
          controlRobot();
          return;
      } 
      if (!odd)
      {
          if ((hour > 6 && hour < 22) || (caring))
          {
            if (!homeVisit)
            {
              homeVisit = true;
              movingToResident = true;
              // Move to resident            
            }
            
            if (!goToNode("Resident0"))
            {
              lookafter();
            }
          }
      }
  }
  else
    {
      if (homeVisit)
      {
        homeVisit = false;
        // Back to initial location
      }
    }
}

void Caregiver::fitnessCallback(msg_pkg::Fitness msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp->fitnessLevel = msg.level;
  if (!temp->checkFitnessLevel())
  {
    temp->exercising = true;
  }
}

void Caregiver::hungerCallback(msg_pkg::Hunger msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp->hungerLevel = msg.level;
}

void Caregiver::hygieneCallback(msg_pkg::Hygiene msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp->hygieneLevel = msg.level;
  if (!temp->checkHygieneLevel())
  {
    temp->showering = true;
  }
}

void Caregiver::moraleCallback(msg_pkg::Morale msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp->moraleLevel = msg.level;
  if (!temp->checkMoraleLevel())
  {
    temp->entertaining = true;
  }
}

void Caregiver::socialnessCallback(msg_pkg::Socialness msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp->socialnessLevel = msg.level;
  if (!temp->checkSocialnessLevel())
  {
    temp->socialising = true;
  }
}

void Caregiver::timeCallback(msg_pkg::Time msg)
{
  Caregiver* temp = dynamic_cast<Caregiver*> (ActorSpawner::getInstance().getActor());
  temp-> hour = msg.hour;
  if ((msg.day % 2) != 0)
  {
    temp->odd = true;
  }
  else
  {
    temp->odd = false;
  }
  if ((msg.hour == temp->LUNCH_TIME) || (msg.hour == temp->BREAKFAST_TIME) || (msg.hour == temp->DINNER_TIME))
  {
    temp->eating = true;
  }
}

bool Caregiver::checkFitnessLevel()
{
  if (fitnessLevel >= CRITICAL_LEVEL)
  {
    return true;
  }
  return false;
}

bool Caregiver::checkHygieneLevel()
{
  if (hygieneLevel >= CRITICAL_LEVEL)
  {
    return true;
  }
  return false;
}

bool Caregiver::checkMoraleLevel()
{
  if (moraleLevel >= CRITICAL_LEVEL)
  {
    return true;
  }
  return false;
}
bool Caregiver::checkSocialnessLevel()
{
  if (socialnessLevel >= CRITICAL_LEVEL)
  {
    return true;
  }
  return false;
}

void Caregiver::lookafter()
{
  if (caring == 0)
  {
    if (eating)
    {
      caring = 1;
    }
    else if (showering)
    {
      caring = 2;
    }
    else if (exercising)
    {
      caring = 3;
    }
    else if (entertaining)
    {
      caring = 4;
    }
    else if (socialising)
    {
      caring = 5;
    }
  }
  else if (caring == 1)
  {
    if (first)
    {
      requestLock("Caregiver");
      first = false;
    }
    if (haveLock)
    {
      if (!goToNode("nodeLivingRoomFeedingPlace"))
      {
        if (hungerLevel >= REASONABLE_LEVEL)
        {
          stopResponse("eating");
          eating = false;
          caring = 0;
          goToNode("Resident0");
        }
        else
        {
          if (y == 40)
          {
            doResponse("eating");
            y = 0;
          }
          else
          {
            y ++;
          }
        }
      }
    }
    else if (deniedLock)
    {
      if (otherUnlocked)
      {
        requestLock("Caregiver");
        deniedLock = false;
        otherUnlocked = false;
      }
    }
  }
  else if (caring == 2)
  {
    if (first)
    {
      requestLock("Caregiver");
      first = false;
    }
    if (haveLock)
    {
      if (!goToNode("nodeBathroomDoorInBathroom"))
      {
        if (hygieneLevel >= REASONABLE_LEVEL)
        {
          stopResponse("showering");
          showering = false;
          caring = 0;
          goToNode("Resident0");
        }
        else
        {
          if (y == 40)
          {
            doResponse("showering");
            y = 0;
            // Showering
          }
          else
          {
            y ++;
          }
        }
      }
    }
    else if (deniedLock)
    {
      if (otherUnlocked)
      {
        requestLock("Caregiver");
        deniedLock = false;
        otherUnlocked = false;
      }
    }
  }
  else if (caring == 3)
  {
    if (first)
    {
      requestLock("Caregiver");
      first = false;
    }
    if (haveLock)
    {
      if (fitnessLevel >= REASONABLE_LEVEL)
      {
        stopResponse("exercising");
        exercising = false;
        caring = 0;
      }
      else
      {
        if (y == 40)
        {
          doResponse("exercising");
          y = 0;
        }
        else
        {
          y ++;
        }
      }
    }
    else if (deniedLock)
    {
      if (otherUnlocked)
      {
        requestLock("Caregiver");
        deniedLock = false;
        otherUnlocked = false;
      }
    }
  }
  else if (caring == 4)
  {
    if (first)
    {
      requestLock("Caregiver");
      first = false;
    }
    if (haveLock)
    {
      if (moraleLevel >= REASONABLE_LEVEL)
      {
        stopResponse("entertaining");
        entertaining = false;
        caring = 0;
      }
      else
      {
        if (y == 40)
        {
          doResponse("entertaining");
          y = 0;
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
        requestLock("Caregiver");
        deniedLock = false;
        otherUnlocked = false;
      }
    }
  }
  else if (caring == 5)
  {
    if (first)
    {
      requestLock("Caregiver");
      first = false;
    }
    if (haveLock)
    {
      if (socialnessLevel >= REASONABLE_LEVEL)
      {
        stopResponse("socialising");
        socialising = false;
        caring = 0;
      }
      else
      {
        if (y == 40)
        {
          doResponse("socialising");
          y = 0;
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
        requestLock("Caregiver");
        deniedLock = false;
        otherUnlocked = false;
      }
    }
  }
}
