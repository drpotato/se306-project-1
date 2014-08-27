#include "Caregiver1.h"
#include "ActorSpawner.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"


void Caregiver1::doInitialSetup()
{
    velLinear = 0.0;
    velRotational = 0.0;

    fitnessLevel = 5;
    hungerLevel = 5;
    hygieneLevel = 5;
    moraleLevel = 5;
    socialnessLevel = 5;

    exercising = false;
    feeding = false;
    bathing = false;
    moralesupporting = false;
    talking = false;

    subscriberFitness = nodeHandle->subscribe("fitness", 1000, Caregiver1::fitnessCallback);
    subscriberHunger = nodeHandle->subscribe("hunger", 1000, Caregiver1::hungerCallback);
    subscriberHygiene = nodeHandle->subscribe("hygiene", 1000, Caregiver1::hygieneCallback);
    subscriberMorale = nodeHandle->subscribe("Morale", 1000, Caregiver1::moraleCallback);
    subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Caregiver1::socialnessCallback);
    subscriberTime = nodeHandle->subscribe("time", 1000, Caregiver1::timeCallback);

    caregiverName = "RobotNode 8";
    y1 = 0;
    y2 = 0;
    y3 = 0;
    y4 = 0;
    y5 = 0;
    x = 0;
    first = true;
    first_call = true;
    returningHome = false;
    returningHome_first = true;
    odd = true;
}


void Caregiver1::doExecuteLoop()
{
    if (odd)
    {
    	caring();
    }
}


void Caregiver1::fitnessCallback(msg_pkg::Fitness msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->fitnessLevel = msg.level;
}


void Caregiver1::hungerCallback(msg_pkg::Hunger msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->hungerLevel = msg.level;
}


void Caregiver1::hygieneCallback(msg_pkg::Hygiene msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->hygieneLevel = msg.level;
}


void Caregiver1::moraleCallback(msg_pkg::Morale msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->moraleLevel = msg.level;
}


void Caregiver1::socialnessCallback(msg_pkg::Socialness msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->socialnessLevel = msg.level;
}


void Caregiver1::timeCallback(msg_pkg::Time msg)
{
    Caregiver1* temp = dynamic_cast<Caregiver1*>( ActorSpawner::getInstance().getActor());

    temp->hour = msg.hour;
    
    if (msg.day % 2 != 0)
    {
        temp->odd = true;
    }
    else 
    {
        temp->odd = false;
    }
    //ROS_INFO("%s", (temp->odd == true)?"true":"false");
}

/*
bool Caregiver1::checkFitnessLevel()
{
    if (fitnessLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver1::checkHungerLevel()
{
    if (hungerLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver1::checkHygieneLevel()
{
    if (hygieneLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver1::checkMoraleLevel()
{
    if (moraleLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver1::checkSocialLevel()
{
    if (socialnessLevel >= 2)
    {
        return true;
    }
    return false;
}
*/
