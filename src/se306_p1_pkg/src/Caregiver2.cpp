#include "Caregiver2.h"
#include "ActorSpawner.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"


void Caregiver2::doInitialSetup()
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

    subscriberFitness = nodeHandle->subscribe("fitness", 1000, Caregiver2::fitnessCallback);
    subscriberHunger = nodeHandle->subscribe("hunger", 1000, Caregiver2::hungerCallback);
    subscriberHygiene = nodeHandle->subscribe("hygiene", 1000, Caregiver2::hygieneCallback);
    subscriberMorale = nodeHandle->subscribe("Morale", 1000, Caregiver2::moraleCallback);
    subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Caregiver2::socialnessCallback);
    subscriberTime = nodeHandle->subscribe("time", 1000, Caregiver2::timeCallback);

    caregiverName = "RobotNode 9";
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


void Caregiver2::doExecuteLoop()
{
    if (!odd)
    {
        caring();
    }
}


void Caregiver2::fitnessCallback(msg_pkg::Fitness msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->fitnessLevel = msg.level;
}


void Caregiver2::hungerCallback(msg_pkg::Hunger msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->hungerLevel = msg.level;
}


void Caregiver2::hygieneCallback(msg_pkg::Hygiene msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->hygieneLevel = msg.level;
}


void Caregiver2::moraleCallback(msg_pkg::Morale msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->moraleLevel = msg.level;
}


void Caregiver2::socialnessCallback(msg_pkg::Socialness msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->socialnessLevel = msg.level;
}


void Caregiver2::timeCallback(msg_pkg::Time msg)
{
    Caregiver2* temp = dynamic_cast<Caregiver2*>( ActorSpawner::getInstance().getActor());

    temp->hour = msg.hour;

    if (msg.day % 2 != 0)
    {
        temp->odd = true;
    }
    else
    {
        temp->odd = false;
    }
}

/*
bool Caregiver2::checkFitnessLevel()
{
    if (fitnessLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver2::checkHungerLevel()
{
    if (hungerLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver2::checkHygieneLevel()
{
    if (hygieneLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver2::checkMoraleLevel()
{
    if (moraleLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver2::checkSocialLevel()
{
    if (socialnessLevel >= 2)
    {
        return true;
    }
    return false;
}
*/
