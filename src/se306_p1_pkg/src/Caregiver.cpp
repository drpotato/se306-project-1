#include "Caregiver.h"
#include "ActorSpawner.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"


void Caregiver::doInitialSetup()
{

}


void Caregiver::doExecuteLoop()
{

}


void Caregiver::caring()
{
    if (returningHome)
    {
        if (returningHome_first)
        {
            returningHome_first = false;
        }
        return;
    }

    if (!feeding)
    {
        if (!checkHungerLevel())
        {
            if (first_call)
            {
                this->startMovingToResident();
                first_call = false;
            }
            if (!(this->movingToResident) )
            {
                feeding=true;
                first = false;
            }
           //After finished entertaining set entertaining to flase
        }
    }
    else if (!bathing)
    {
        if (!checkHygieneLevel())
        {
            if (first_call)
            {
                this->startMovingToResident();
                first_call = false;
            }
            if (!(this->movingToResident) )
            {
                bathing=true;
                first = false;
            }
           //After finished entertaining set entertaining to flase
        }

    }
    else if (!exercising)
    {
        if (!checkFitnessLevel())
        {
            if (first_call)
            {
                this->startMovingToResident();
                first_call = false;
            }
            if (!(this->movingToResident) )
            {
                exercising=true;
                first = false;
            }
           //After finished entertaining set entertaining to flase
        }

    }
    else if (!talking)
    {
        if (!checkSocialLevel())
        {
            if (first_call)
            {
                this->startMovingToResident();
                first_call = false;
            }
            if (!(this->movingToResident) )
            {
                talking=true;
                first = false;
            }
           //After finished entertaining set entertaining to flase
        }

    }
    else if (!moralesupporting)
    {
        if (!checkMoraleLevel())
        {
            if (first_call)
            {
                this->startMovingToResident();
                first_call = false;
            }
            if (!(this->movingToResident) )
            {
                moralesupporting=true;
                first = false;
            }
           //After finished entertaining set entertaining to flase
        }

    }
    else
    {
        if (feeding)
        {
            if (hungerLevel == 5)
            {
                //Add do last desponse call that kurt implimented
                stopResponse("hunger");
                hungerLevel = false;
                returningHome = true;
            }
            else
            {
                if (y1 == 40)
                {
                    doResponse("hunger");
                    y1 = 0;
                }
                else
                {
                    y1++;
                }
            }
        }
        else if (bathing)
        {
            if (hygieneLevel == 5)
            {
                //Add do last desponse call that kurt implimented
                stopResponse("hygiene");
                hygieneLevel = false;
                returningHome = true;
            }
            else
            {
                if (y2 == 40)
                {
                    doResponse("hygiene");
                    y2 = 0;
                }
                else
                {
                    y2++;
                }
            }
        }
        else if (exercising)
        {
            if (fitnessLevel == 5)
            {
                //Add do last desponse call that kurt implimented
                stopResponse("fitness");
                fitnessLevel = false;
                returningHome = true;
            }
            else
            {
                if (y3 == 40)
                {
                    doResponse("fitness");
                    y3 = 0;
                }
                else
                {
                    y3++;
                }
            }
        }
        else if (talking)
        {
            if (socialnessLevel == 5)
            {
                //Add do last desponse call that kurt implimented
                stopResponse("socialness");
                socialnessLevel = false;
                returningHome = true;
            }
            else
            {
                if (y4 == 40)
                {
                    doResponse("socialness");
                    y4 = 0;
                }
                else
                {
                    y4++;
                }
            }
        }
        else if (moralesupporting)
        {
            if (moraleLevel == 5)
            {
                //Add do last desponse call that kurt implimented
                stopResponse("morale");
                moraleLevel = false;
                returningHome = true;
            }
            else
            {
                if (y5 == 40)
                {
                    doResponse("morale");
                    y5 = 0;
                }
                else
                {
                    y5++;
                }
            }
        }

    }
}
/*
void Caregiver::fitnessCallback(msg_pkg::Fitness msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

    temp->fitnessLevel = msg.level;
}


void Caregiver::hungerCallback(msg_pkg::Hunger msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

    temp->hungerLevel = msg.level;
}


void Caregiver::hygieneCallback(msg_pkg::Hygiene msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

    temp->hygieneLevel = msg.level;
}


void Caregiver::moraleCallback(msg_pkg::Morale msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

    temp->moraleLevel = msg.level;
}


void Caregiver::socialnessCallback(msg_pkg::Socialness msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

    temp->socialnessLevel = msg.level;
}


void Caregiver::timeCallback(msg_pkg::Time msg)
{
    Caregiver* temp = dynamic_cast<Caregiver*>( ActorSpawner::getInstance().getActor());

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
*/

bool Caregiver::checkFitnessLevel()
{
    if (fitnessLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver::checkHungerLevel()
{
    if (hungerLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver::checkHygieneLevel()
{
    if (hygieneLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver::checkMoraleLevel()
{
    if (moraleLevel >= 2)
    {
        return true;
    }
    return false;
}


bool Caregiver::checkSocialLevel()
{
    if (socialnessLevel >= 2)
    {
        return true;
    }
    return false;
}
