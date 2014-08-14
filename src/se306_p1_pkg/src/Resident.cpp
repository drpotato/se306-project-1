#include "Resident.h"
#include "msg_pkg/Entertainedness.h"

void Resident::doInitialSetup()
{
  // Initially the resident is not locked
  lock_ = false;
  entertainedness = 5;
  publisherEntertainedness = nodeHandle->advertise<msg_pkg::Entertainedness>("entertainedness", 1000);
}

void Resident::doExecuteLoop()
{
    
}


void Resident::publishEntertainedness()
{
    msg_pkg::Entertainedness entertainednessMessage;
    entertainednessMessage.level = entertainedness;
    
    publisherEntertainedness.publish(entertainednessMessage);
}

void Resident::setEntertainedness(int newLevel)
{

    entertainedness = newLevel;
    //Every time the entertainedness attribute changes, publish a message containing the new level.
    this->publishEntertainedness();
    //Spin in a circle?
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

