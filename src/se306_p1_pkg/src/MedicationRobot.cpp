#include "MedicationRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

string MedicationRobot::getActorName()
{
  return "MedicationRobot";
}

// A Robot which gives the Resident his medication.
void MedicationRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	healthLevel = 5;
	healing = false;
	active = false;
	subscriberHealth = nodeHandle->subscribe("health", 1000, MedicationRobot::healthCallback);
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
	residentName = "Resident0";
}

void MedicationRobot::doExecuteLoop()
{
	if (RCmode == "medicationRobot") {
    	MedicationRobot::controlRobot();
  	} else if (active) {
  		if (first) {
  			requestLock("MedicationRobot");
			first = false;
  		}

  		if (haveLock) {
  			if (travellingToResident) {
				if(MedicationRobot::goToNode("Resident")) {
					travellingToResident = false;
					healing = true;
				}
			 } else if (returningHome) {
				if(MedicationRobot::goToNode("nodeMedicationRobotHome")) {
					returningHome = false;
					first = true;
					active = false;
				}
			 } else if (healing) {
				if (healthLevel == 100) {
					MedicationRobot::stopResponse("health");
					healing = false;
				} else {
					healthLevel += 10;
					MedicationRobot::doResponse("health");
				}
			}
		} else if (deniedLock) {
			if (otherUnlocked) {
				requestLock("MedicationRobot");
				deniedLock = false;
				otherUnlocked = false;
			}
		}
	}
}

void MedicationRobot::healthCallback(msg_pkg::Health msg)
{
 	MedicationRobot* temp = dynamic_cast<MedicationRobot*>( ActorSpawner::getInstance().getActor());

 	temp->healthLevel = msg.level;

 	if (msg.level < 70)	{
 		// Give medicine
 		temp->travellingToResident = true;
 		temp->active = true;
 	}
}