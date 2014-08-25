#include "MedicationRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

// A Robot which gives the Resident his medication.
void MedicationRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, MedicationRobot::moraleCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void MedicationRobot::doExecuteLoop()
{
	if (returningHome){

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}

        return;

	}

	if (!entertaining)
	{
		if (!checkMedicationLevel())
		{
			if (first_call)
			{
				//this->startMovingToResident();
				first_call = false;
			}

	    	if (!(this->movingToResident) )
	    	{
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		entertaining=true;
	    		first = false;
	    	}

		}
	}
	else
	{
		if (moraleLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			MedicationRobot::stopResponse("entertaining");
			entertaining = false;
			returningHome = true;

		}
		else
		{

			if (y == 40)
			{
				MedicationRobot::doResponse("entertaining");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}

// TODO: SHOULD BE MEDICATION ##################################################################################################################################
void MedicationRobot::moraleCallback(msg_pkg::Morale msg)
{
 	MedicationRobot* temp = dynamic_cast<MedicationRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
}

bool MedicationRobot::checkMedicationLevel()
{
	if (moraleLevel>=2 )
	{
		return true;
	}
	return false;
}
