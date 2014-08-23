#include <stdlib.h>
#include <time.h>
#include <msg_pkg/Nurse.h>
#include "Doctor.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"

void Doctor::doInitialSetup()
{
	velLinear = 0.0;
    velRotational = 0.0;
    srand(time(NULL));
    subscriberTelephone = nodeHandle->subscribe("telephone", 1000, Doctor::telephoneCallback);
    homeVisit = false;
    
    // Set up publishers.
 	publisherNurse1 = nodeHandle->advertise<msg_pkg::Nurse>("nurse1", 1000);
 	publisherNurse2 = nodeHandle->advertise<msg_pkg::Nurse>("nurse2", 1000);
    
}



void Doctor::doExecuteLoop()
{
	if (homeVisit)
	{
		attendPatient();
	}

}

void Doctor::telephoneCallback(msg_pkg::Telephone msg)
{

	//TODO: Make something happen in the if statement
	Doctor* temp = dynamic_cast<Doctor*>( ActorSpawner::getInstance().getActor());
	if (msg.contact == "doctor" && !temp->homeVisit)
	{
		temp->homeVisit = true;
		temp->callNurses();
	}

}

void Doctor::emergency()
{
	ROS_DEBUG("TRANSMITTING THERE IS AN EMERGENCY");

}

void Doctor::attendPatient()
{

}

void Doctor::callNurses()
{
	float num = rand() % 3;
    if (num < 1){
    	return;
    } else if (num > 2){
    	//TODO: Call all nurses
    } else {
    	//TODO: Call one Nurse
    }

	ROS_DEBUG("Called Nurse");

}