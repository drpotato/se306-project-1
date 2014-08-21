#include "Doctor.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"

void Doctor::doInitialSetup()
{
	velLinear = 0.0;
    velRotational = 0.0;
    subscriberHealth = nodeHandle->subscribe("telephone", 1000, Doctor::telephoneCallback);
    homeVisit = false;
}



void Doctor::doExecuteLoop()
{

}

void Doctor::telephoneCallback(msg_pkg::Telephone msg)
{

	//TODO: Make something happen in the if statement
	Doctor* temp = dynamic_cast<Doctor*>( ActorSpawner::getInstance().getActor());
	if (msg.contact == "doctor")
	{
		temp->homeVisit = true;
	}

}

void Doctor::emergency()
{
	ROS_DEBUG("HEARD THAT THERE IS AN EMERGENCY");

}