#include "Doctor.h"

#include "PathPlanner.h"
#include "PathPlannerNode.h"

void Doctor::doInitialSetup()
{
	velLinear = 0.0;
    velRotational = 0.0;
    subscriberHealth = nodeHandle->subscribe("socialness", 1000, Doctor::healthCallback);
}



void Doctor::doExecuteLoop()
{

}

void Doctor::healthCallback(msg_pkg::Health msg)
{

	Doctor* temp = dynamic_cast<Doctor*>( ActorSpawner::getInstance().getActor());
	temp->healthLevel = msg.level;

	if (msg.level < 2){
		temp->emergency();
	}

}

void Doctor::emergency()
{
	ROS_DEBUG("HEARD THAT THERE IS AN EMERGENCY");

}