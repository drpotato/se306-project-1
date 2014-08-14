#include "EntertainmentRobot.h"
#include "ActorSpawner.h"



void EntertainmentRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	entertainednessLevel = 5;
	entertaining = false;
	subscriberEntertainedness = nodeHandle->subscribe("entertainedness", 1000, EntertainmentRobot::entertainednessCallback);	

}

void EntertainmentRobot::doExecuteLoop()
{
	ROS_INFO("Value %i", entertainednessLevel);
	if (checkEntertainmentLevel())
	{
		ROS_INFO("Nothing to do here");
	} else {
		entertaining = true;
		//Call method to do the entertaining
		//After finished entertaining set entertaining to flase
		ROS_INFO("My master needs me!!");
	}
}


void EntertainmentRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor("kurt fix this shit"));

 	temp->entertainednessLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool EntertainmentRobot::checkEntertainmentLevel()
{
	if (entertainednessLevel>2 | entertaining){
		return true;
	}
	return false;
}