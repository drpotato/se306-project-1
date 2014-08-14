#include "EntertainmentRobot.h"



void EntertainmentRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;

	subscriberEntertainedness = nodeHandle->subscribe("entertainedness", 1000, EntertainmentRobot::entertainednessCallback);	

}

void EntertainmentRobot::doExecuteLoop()
{
	
}


void EntertainmentRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 
}