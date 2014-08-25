#include "EntertainmentRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

// A Robot that provides the Resident with entertainment (possibly TV)
void EntertainmentRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, EntertainmentRobot::moraleCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void EntertainmentRobot::doExecuteLoop()
{
	if (returningHome){
		//ROS_INFO("MOVEING TO HOME");

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}
        
        return;

	}

	if (!entertaining)
	{
		if (!checkMoraleLevel())
		{
			if (first_call)
			{
				//this->activeNode = &node5;
                this->goToNode("Resident");
				first_call = false;
			}

	    	if (!(true) )
	    	{
	    		//EntertainmentRobot::doResponse("entertaining");
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		entertaining=true;
	    		first = false;
	    	}

			//After finished entertaining set entertaining to flase

		}
	} 
	else 
	{
		if (moraleLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			EntertainmentRobot::stopResponse("entertaining");
			entertaining = false;
			returningHome = true;

		} 
		else
		{

			if (y == 40)
			{
				EntertainmentRobot::doResponse("entertaining");
				y=0;
			} 
			else 
			{
				y++;
			}	
		}
	}
}


// Upon receiving a message published to the 'entertainedness' topic, respond appropriately.
void EntertainmentRobot::moraleCallback(msg_pkg::Morale msg)
{
 	EntertainmentRobot* temp = dynamic_cast<EntertainmentRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool EntertainmentRobot::checkMoraleLevel()
{
	if (moraleLevel>=2 )
	{
		return true;
	}
	return false;
}
