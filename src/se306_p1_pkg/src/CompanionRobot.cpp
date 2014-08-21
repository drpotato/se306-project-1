#include "CompanionRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"

// A robot who provides the Resident with companionship (could be a robotic dog, a video call system, sex bot)
void CompanionRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	moraleLevel = 5;
	giving_morale = false;
	residentName = "RobotNode2";
	subscriberMorale = nodeHandle->subscribe("morale", 1000, CompanionRobot::moraleCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void CompanionRobot::doExecuteLoop()
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

	if (!giving_morale)
	{
		if (!checkMoraleLevel())
		{
			if (first_call)
			{
				//this->activeNode = &node5;
				this->startMovingToResident();
				first_call = false;
			}
	    	if (!(this->movingToResident) )
	    	{
	    		//CompanionRobot::doResponse("socialising");
	    		ROS_INFO("CHANGED TO socialising");
	    		giving_morale=true;
	    		first = false;
	    	}

			//After finished socialising set socialising to flase

		}
	}
	else
	{
		if (moraleLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			CompanionRobot::stopResponse("giving morale");
			giving_morale = false;
			returningHome = true;

		}
		else
		{

			if (y == 40)
			{
				CompanionRobot::doResponse("socialising");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}

// TODO: SHOULD BE COMPANIONSHIP/LONELINESS ########################################################################################################
void CompanionRobot::moraleCallback(msg_pkg::Morale msg)
{
 	CompanionRobot* temp = dynamic_cast<CompanionRobot*>( ActorSpawner::getInstance().getActor());

 	temp->moraleLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool CompanionRobot::checkMoraleLevel()
{
	if (moraleLevel>=2 )
	{
		return true;
	}
	return false;
}
