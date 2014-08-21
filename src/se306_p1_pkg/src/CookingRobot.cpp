#include "CookingRobot.h"
#include "ActorSpawner.h"

#include "Actor.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include "ActorSpawner.h"



void CookingRobot::doInitialSetup()
{
	velLinear = 0.0;
	velRotational = 0.0;
	entertainednessLevel = 5;
	entertaining = false;
	residentName = "RobotNode2";
	subscriberEntertainedness = nodeHandle->subscribe("entertainedness", 1000, CookingRobot::entertainednessCallback);
	y = 0;
	x = 0;
	first = true;
	first_call = true;
	returningHome = false;
	returningHome_first = true;
}

void CookingRobot::doExecuteLoop()
{
	if (returningHome){
		//ROS_INFO("MOVEING TO HOME");

		if (returningHome_first){
			returningHome_first = false;
			//TODO: Matt fix this shit (Target node reset upon reach destination)
			//targetNode = 0;
		}
		/*
        PathPlannerNode *target = this->pathPlanner.getNode(&node5Name);
        vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
        if (this->goToNode(path))
        {
        	//ROS_INFO("ARRIVE HOME");
        	returningHome=false;
        }*/


        return;

	}

	if (!entertaining)
	{
		if (checkCookingLevel())
		{
			//ROS_INFO("Nothing to do here");
		}
		else
		{
			if (first_call)
			{
				//this->activeNode = &node5;
				this->startMovingToResident();
				first_call = false;
			}
			//Call method to do the entertaining
			//PathPlannerNode *target = this->pathPlanner.getNode(&node2Name);
	    	//vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);



	    	//The or in this case is just for the alpha, remove once the robot is capable of reaching the resident
	    	if (!(this->movingToResident) )
	    	{
	    		//CookingRobot::doResponse("entertaining");
	    		ROS_INFO("CHANGED TO ENTERTAINING");
	    		entertaining=true;
	    		first = false;
	    	}

			//After finished entertaining set entertaining to flase

		}
	}
	else
	{
		if (entertainednessLevel == 5)
		{
			//Add do last desponse call that kurt implimented
			CookingRobot::stopResponse("entertaining");
			entertaining = false;
			returningHome = true;

		}
		else
		{

			if (y == 40)
			{
				CookingRobot::doResponse("entertaining");
				y=0;
			}
			else
			{
				y++;
			}
		}
	}
}


void CookingRobot::entertainednessCallback(msg_pkg::Entertainedness msg)
{
 	CookingRobot* temp = dynamic_cast<CookingRobot*>( ActorSpawner::getInstance().getActor());

 	temp->entertainednessLevel = msg.level;
 	//ROS_INFO("Changed value");
}

bool CookingRobot::checkCookingLevel()
{
	if (entertainednessLevel>=2 )
	{
		return true;
	}
	return false;
}
