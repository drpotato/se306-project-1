#include "Relative.h"
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>

#include "PathPlanner.h"
#include "PathPlannerNode.h"

#include "Robot.h"



void Relative::doInitialSetup()
{

  interacting = false;
  
  // Set up subscribers
  subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Relative::socialnessCallback);

  velLinear = 0;
  velRotational = 0.0;

}

void Relative::doExecuteLoop()
{
  /*
  // Moveto goes here

  // Publish interaction with 'Socialness' attribute
  doResponse("Socialness");

  // Publish leave message
  publishLeave();
  */

  if (!interacting)
	{
		if (!checkSocialnessLevel())
		{ 
			if (first_call)
			{
				this->activeNode = &node4;
				this->startMovingToResident();
				first_call = false;
			}
	    	
    		

	    	//The or in this case is just for the alpha, remove once the robot is capable of reaching the resident
	    	if (!(this->movingToResident) ){
	    		//EntertainmentRobot::doResponse("entertaining");
	    		ROS_INFO("Relative now interacting (socialising) with resident!");
	    		interacting = true;
	    		first = false;
	    	}
	    	
			//After finished entertaining set entertaining to flase

		}
	} else {
		if (residentSocialnessLevel == 5)
		{
			interacting = false;
		} else 
		{
			Relative::doResponse("socialising");
			
		} 
		
	}
    
}


void Relative::socialnessCallback(msg_pkg::Socialness msg)
{
	// Debug	
	ROS_INFO("callback!");
 	
	Relative* temp = dynamic_cast<Relative*>( ActorSpawner::getInstance().getActor());

 	temp->residentSocialnessLevel = msg.level;

	// Debug
 	ROS_INFO("Changed value");
	
}

bool Relative::checkSocialnessLevel()
{
	if (residentSocialnessLevel>=2 ) {
		return true;
	}

	return false;
}









