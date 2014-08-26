#include "Nurse.h"
#include <string>

#include <iostream> 
#include <cstddef>

#include "PathPlanner.h"
#include "PathPlannerNode.h"

void Nurse::doInitialSetup()
{
	velLinear = 0.0;
    velRotational = 0.0;

    string name = ros::this_node::getName();
    std::size_t found6 = name.find_first_of("6");
    std::size_t found7 = name.find_first_of("7");

    if (found6!=std::string::npos){
    	subscriberNurse = nodeHandle->subscribe("nurse1", 1000, Nurse::nurseCallback);
    } else if(found7!=std::string::npos){
    	subscriberNurse = nodeHandle->subscribe("nurse2", 1000, Nurse::nurseCallback);
    } else {
    	//YOU BROKE THE BUILD!
    	ROS_INFO("THIS IS BROKEN! NURSES SHOULD BE KEPT TO ROSNODE6 AND ROSNODE7");
    	ROS_INFO("MY NAME IS %s",name.c_str());
    }
  
}



void Nurse::doExecuteLoop()
{
	if (assist)
	{
		//Follow the doctor
		//goToNode("RosNode4");
	}

}


void Nurse::nurseCallback(msg_pkg::Nurse msg)
{

	//TODO: Make something happen in the if statement
	Nurse* temp = dynamic_cast<Nurse*>( ActorSpawner::getInstance().getActor());
	temp->assist = msg.follow;

}

