#include "Relative.h"
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>

#include "PathPlanner.h"
#include "PathPlannerNode.h"


string Relative::getActorName()
{
  return "Relative";
}

// A Visitor who provides Socialness.
void Relative::doInitialSetup()
{
    velLinear = 0.0;
    velRotational = 0.0;
    socialnessLevel = 5;
    socialising = false;
    residentName = "Relative1";
    subscriberSocialness = nodeHandle->subscribe("socialness", 1000, Relative::socialnessCallback);
    y = 0;
    x = 0;
    first = true;
    first_call = true;
    returningHome = false;
    returningHome_first = true;
}

void Relative::doExecuteLoop()
{
    if (RCmode == "relative1")
    {
        Relative::controlRobot();
    }
    if (returningHome){
        //ROS_INFO("MOVING TO HOME");

        if (returningHome_first){
            //this->activeNode = &node1;
            returningHome_first = false;
            //TODO: Matt fix this shit (Target node reset upon reach destination)
            //targetNode = 0;
        }
        /*PathPlannerNode *target = this->pathPlanner.getNode(&nodeDoorName);
        vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);*
        if (this->goToNode(path))
        {
            //ROS_INFO("ARRIVE HOME");
            returningHome=false;
        }*/
        return;

    }

    if (!socialising)
    {
        if (!checkSocialnessLevel())
        {
            if (first_call)
            {
                //TODO: go to resident
                first_call = false;
            }

            if (!(true) ) //TODO: go to resident
            {
                //Relative::doResponse("socialising");
                ROS_INFO("CHANGED TO SOCIALISING");
                socialising=true;
                first = false;
            }
        }
    }

    else {
        if (socialnessLevel == 5)
        {
            //Add do last desponse call that kurt implimented
            Relative::stopResponse("socialising");
            socialising = false;
            returningHome = true;

        } 
        else
        {

            if (y == 40)
            {
                Relative::doResponse("socialising");
                y=0;
            } 
            else 
            {
                y++;
            }   
        }
    }
}

/*
 * Upon receiving a message published to the 'socialness' topic, respond appropriately.
 */
void Relative::socialnessCallback(msg_pkg::Socialness msg)
{ 	
	Relative* temp = dynamic_cast<Relative*>( ActorSpawner::getInstance().getActor());

 	temp->socialnessLevel = msg.level;	
}

bool Relative::checkSocialnessLevel()
{
  return socialnessLevel >= 2;
}
