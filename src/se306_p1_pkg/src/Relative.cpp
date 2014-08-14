#include "Relative.h"
#include <msg_pkg/Interaction.h>

#include "PathPlanner.h"
#include "PathPlannerNode.h"

#include "Robot.h"

void Relative::doInitialSetup()
{
  // Create publisher for interaction with Resident
  //publisherInteraction = nodeHandle->advertise<msg_pkg::Interaction>("Lonliness", false);

  // Create subscriber to lonliness attribute of Resident

  // Create 'Leave' publisher
  

  velLinear = 0;
  velRotational = 0.0;

  // Testing only - remove this
  //gotoPosition(-4.0, -4.0);

}

void Relative::doExecuteLoop()
{
  
  PathPlannerNode *target = this->pathPlanner.getNode(&node3Name);
  vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
  this->goToNode(path);
  
  // Moveto goes here
  doResponse("Socialness");


  /* 
   * Make relative spin on spot while interating
   */
  velRotational = 1.0;

  /*
  if (interacting)
  {
    // velRotational = 0.25
  } else {
    // Is moving to resident
  }
  */

  

  // Publish leave message
  publishLeave();

  


    
}
