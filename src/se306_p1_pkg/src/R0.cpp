#include "R0.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"


void R0::doInitialSetup()
{
	velLinear = 0;
	velRotational = 0.0;
    PathPlannerNode
    PathPlannerNode node1 = PathPlannerNode("testnode",0,0);
    this->pathPlanner.addNode(&node1);
}

void R0::doExecuteLoop()
{
    
}
