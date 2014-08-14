#include "R0.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"


void R0::doInitialSetup()
{
	velLinear = 0;
	velRotational = 0.0;
}

void R0::doExecuteLoop()
{
    PathPlannerNode *target = this->pathPlanner.getNode(&node4Name);
    vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    this->goToNode(path);
}
