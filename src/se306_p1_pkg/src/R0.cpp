#include "R0.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include <queue>

void R0::doInitialSetup()
{
	velLinear = 0;
	velRotational = 0.0;
}

void R0::doExecuteLoop()
{
    //Dummy code drives to a few nodes, could change this
    string node3Name = "testnode3";
    PathPlannerNode *target = this->pathPlanner.getNode(&node3Name);
    vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    this->goToNode(path);
}
