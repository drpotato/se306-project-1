#include "R0.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"
#include <queue>

void R0::doInitialSetup()
{
	velLinear = 0;
	velRotational = 0.0;
    
    string node1Name = "testnode1";
    string node2Name = "testnode2";
    string node3Name = "testnode3";
    
    
    PathPlannerNode node1 = PathPlannerNode(&node1Name,0,0);
    PathPlannerNode node2 = PathPlannerNode(&node2Name,2,2);
    PathPlannerNode node3 = PathPlannerNode(&node3Name,5,0);

    
    
    node1.addNeighbour(&node2);
    node2.addNeighbour(&node1);
    node2.addNeighbour(&node3);
    node3.addNeighbour(&node2);
    
    this->pathPlanner.addNode(&node1);
    this->pathPlanner.addNode(&node2);
    this->pathPlanner.addNode(&node3);
    
    this->activeNode = &node1;
   
    status = 0;
    
}

void R0::doExecuteLoop()
{
    ROS_INFO_STREAM("foo");
    string node3Name = "testnode3";
    
    PathPlannerNode *target = this->pathPlanner.getNode(&node3Name);
    vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    

    this->goToNode(path);
}
