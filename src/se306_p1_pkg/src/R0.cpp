#include "R0.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"


void R0::doInitialSetup()
{
	velLinear = 0;
	velRotational = 0.0;
    
    string node1Name = "testnode1";
    string node2Name = "testnode2";
    string node3Name = "testnode3";
    
    PathPlannerNode node1(&node1Name,0,0);
    PathPlannerNode node2(&node2Name,1,0);
    PathPlannerNode node3(&node3Name,2,0);
    
    
    node1.addNeighbour(&node2);
    node2.addNeighbour(&node1);
    node2.addNeighbour(&node3);
    node3.addNeighbour(&node2);
    
    this->pathPlanner.addNode(&node1);
    this->pathPlanner.addNode(&node2);
    this->pathPlanner.addNode(&node3);
    
    this->activeNode = &node1;
    string nodeName = "testnode3";
    ROS_INFO_STREAM("pathfinding");
    this->goToNode(&nodeName);
    
    this->velRotational = 0.3;
    this->velLinear = 0.1;
}

void R0::doExecuteLoop()
{
    this->faceDirection(0,0);

}
