#include "PathPlanner.h"
#include "ros/ros.h"


vector<PathPlannerNode*> PathPlanner::pathToNode(PathPlannerNode *startNode,PathPlannerNode *target)
{
    
    
    PathPlannerNode *top;
    
    queue<PathPlannerNode*> s;
    s.push(startNode);
    
    ROS_INFO_STREAM(*(target->getName()));
    
    startNode->setVisited(true);
    
    
    
    while (s.empty() == false){
        top = s.front();
        s.pop();
        if (top->getName()->compare(*(target->getName())) == 0){
            //found it!
            break;
        }
        for (int i =0;i<top->neighbours.size();i++){
            if (top->neighbours[i]->isVisited() == false){
                s.push(top->neighbours[i]);
                this->previousNodes[*(top->neighbours[i]->getName())] = *(top->getName());
            }
        }
    }
    vector<PathPlannerNode*> path;
    PathPlannerNode* iter = top;
    while (iter->getName()->compare(*(startNode->getName())) == 0){
        path.insert(path.begin(),iter);
        iter = this->getNode(&(this->previousNodes[*(iter->getName())]));
    }
    return path;
}

void PathPlanner::addNode(PathPlannerNode* p){
    this->nodes.push_back(p);
}

PathPlannerNode* PathPlanner::getNode(string* name){
    int i =0;
    for (i=0;i<this->nodes.size();i++){
        PathPlannerNode* node = this->nodes[i];
        if (node->getName()->compare(*name) == 0){
            return node;
        }
    }
    ROS_INFO_STREAM("Should never get here");
}
