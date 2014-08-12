#include "PathPlanner.h"



void PathPlanner::pathToNode(string start,string target)
{
    PathPlannerNode startNode = PathPlannerNode(start);
    PathPlannerNode *top;
    
    queue<PathPlannerNode> s;
    s.push(startNode);
    startNode.setVisited(true);
    while (s.empty() == false){
        top = &s.front();
        s.pop();
        if (top->getName() == target){
            //found it!
            break;
        }
        for (int i =0;i<top->neighbours.size();i++){
            if (top->neighbours[i].isVisited() == false){
                s.push(top->neighbours[i]);
            }
        }
    }
    
}

void PathPlanner::addNode(PathPlannerNode p){
    this->nodes.push_back(p);
}

PathPlannerNode PathPlanner::getNode(string name){
    int i =0;
    for (i=0;i<this->nodes.size();i++){
        if (this->nodes[i].getName() == name){
            return nodes[i];
        }
    }
}
