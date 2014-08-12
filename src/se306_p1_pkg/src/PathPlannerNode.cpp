#include "PathPlanner.h"


PathPlannerNode::PathPlannerNode(string inputName,double x,double y){
    this->name = inputName;
    this->px = x;
    this->py = y;
}

string PathPlannerNode::getName(){
    return this->name;
}

bool PathPlannerNode::isVisited(){
    return visited;
}

void PathPlannerNode::setVisited(bool newVisited){
    this->visited = newVisited;
}

