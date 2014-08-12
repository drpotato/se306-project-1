#include "PathPlanner.h"


PathPlannerNode::PathPlannerNode(string inputName){
    this->name = inputName;
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

