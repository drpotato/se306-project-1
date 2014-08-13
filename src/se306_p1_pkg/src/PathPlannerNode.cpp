#include "PathPlanner.h"
#include "ros/ros.h"

PathPlannerNode::PathPlannerNode(string* inputName,double x,double y){
    this->name = inputName;
    this->px = x;
    this->py = y;
}

string* PathPlannerNode::getName(){
    ROS_INFO_STREAM("name");
    ROS_INFO_STREAM(*(this->name));
    return this->name;
}

bool PathPlannerNode::isVisited(){
    return visited;
}

void PathPlannerNode::setVisited(bool newVisited){
    this->visited = newVisited;
}

void PathPlannerNode::addNeighbour(PathPlannerNode* newNode){
    this->neighbours.push_back(newNode);
}



void PathPlannerNode::removeNeighbour(PathPlannerNode* deleteNode){
    for (int i=0; i<this->neighbours.size(); i++){
        if (this->neighbours[i]->getName() == deleteNode->getName()){
            this->neighbours.erase(this->neighbours.begin()+i);
            break;
        }
    }
}
