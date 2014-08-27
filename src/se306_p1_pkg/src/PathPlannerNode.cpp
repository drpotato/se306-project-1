#include "PathPlanner.h"
#include "ros/ros.h"

// A single navigation system waypoint.
// Has a name, an x and y position, a boolean indicating whether or not it has been visited, and a list of neighbouiring nodes to which it has a direct line of sight.
PathPlannerNode::PathPlannerNode(string inputName, double x, double y){
    this->name = inputName;
    this->px = x;
    this->py = y;
}

PathPlannerNode::PathPlannerNode(){}

string PathPlannerNode::getName(){
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


//Copy Constructor
PathPlannerNode::PathPlannerNode(const PathPlannerNode &other){
  this->name = other.name;
  this->px = other.px;
  this->py = other.py;
}
