#include "PathPlanner.h"
#include "ros/ros.h"

// A single navigation system waypoint.
// Has a name, an x and y position, a boolean indicating whether or not it has been visited, and a list of neighbouiring nodes to which it has a direct line of sight.
PathPlannerNode::PathPlannerNode(string inputName, double x, double y,bool robot){
    this->name = inputName;
    this->px = x;
    this->py = y;
    this->isRobot = robot;
}


string PathPlannerNode::getName() {
    return this->name;
}

bool PathPlannerNode::isVisited() {
    return visited;
}

void PathPlannerNode::setVisited(bool newVisited){
    this->visited = newVisited;
}

void PathPlannerNode::addNeighbour(string newNode) {
    this->neighbours.push_back(newNode);

}

void PathPlannerNode::addNeighbour(PathPlannerNode* newNode){
  this->addNeighbour(newNode->getName());
}

void PathPlannerNode::removeNeighbour(string deleteNode) {

    for (int i=0; i<this->neighbours.size(); i++) {
        if (this->neighbours[i].compare(deleteNode) == 0) {
            this->neighbours.erase(this->neighbours.begin()+i);
            break;
        }
    }

}

void PathPlannerNode::removeAllNeighbours(){
  this->neighbours.clear();
}
