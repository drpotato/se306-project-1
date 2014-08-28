#include "PathPlanner.h"
#include "ros/ros.h"

// A single navigation system waypoint.
// Has a name, an x and y position, a boolean indicating whether or not it has been visited, and a list of neighbouiring nodes to which it has a direct line of sight.
PathPlannerNode::PathPlannerNode(string inputName, double x, double y){
    this->name = inputName;
    this->px = x;
    this->py = y;
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
  int num = this->neighbours.size();
  ROS_INFO("Node %s has %d neighbours", this->getName().c_str(), num);
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
  for (int i=0; i<this->neighbours.size(); i++) {
        PathPlanner::getNode(this->neighbours[i])->removeNeighbour(this->getName());
  }

  this->neighbours.clear();
}