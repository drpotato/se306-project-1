#include "PathPlanner.h"
#include "ros/ros.h"

// A single navigation system waypoint.
// Has a name, an x and y position, a boolean indicating whether or not it has been visited, and a list of neighbouiring nodes to which it has a direct line of sight.
PathPlannerNode::PathPlannerNode(string inputName, double x, double y){
    this->name = inputName;
    this->px = x;
    this->py = y;
}

PathPlannerNode::PathPlannerNode() {}

string PathPlannerNode::getName() {
    return this->name;
}

bool PathPlannerNode::isVisited() {
    return visited;
}

void PathPlannerNode::setVisited(bool newVisited){
    this->visited = newVisited;
}

void PathPlannerNode::addNeighbour(PathPlannerNode* newNode) {
    ROS_INFO("Added Neighbour %s, I am %s",newNode->getName().c_str(),this->getName().c_str());
    this->neighbours.push_back(newNode);
}

void PathPlannerNode::removeNeighbour(PathPlannerNode* deleteNode) {
    ROS_INFO("we have %d neighbours and my name is %s",this->neighbours.size(),this->getName().c_str());
    for (int i=0; i<this->neighbours.size(); i++) {
        ROS_INFO("compare %s to %s",this->neighbours[i]->getName().c_str(),deleteNode->getName().c_str());
        if (this->neighbours[i]->getName().compare(deleteNode->getName()) == 0) {
            ROS_INFO("Erasing neighbour %s for node %s",this->neighbours[i]->getName().c_str(),this->getName().c_str());
            this->neighbours.erase(this->neighbours.begin()+i);
            break;
        }
    }

}

void PathPlannerNode::removeAllNeighbours(){
  for (int i=0; i<this->neighbours.size(); i++) {
        this->neighbours[i]->removeNeighbour(this);
  }

  this->neighbours.clear();
}


//Copy Constructor
PathPlannerNode::PathPlannerNode(const PathPlannerNode &other) {
  this->name = other.name;
  this->px = other.px;
  this->py = other.py;
  this->neighbours = other.neighbours;
}
