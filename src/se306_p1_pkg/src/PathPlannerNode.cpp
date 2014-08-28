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
    ROS_INFO("we have %d neighbours %s",this->neighbours.size(),this->getName().c_str());
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
  for (int i=0; i<this->neighbours.size(); i++) {
        PathPlanner::getNode(this->neighbours[i])->removeNeighbour(this->getName());
  }

  this->neighbours.clear();
}


//Copy Constructor
//PathPlannerNode::PathPlannerNode(const PathPlannerNode &other) : name(other.name),px(other.px),py(other.py),neighbours(other.neighbours){}
