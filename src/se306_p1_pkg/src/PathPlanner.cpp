#include "PathPlanner.h"
#include "PathPlannerListener.h"
#include "ros/ros.h"
#include <msg_pkg/Location.h>

// This class maintains a graph of navigation waypoint nodes, and calculates the shortest (fewest nodes) path between any two of them.

PathPlanner::PathPlanner() {
    nodeBedroomCentreName = "nodeBedroomCentre";
    nodeHallwayByBedroomName = "nodeHallwayByBedroom";
    nodeHallwayByLivingRoomName = "nodeHallwayByLivingRoom";
    nodeGuestBedroomCentreName = "nodeGuestBedroomCentre";
    nodeHouseDoorName = "nodeHouseDoor";

    nodeBedroomCentre = new PathPlannerNode(nodeBedroomCentreName, -2.5, 3, false);
    nodeHallwayByBedroom = new PathPlannerNode(nodeHallwayByBedroomName, -2.5, -0, false);
    nodeHallwayByLivingRoom = new PathPlannerNode(nodeHallwayByLivingRoomName, 3, 0, false);
    nodeGuestBedroomCentre = new PathPlannerNode(nodeGuestBedroomCentreName, -2.5, -3, false);
    nodeHouseDoor = new PathPlannerNode(nodeHouseDoorName, 2.8, 5, false);

    // Specify which nodes have a clear line of sight to each other.
    nodeBedroomCentre->addNeighbour(nodeHallwayByBedroom->getName());
    nodeBedroomCentre->addNeighbour(nodeGuestBedroomCentre->getName());

    nodeHallwayByBedroom->addNeighbour(nodeBedroomCentre->getName());
    nodeHallwayByBedroom->addNeighbour(nodeHallwayByLivingRoom->getName());
    nodeHallwayByBedroom->addNeighbour(nodeGuestBedroomCentre->getName());

    nodeHallwayByLivingRoom->addNeighbour(nodeHallwayByBedroom->getName());
    nodeHallwayByLivingRoom->addNeighbour(nodeHouseDoor->getName());

    nodeGuestBedroomCentre->addNeighbour(nodeHallwayByBedroom->getName());
    nodeGuestBedroomCentre->addNeighbour(nodeBedroomCentre->getName());

    nodeHouseDoor->addNeighbour(nodeHallwayByLivingRoom->getName());

    // Add the nodes to the path planner's graph of nodes and connections.
    addNode(*nodeBedroomCentre);
    addNode(*nodeHallwayByBedroom);
    addNode(*nodeHallwayByLivingRoom);
    addNode(*nodeGuestBedroomCentre);
    addNode(*nodeHouseDoor);
}

void PathPlanner::update(string name){
  //Get messages from listener
  queue<msg_pkg::Location>* messages = PathPlannerListener::getMessages(name);
  while(messages->size() >0){
    msg_pkg::Location msg = messages->front();
    messages->pop();
    this->processMessage(msg);
  }
}

void PathPlanner::updateAll(){
  set<string> queueTitles = PathPlannerListener::getQueueTitles();
  for (set<string>::iterator it = queueTitles.begin(); it != queueTitles.end(); ++it)
  {
    update(*it);
  }
}

void PathPlanner::processMessage(msg_pkg::Location msg){
  // Find Actor of this name in graph and remove it.
  string name = msg.id;
  double x = msg.xpos;
  double y = msg.ypos;
  if (hasNode(name)) {
      updateNode(name, x, y);
      //ROS_INFO("Updating node %s to (%f, %f)", name.c_str(), x, y);
  } else {
      ROS_INFO("adding new node (%s)", name.c_str());
      PathPlannerNode* newNode = new PathPlannerNode(name, x, y, true);
      PathPlannerNode* closestNode = getClosestNode(x, y);
      addNode(*newNode);
      getNode(name)->addNeighbour(closestNode);
      closestNode->addNeighbour(name);
  }
}

// Returns the shortest path between the two given nodes.
vector<PathPlannerNode*> PathPlanner::pathToNode(string startNode,string target)
{
    ROS_INFO_STREAM("2.1");
    PathPlannerNode *top;
    queue<PathPlannerNode*> s;
    s.push(this->getNode(startNode));
    ROS_INFO_STREAM("2.2");

    for (int i = 0; i < nodes.size(); i++){
        nodes[i].setVisited(false);
        ROS_INFO_STREAM("2.3x");
    }

    if(!hasNode(target)) {
	vector<PathPlannerNode*> retVal;
        return retVal;
    }

    this->getNode(startNode)->setVisited(true);

    ROS_INFO_STREAM("2.5");

    while (s.empty() == false){
    ROS_INFO_STREAM("2.6x");
        top = s.front();
        s.pop();
        if (top->getName().compare(target) == 0) {
    ROS_INFO_STREAM("2.7 - Target node found");
            break;
        }
        for (int i =0;i<top->neighbours.size();i++){
            if (getNode(top->neighbours[i])->isVisited() == false){
                s.push(getNode(top->neighbours[i]));
                getNode(top->neighbours[i])->previous = top;
            }
        }
        top->setVisited(true);
    }

    vector<PathPlannerNode*> path;
    PathPlannerNode* iter = top;

    while (iter->getName().compare(startNode) != 0) {
        path.insert(path.begin(),iter);
        iter = iter->previous;
    }
    path.insert(path.begin(),getNode(startNode));

    for (int i=0;i<path.size();i++) {
        path[i]->visited = false;
    }

    ROS_INFO_STREAM("Path found!");
    for (int i=0;i < path.size(); i++) {
        ROS_INFO_STREAM(path[i]->getName());
    }
    return path;
}

// Removes a node from the graph, and removes it from all its' neighbours' lists of neighbours.
void PathPlanner::removeNode(string* name) {
    ROS_INFO("removenode called");
    for(int i=0;i<nodes.size();i++) {
        PathPlannerNode* currentNode = &nodes[i];
        if (currentNode->getName().compare(*name) == 0) {
            // For each of its neighbours
            for (int j = 0; j < currentNode->neighbours.size(); j++) {
                PathPlannerNode* neighbour = getNode(currentNode->neighbours[j]);
                neighbour->removeNeighbour(*name);
            }
          nodes.erase(nodes.begin()+i);
          return;
        }
    }
}

// Adds a PathPlannerNode to the graph.
void PathPlanner::addNode(PathPlannerNode p) {
    nodes.push_back(p);
    ROS_INFO("Node added: %s", p.getName().c_str());
    int num = nodes.size();
    ROS_INFO("Current size of nodes is %d", num);
}

// Returns the PathPlannerNode with the given name (if any).
PathPlannerNode* PathPlanner::getNode(string name) {
    for (int i = 0; i < nodes.size(); i++) {
        PathPlannerNode* node = &nodes[i];
        if (node->getName().compare(name) == 0) {
            return node;
        }
    }
    return NULL;
}

bool PathPlanner::hasNode(string name){
    for (int i = 0; i < nodes.size(); i++) {
        PathPlannerNode* node = &nodes[i];
        if (node->getName().compare(name) == 0) {
            return true;
        }
    }
    return false;
}

// Updates the PathPlannerNode's location with the given x and y, and refinds its closest neighbour.
void PathPlanner::updateNode(string name, double x, double y) {
    PathPlannerNode* node = getNode(name);
    //TODO: Remove links to this node
    node->removeAllNeighbours();
    PathPlannerNode* closestNode = getClosestNode(x, y);
    node->addNeighbour(closestNode->getName());
    closestNode->addNeighbour(name);
    node->px = x;
    node->py = y;
}

// Returns a pointer to the waypoint closest to the given set of coordinates.
// Does not check for walls or collisions, so we will need sufficient coverage of waypoints to ensure this does not become a problem.
PathPlannerNode* PathPlanner::getClosestNode(double x, double y) {
    PathPlannerNode * closestNode = &nodes[0];

    for (int i = 1; i < nodes.size(); i++){
        PathPlannerNode* nodeToCompare = &nodes[i];
        double distanceToOld = sqrtf(pow(x - closestNode->px, 2) + pow(y - closestNode->py, 2));
        double distanceToNew = sqrtf(pow(x - nodeToCompare->px, 2) + pow(y - nodeToCompare->py, 2));
        ROS_INFO_STREAM(" ");
        ROS_INFO("Distance to old is %f", distanceToOld);
        ROS_INFO("Distance to new is %f", distanceToNew);
        if(nodeToCompare->isRobot) {ROS_INFO_STREAM("This is an Actor");}
        if (distanceToNew < distanceToOld && !nodeToCompare->isRobot) {
            ROS_INFO_STREAM("THIS IS LESS");
            closestNode = nodeToCompare;
        }
    }
    ROS_INFO_STREAM("Closest node found, returning");
    return closestNode;
}