#include "PathPlanner.h"
#include "ros/ros.h"
#include <msg_pkg/Location.h>

vector<PathPlannerNode> PathPlanner::nodes;
ros::Subscriber PathPlanner::subscriberLocation;
ros::NodeHandle* PathPlanner::nodeHandle;

// This class maintains a graph of navigation waypoint nodes, and calculates the shortest (fewest nodes) path between any two of them.

PathPlanner::PathPlanner() {
    nodeHandle = new ros::NodeHandle();
    subscriberLocation = nodeHandle->subscribe("location", 1000, PathPlanner::locationCallback);

    nodeBedroomCentreName = "nodeBedroomCentre";
    nodeHallwayByBedroomName = "nodeHallwayByBedroom";
    nodeHalllwayByLivingRoomName = "nodeHalllwayByLivingRoom";
    nodeGuestBedroomCentreName = "nodeGuestBedroomCentre";
    nodeHouseDoorName = "nodeHouseDoor";

    string nodeResidentName = "RobotNode0";
    string nodeEntertainmentRobotName = "RobotNode2";
    string nodeRelativeName = "RobotNode3";
    string nodeMedicationRobotName = "RobotNode6";
    string nodeCompanionRobotName = "RobotNode7";
    string nodeCookingRobotName = "RobotNode8";

    nodeResident = PathPlannerNode(&nodeResidentName, 0, 0);
    nodeEntertainmentRobot = PathPlannerNode(&nodeEntertainmentRobotName, 0, 0);
    nodeRelative = PathPlannerNode(&nodeRelativeName, 0, 0);
    nodeMedicationRobot = PathPlannerNode(&nodeMedicationRobotName, 0, 0);
    nodeCompanionRobot = PathPlannerNode(&nodeCompanionRobotName, 0, 0);
    nodeCookingRobot = PathPlannerNode(&nodeCookingRobotName, 0, 0);

    nodeBedroomCentre = PathPlannerNode(&nodeBedroomCentreName, -2.5, 3);
    nodeHallwayByBedroom = PathPlannerNode(&nodeHallwayByBedroomName, -2.5, -0);
    nodeHalllwayByLivingRoom = PathPlannerNode(&nodeHalllwayByLivingRoomName, 3, 0);
    nodeGuestBedroomCentre = PathPlannerNode(&nodeGuestBedroomCentreName, -2.5, -3);
    nodeHouseDoor = PathPlannerNode(&nodeHouseDoorName, 2.8, 5);

    // Specify which nodes have a clear line of sight to each other.
    nodeBedroomCentre.addNeighbour(&nodeHallwayByBedroom);
    nodeBedroomCentre.addNeighbour(&nodeGuestBedroomCentre);

    nodeHallwayByBedroom.addNeighbour(&nodeBedroomCentre);
    nodeHallwayByBedroom.addNeighbour(&nodeHalllwayByLivingRoom);
    nodeHallwayByBedroom.addNeighbour(&nodeGuestBedroomCentre);

    nodeHalllwayByLivingRoom.addNeighbour(&nodeHallwayByBedroom);
    nodeHalllwayByLivingRoom.addNeighbour(&nodeHouseDoor);

    nodeGuestBedroomCentre.addNeighbour(&nodeHallwayByBedroom);
    nodeGuestBedroomCentre.addNeighbour(&nodeBedroomCentre);

    nodeHouseDoor.addNeighbour(&nodeHalllwayByLivingRoom);

    // Add the nodes to the path planner's graph of nodes and connections.
    addNode(nodeBedroomCentre);
    addNode(nodeHallwayByBedroom);
    addNode(nodeHalllwayByLivingRoom);
    addNode(nodeGuestBedroomCentre);
    addNode(nodeHouseDoor);
    /*
    addNode(nodeResident);
    addNode(nodeEntertainmentRobot);
    addNode(nodeRelative);
    addNode(nodeMedicationRobot);
    addNode(nodeCompanionRobot);
    addNode(nodeCookingRobot);*/
}

// When a location message is received, updates the graph with that Actor's new location.
void PathPlanner::locationCallback(msg_pkg::Location msg)
{
    // Find Actor of this name in graph and remove it.
    string name = msg.id;
    double x = msg.xpos;
    double y = msg.ypos;

    ROS_INFO("Got callback from %s",name.c_str());

    if (getNode(name) != NULL) {
        updateNode(name, x, y);
    } else {
        ROS_INFO_STREAM("Node does not exist; create it");
        PathPlannerNode* newNode = new PathPlannerNode(&name, x, y);
        PathPlannerNode* closestNode = getClosestNode(x,y);
        newNode->addNeighbour(closestNode);
        closestNode->addNeighbour(newNode);
        addNode(*newNode);
    }
}

// Returns the shortest path between the two given nodes.
vector<PathPlannerNode*> PathPlanner::pathToNode(PathPlannerNode *startNode,PathPlannerNode *target)
{
    PathPlannerNode *top;
    queue<PathPlannerNode*> s;
    s.push(startNode);

    for (int i = 0; i < nodes.size(); i++){
        nodes[i].visited = false;
    }

    startNode->setVisited(true);


    while (s.empty() == false){
        top = s.front();
        s.pop();
        if (top->getName()->compare(*(target->getName())) == 0) {
            //found it!
            break;
        }
        for (int i =0;i<top->neighbours.size();i++){
            if (top->neighbours[i]->isVisited() == false){
                s.push(top->neighbours[i]);
                top->neighbours[i]->previous = top;
            }
        }
        top->setVisited(true);
    }

    vector<PathPlannerNode*> path;
    PathPlannerNode* iter = top;
    while (iter->getName()->compare(*(startNode->getName())) != 0) {
        path.insert(path.begin(),iter);
        iter = iter->previous;
    }
    path.insert(path.begin(),startNode);

    for (int i=0;i<path.size();i++) {
        path[i]->visited = false;
    }
        return path;
}

// Removes a node from the graph, and removes it from all its' neighbours' lists of neighbours.
void PathPlanner::removeNode(string* name) {

    for(int i=0;i<nodes.size();i++) {
        PathPlannerNode* currentNode = &nodes[i];
        if (currentNode->getName()->compare(*name) == 0) {
            // For each of its neighbours
            for (int j = 0; j < currentNode->neighbours.size(); j++) {
                PathPlannerNode* neighbour = currentNode->neighbours[j];
                neighbour->removeNeighbour(currentNode);
            }
          nodes.erase(nodes.begin()+i);
          return;
        }
    }
}

// Adds a PathPlannerNode to the graph.
void PathPlanner::addNode(PathPlannerNode p) {
    nodes.push_back(p);
    int num = nodes.size();
    ROS_INFO("Size of graph is now: %d", num);
}

// Returns the PathPlannerNode with the given name (if any).
PathPlannerNode* PathPlanner::getNode(string name) {
    for (int i = 0; i < nodes.size(); i++) {
        PathPlannerNode* node = &nodes[i];
        if (node->getName()->compare(name) == 0) {
            return node;
        }
    }
    return NULL;
}

// Updates the PathPlannerNode's location with the given x and y, and refinds its closest neighbour.
void PathPlanner::updateNode(string name, double x, double y) {
    for (int i = 0; i < nodes.size(); i++){
        PathPlannerNode* node = &nodes[i];


        if (node->getName()->compare(name) == 0) {
            // This node represents an Actor, and so will only have one neighbour.
            PathPlannerNode* neighbour = node->neighbours[0];
            neighbour->removeNeighbour(node);
        }
        PathPlannerNode* closestNode = getClosestNode(x, y);
        node->addNeighbour(closestNode);
        closestNode->addNeighbour(node);
    }
    int num = nodes.size();
}

// Returns a pointer to the waypoint closest to the given set of coordinates.
// Does not check for walls or collisions, so we will need sufficient coverage of waypoints to ensure this does not become a problem.
PathPlannerNode* PathPlanner::getClosestNode(double x, double y) {
    PathPlannerNode * closestNode = &nodes[0];

    for (int i = 1; i < nodes.size(); i++){
        PathPlannerNode* nodeToCompare = &nodes[i];
        double distanceToOld = sqrtf(pow(x - closestNode->px, 2) + pow(y - closestNode->py, 2));
        double distanceToNew = sqrtf(pow(x - nodeToCompare->px, 2) + pow(y - nodeToCompare->py, 2));
        if (distanceToNew < distanceToOld) {
            closestNode = nodeToCompare;
        }
    }
    return closestNode;
}
