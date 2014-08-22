#include "PathPlanner.h"
#include "ros/ros.h"

// This class maintains a graph of navigation waypoint nodes, and calculates the shortest (fewest nodes) path between any two of them.

// Returns the shortest path between the two given nodes.
vector<PathPlannerNode*> PathPlanner::pathToNode(PathPlannerNode *startNode,PathPlannerNode *target)
{
    PathPlannerNode *top;
    queue<PathPlannerNode*> s;
    s.push(startNode);
    
    for (int i=0;i<this->nodes.size();i++){
        this->nodes[i]->visited = false;
    }
    
    startNode->setVisited(true);
    
    
    while (s.empty() == false){
        top = s.front();
        s.pop();
        if (top->getName()->compare(*(target->getName())) == 0){
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
    while (iter->getName()->compare(*(startNode->getName())) != 0){
        path.insert(path.begin(),iter);
        iter = iter->previous;
    }
    path.insert(path.begin(),startNode);
    
    for (int i=0;i<path.size();i++){
        path[i]->visited = false;
    }
        return path;
}

// Adds a PathPlannerNode to the graph.
void PathPlanner::addNode(PathPlannerNode* p){
    this->nodes.push_back(p);
}

// Returns the PathPlannerNode with the given name (if any).
PathPlannerNode* PathPlanner::getNode(string* name){
    int i =0;
    for (i=0;i<this->nodes.size();i++){
        PathPlannerNode* node = this->nodes[i];
        if (node->getName()->compare(*name) == 0){
            return node;
        }else{
        }
    }
}

// Returns a pointer to the waypoint closest to the given set of coordinates.
// Does not check for walls or collisions, so we will need sufficient coverage of waypoints to ensure this does not become a problem.
PathPlannerNode* PathPlanner::getClosestNode(int x, int y){
    int i;
    PathPlannerNode * closestNode = this->nodes[0];

    for (i=1;i<this->nodes.size();i++){
        PathPlannerNode* nodeToCompare = this->nodes[i];
        double distanceToOld = sqrtf(pow(x - closestNode->px, 2) + pow(y - closestNode->py, 2) 
        double distanceToNew = sqrtf(pow(x - nodeToCompare->px, 2) + pow(y - nodeToCompare->py, 2) 
        if (distanceToNew < distanceToOld) {
            closestNode = nodeToCompare;
        }
    }
    return closestNode;
}
