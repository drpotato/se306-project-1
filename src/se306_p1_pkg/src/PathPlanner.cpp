#include "PathPlanner.h"


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

void PathPlanner::addNode(PathPlannerNode* p){
    this->nodes.push_back(p);
}

PathPlannerNode* PathPlanner::getNode(string* name){
    int i =0;
    for (i=0;i<this->nodes.size();i++){
        PathPlannerNode* node = this->nodes[i];
        if (node->getName()->compare(*name) == 0){
            return node;
        }
    }
}
