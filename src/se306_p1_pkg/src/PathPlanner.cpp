#include "PathPlanner.h"



void PathPlanner::pathToNode(string start,string target)
{
    PathPlannerNode startNode = PathPlannerNode(start);
    PathPlannerNode *top;
    
    queue<PathPlannerNode> s;
    s.push(startNode);
    startNode.setVisited(true);
    while (s.empty() == false){
        top = &s.front();
        s.pop();
        if (top->getName() == target){
            //found it!
            break;
        }
        for (int i =0;i<top->neighbours.size();i++){
            if (top->neighbours[i].isVisited() == false){
                s.push(top->neighbours[i]);
            }
        }
    }

    
}
