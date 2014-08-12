#ifndef SE306P1_PATHPLANNER_H_DEFINED
#define SE306P1_PATHPLANNER_H_DEFINED

#include <vector>
#include <string>
#include <queue>
#include "PathPlannerNode.h"
using namespace std;

class PathPlanner
{
private:
    vector<PathPlannerNode> nodes;
public:
	void pathToNode(string,string);
    void addNode(PathPlannerNode);
    PathPlannerNode getNode(string);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED

