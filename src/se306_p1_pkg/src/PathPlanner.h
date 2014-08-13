#ifndef SE306P1_PATHPLANNER_H_DEFINED
#define SE306P1_PATHPLANNER_H_DEFINED

#include <vector>
#include <string>
#include <queue>
#include <boost/unordered_map.hpp>
#include "PathPlannerNode.h"

using namespace std;

class PathPlanner
{
private:
    vector<PathPlannerNode*> nodes;
    typedef boost::unordered_map<string,string> map;
    map previousNodes;
public:
	vector<PathPlannerNode*> pathToNode(PathPlannerNode*,PathPlannerNode*);
    void addNode(PathPlannerNode*);
    PathPlannerNode* getNode(string*);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED

