#ifndef SE306P1_PATHPLANNER_H_DEFINED
#define SE306P1_PATHPLANNER_H_DEFINED

#include <vector>
#include <string>
#include <queue>
#include "ros/ros.h"
#include <boost/unordered_map.hpp>
#include "PathPlannerNode.h"
#include <msg_pkg/Location.h>


using namespace std;

class PathPlanner {
private:
    static vector<PathPlannerNode*> nodes;
    static ros::Subscriber subscriberLocation;
    static ros::NodeHandle *nodeHandle;

    typedef boost::unordered_map<string,string> map;

public:
    PathPlanner();
    
    static vector<PathPlannerNode*> pathToNode(PathPlannerNode*, PathPlannerNode*);
    static void addActorNode(PathPlannerNode*);
    static void addNode(PathPlannerNode*);
    static PathPlannerNode* removeNode(string*);
    static PathPlannerNode* getNode(string*);
    static PathPlannerNode* getClosestNode(int, int);
    static void locationCallback(msg_pkg::Location msg);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED