#ifndef SE306P1_PATHPLANNER_H_DEFINED
#define SE306P1_PATHPLANNER_H_DEFINED

#include <vector>
#include <string>
#include <queue>
#include "ros/ros.h"
#include <boost/unordered_map.hpp>
#include "PathPlannerNode.h"
#include <msg_pkg/Location.h>
#include "ActorLocation.h"

using namespace std;

class PathPlanner {
private:
    vector<ActorLocation> actorLocations;
    vector<PathPlannerNode*> nodes;
    typedef boost::unordered_map<string,string> map;

    ros::Subscriber subscriberLocation;
    ros::NodeHandle *nodeHandle;
    
public:
    PathPlanner();
	vector<PathPlannerNode*> pathToNode(PathPlannerNode*, PathPlannerNode*);
    void addActorNode(PathPlannerNode*);
    void addNode(PathPlannerNode*);
    PathPlannerNode* removeNode(string*);
    PathPlannerNode* getNode(string*);
    PathPlannerNode* getClosestNode(int, int);
    static void locationCallback(msg_pkg::Location msg);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED