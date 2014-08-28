#ifndef SE306P1_PATHPLANNERLISTENER_H_DEFINED
#define SE306P1_PATHPLANNERLISTENER_H_DEFINED

#include <set>
#include <string>
#include <queue>
#include "ros/ros.h"
#include "PathPlannerNode.h"
#include <msg_pkg/Location.h>

using namespace std;

class PathPlannerListener {
private:
    static ros::Subscriber subscriberLocation;
    static ros::NodeHandle *nodeHandle;
    static map<std::string, queue<msg_pkg::Location> > messageQueues;

public:
    PathPlannerListener();
    static void locationCallback(msg_pkg::Location);
    static queue<msg_pkg::Location>* getMessages(string);
    static set<string> getQueueTitles();
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED
