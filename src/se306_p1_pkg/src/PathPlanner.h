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
    vector<PathPlannerNode> nodes;

    typedef boost::unordered_map<string,string> map;

    string nodeBedroomCentreName;
    string nodeHallwayByBedroomName;
    string nodeHallwayByLivingRoomName;
    string nodeGuestBedroomCentreName;
    string nodeHouseDoorName;

    PathPlannerNode* nodeBedroomCentre;
    PathPlannerNode* nodeHallwayByBedroom;
    PathPlannerNode* nodeHallwayByLivingRoom;
    PathPlannerNode* nodeGuestBedroomCentre;
    PathPlannerNode* nodeHouseDoor;
    
    void processMessage(msg_pkg::Location);

public:
    PathPlanner();
    void update(string);
    void updateAll();
    vector<PathPlannerNode*> pathToNode(string,string);
    void addNode(PathPlannerNode);
    bool hasNode(string);
    void removeNode(string*);
    PathPlannerNode* getNode(string);
    void updateNode(string, double, double);
    PathPlannerNode* getClosestNode(double, double);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED
