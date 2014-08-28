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
    /*
    //TODO: Refactor
    static ros::Subscriber subscriberLocation;
    static ros::NodeHandle *nodeHandle;
    */
    typedef boost::unordered_map<string,string> map;

    string nodeBedroomCentreName;
    string nodeHallwayByBedroomName;
    string nodeHalllwayByLivingRoomName;
    string nodeGuestBedroomCentreName;
    string nodeHouseDoorName;


    PathPlannerNode* nodeBedroomCentre;
    PathPlannerNode* nodeHallwayByBedroom;
    PathPlannerNode* nodeHalllwayByLivingRoom;
    PathPlannerNode* nodeGuestBedroomCentre;
    PathPlannerNode* nodeHouseDoor;

public:
    PathPlanner();
    void processMessage(msg_pkg::Location);
    vector<PathPlannerNode*> pathToNode(PathPlannerNode*, PathPlannerNode*);
    void addNode(PathPlannerNode);
    bool hasNode(string);
    void removeNode(string*);
    PathPlannerNode* getNode(string);
    void updateNode(string, double, double);
    PathPlannerNode* getClosestNode(double, double);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED
