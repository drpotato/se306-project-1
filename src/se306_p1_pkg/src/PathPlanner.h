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
    static vector<PathPlannerNode> nodes;
    static ros::Subscriber subscriberLocation;
    static ros::NodeHandle *nodeHandle;

    typedef boost::unordered_map<string,string> map;

    string nodeBedroomCentreName;
    string nodeHallwayByBedroomName;
    string nodeHalllwayByLivingRoomName;
    string nodeGuestBedroomCentreName;
    string nodeHouseDoorName;

    PathPlannerNode nodeBedroomCentre;
    PathPlannerNode nodeHallwayByBedroom;
    PathPlannerNode nodeHalllwayByLivingRoom;
    PathPlannerNode nodeGuestBedroomCentre;
    PathPlannerNode nodeHouseDoor;

    string nodeResidentName;
    string nodeEntertainmentRobotName;
    string nodeRelativeName;
    string nodeMedicationRobotName;
    string nodeCompanionRobotName;
    string nodeCookingRobotName;

    PathPlannerNode nodeResident;
    PathPlannerNode nodeEntertainmentRobot;
    PathPlannerNode nodeRelative;
    PathPlannerNode nodeMedicationRobot;
    PathPlannerNode nodeCompanionRobot;
    PathPlannerNode nodeCookingRobot;

public:
    PathPlanner();

    static vector<PathPlannerNode*> pathToNode(PathPlannerNode*, PathPlannerNode*);
    static void addNode(PathPlannerNode);
    static void removeNode(string*);
    static PathPlannerNode* getNode(string);
    static void updateNode(string, double, double);
    static PathPlannerNode* getClosestNode(double, double);
    static void locationCallback(msg_pkg::Location msg);
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED
