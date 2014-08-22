#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include <nav_msgs/Odometry.h>
#include <msg_pkg/Location.h>

#include "ros/ros.h"
#include "PathPlanner.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "ActorLocation.h"

class Actor
{
public:
	Actor();
	virtual ~Actor();

	void initialSetup(unsigned int robotID, double px, double py, double theta);
	bool executeLoop();

	void initialSetupStage();
	void publishLocation();

	void executeLoopStageSubscription();
	void executeLoopStagePublication();

	static void StageOdom_callback(nav_msgs::Odometry msg);
	static void locationCallback(msg_pkg::Location msg);

    bool moveToResident();
    void startMovingToResident();


protected:

	virtual void doInitialSetup() = 0;
	virtual void doExecuteLoop() = 0;

	void doResponse(const char *attribute);
        void stopResponse(const char *attribute);

	//velocity of the robot
	double velLinear;
	double velRotational;

	//pose of the robot
	double pxInitial;
	double pyInitial;
	double thetaInitial;
	double px;
	double py;
	double theta;

	// ROS-specific stuff
	ros::NodeHandle *nodeHandle;
	ros::Rate *loopRate;

	ros::Publisher  publisherStageVelocity;
	ros::Publisher  publisherLocation;
	ros::Publisher  publisherInteraction;
	ros::Subscriber subscriberStageOdometry;
	ros::Subscriber subscriberStageLaserScan;
	ros::Subscriber subscriberLocation;

	std::string rosName;
	std::string stageName;

    //Path Planner
    bool goToNode(vector<PathPlannerNode*> &path);
    PathPlannerNode* getActiveNode();

    bool movingToResident;

private:
    vector<ActorLocation> actorLocations;
    PathPlanner pathPlanner;
    PathPlannerNode *activeNode;
    double faceDirection(double,double);
    double getActorX(string);
    double getActorY(string);
    bool gotoPosition(double x,double y);
    int targetNode;
    PathPlannerNode node1;
    PathPlannerNode node2;
    PathPlannerNode node3;
    PathPlannerNode node4;
    PathPlannerNode node5;
    PathPlannerNode nodeDoor;
    string node1Name;
    string node2Name;
    string node3Name;
    string node4Name;
    string node5Name;
    string nodeDoorName;
};


#endif // #ifndef SE306P1_ACTOR_ACTOR_H_DEFINED

