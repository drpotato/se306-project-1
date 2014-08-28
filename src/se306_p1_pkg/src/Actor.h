#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include <nav_msgs/Odometry.h>
#include <msg_pkg/Location.h>
#include <msg_pkg/Unlock.h>
#include <msg_pkg/LockStatus.h>
#include "GraphSearch.h"

#include "ros/ros.h"
#include <vector>
#include <string>
#include <map>
#include "std_msgs/String.h"
#include "ActorLocation.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"

/* Macros */
#define CRITICAL_LEVEL 20
#define REASONABLE_LEVEL 80
#define LEVEL_MAX 100 // Final release should be 100
#define LEVEL_MIN 0 // Final release should be 0
#define FREQUENCY 10
#define INC_AMOUNT 10

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
	static void lockStatusCallback(msg_pkg::LockStatus msg);
	static void unlockCallback(msg_pkg::Unlock msg);
	static void locationCallback(msg_pkg::Location msg);

	  // Event hours
  const static int WAKE_TIME = 7;
  const static int BREAKFAST_TIME = 8;
  const static int LUNCH_TIME = 13;
  const static int DINNER_TIME = 18;
  const static int SLEEP_TIME = 23;
	
	ros::NodeHandle &getNodeHandle() const;

	//The rate at which ros will loop - used to calculate time of day
    const static int LOOP_RATE = 10;
    bool gotoPosition(double x,double y);
    enum ActorType {Doctor=3, Nurse=2, Caregiver=2, Visitor=1, Robot=0};

    bool haveLock;
    bool deniedLock;
    bool otherUnlocked;

    bool firstGoToNode;

    struct NodeLocation {
    	int x;
    	int y;
	};

    std::map<string, NodeLocation> nodeLocations;

protected:

	virtual void doInitialSetup() = 0;
	virtual void doExecuteLoop() = 0;
        
    virtual string getActorName() = 0;

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
	ros::Subscriber subscriberlockStatus;
	ros::Subscriber subscriberUnlock;
	ros::Subscriber subscriberLocation;

	ros::Publisher publisherRequestLock;
	ros::Subscriber subscriberLockStatus;
	void requestLock(std::string actor_name);
	ros::Publisher publisherUnlock;
	void unlock();

	std::string rosName;
	std::string stageName;

    //Path Planner
	//Path Planner
    bool goToNode(string);

    bool movingToResident;

    string RCmode;
    void controlRobot();

private:
    
    double faceDirection(double,double);

    vector<GraphSearch::point> *path;
    GraphSearch::point *pDestination;
    GraphSearch::point *pStart;
    
	PathPlanner pathPlanner;
	string currentNode;
	int currentNodeIndex;

    void checkKeyboardPress();
    bool modeSet();
    bool inMode(string mode);
    void toggleMode(string mode);
};


#endif // #ifndef SE306P1_ACTOR_ACTOR_H_DEFINED

