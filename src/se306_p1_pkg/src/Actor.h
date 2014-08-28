#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include <nav_msgs/Odometry.h>
#include <msg_pkg/Location.h>
#include <msg_pkg/Unlock.h>
#include <msg_pkg/LockStatus.h>

#include "ros/ros.h"
#include <vector>
#include <string>
#include "std_msgs/String.h"
#include "ActorLocation.h"
#include "PathPlanner.h"
#include "PathPlannerNode.h"

/* Macros */
#define CRITICAL_LEVEL 90
#define LEVEL_MAX 100 // Final release should be 100
#define LEVEL_MIN 0 // Final release should be 0
#define FREQUENCY 10

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
	static void lockStatusCallback(msg_pkg::LockStatus msg);
	static void unlockCallback(msg_pkg::Unlock msg);

    bool moveToResident();
    void startMovingToResident();
	
	ros::NodeHandle &getNodeHandle() const;

	//The rate at which ros will loop - used to calculate time of day
    const static int LOOP_RATE = 10;

    enum ActorType {Doctor=3, Nurse=2, Caregiver=2, Visitor=1, Robot=0};

    bool haveLock;
    bool deniedLock;
    bool otherUnlocked;


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
	ros::Subscriber subscriberLocation;
	ros::Subscriber subscriberlockStatus;
	ros::Subscriber subscriberUnlock;

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
    bool gotoPosition(double x,double y);
	PathPlanner pathPlanner;
	string currentNode;
	int currentNodeIndex;

    void checkKeyboardPress();
    bool modeSet();
    bool inMode(string mode);
    void toggleMode(string mode);
};


#endif // #ifndef SE306P1_ACTOR_ACTOR_H_DEFINED

