#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include <nav_msgs/Odometry.h>
#include <msg_pkg/Location.h>

#include "ros/ros.h"
#include <string.h>
#include "std_msgs/String.h"

class Actor
{
public:
        Actor();
	virtual ~Actor();

	void initialSetup(unsigned int robotID);
	bool executeLoop();
	
	void initialSetupStage();

	void publishLocation();

	void executeLoopStageSubscription();
	void executeLoopStagePublication();
        
        static void StageOdom_callback(nav_msgs::Odometry msg);
        static void locationCallback(msg_pkg::Location msg);
       

protected:
	
	virtual void doInitialSetup() = 0;
	virtual void doExecuteLoop() = 0;

	//velocity of the robot
	double velLinear;
	double velRotational;

	//pose of the robot
	double px;
	double py;
	double theta;
	
	// ROS-specific stuff
	ros::NodeHandle *nodeHandle;
	ros::Rate *loopRate;

	ros::Publisher  publisherStageVelocity;
	ros::Subscriber subscriberStageOdometry;
	ros::Subscriber subscriberStageLaserScan;

	ros::Subscriber subscriberLocation;
	ros::Publisher publisherLocation;
	
	std::string rosName;
	std::string stageName;
};


#endif // #ifndef SE306P1_ACTOR_ACTOR_H_DEFINED

