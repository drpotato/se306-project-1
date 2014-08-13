#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include <nav_msgs/Odometry.h>

#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"

class Actor
{
public:
        Actor();
	virtual ~Actor();

	void initialSetup(unsigned int robotID);
	bool executeLoop();
	
	void initialSetupStage();

	//void StageOdom_callback();
	void executeLoopStageSubscription();
	void executeLoopStagePublication();
        
        static void StageOdom_callback(nav_msgs::Odometry msg);
       

protected:
	void locationCallback(const std_msgs::String::ConstPtr& msg);
	virtual void doInitialSetup() = 0;
	virtual void doExecuteLoop() = 0;

	//velocity of the robot
	double velLinear;
	double velRotational;

	//pose of the robot
	double px;
	double py;
	double theta;

	//id of the robot
	//string robotidentification;
	
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

