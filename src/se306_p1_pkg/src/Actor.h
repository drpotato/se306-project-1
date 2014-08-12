#ifndef SE306P1_ACTOR_ACTOR_H_DEFINED
#define SE306P1_ACTOR_ACTOR_H_DEFINED

#include "ros/ros.h"
#include <string>

class Actor
{
public:
	Actor();
	virtual ~Actor();

	void initialSetup(unsigned int robotID);
	bool executeLoop();
	
	void initialSetupStage();
	void executeLoopStageSubscription();
	void executeLoopStagePublication();

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

	ros::Publisher location_pub;
	
	std::string rosName;
	std::string stageName;
};


#endif // #ifndef SE306P1_ACTOR_ACTOR_H_DEFINED

