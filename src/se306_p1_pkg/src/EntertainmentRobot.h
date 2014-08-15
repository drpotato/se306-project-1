#ifndef SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED
#define SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED

#include "Robot.h"

#include <msg_pkg/Entertainedness.h>


class EntertainmentRobot : public Robot
{
protected:
	virtual void doInitialSetup(); 
	virtual void doExecuteLoop();

	bool checkEntertainmentLevel();

	static void entertainednessCallback(msg_pkg::Entertainedness msg);

	int8_t entertainednessLevel;
	bool entertaining;

	ros::Subscriber subscriberEntertainedness;

	string residentName;
	int y;
	int x;
	bool first;
	bool first_call;
	bool returningHome;
	bool returningHome_first;
};

#endif