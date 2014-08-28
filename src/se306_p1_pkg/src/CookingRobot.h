#ifndef SE306P1_ACTOR_COOKINGROBOT_H_DEFINED
#define SE306P1_ACTOR_COOKINGROBOT_H_DEFINED

#include "Robot.h"
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Time.h>

class CookingRobot : public Robot
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();

	static void hungerCallback(msg_pkg::Hunger msg);
	static void timeCallback(msg_pkg::Time msg);
	string getActorName();

	bool cooking;
	bool moving_to_stove;
	ros::Subscriber subscriberHunger;
	ros::Subscriber subscriberTime;

	string residentName;
	int y;
	int x;
	bool first;
	bool first_call;
	bool returningHome;
	bool returningHome_first;
};

#endif
