#ifndef SE306P1_ACTOR_COOKINGROBOT_H_DEFINED
#define SE306P1_ACTOR_COOKINGROBOT_H_DEFINED

#include "Robot.h"
#include <msg_pkg/Hunger.h>

class CookingRobot : public Robot
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();

	bool checkHungerLevel();

	static void hungerCallback(msg_pkg::Hunger msg);
	string getActorName();

	int8_t hungerLevel;
	bool cooking;

	ros::Subscriber subscriberHunger;

	string residentName;
	int y;
	int x;
	bool first;
	bool first_call;
	bool returningHome;
	bool returningHome_first;
};

#endif
