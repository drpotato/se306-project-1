#ifndef SE306P1_ACTOR_COMPANIONROBOT_H_DEFINED
#define SE306P1_ACTOR_COMPANIONROBOT_H_DEFINED

#include "Robot.h"
#include <msg_pkg/Morale.h>

class CompanionRobot : public Robot
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();

	bool checkMoraleLevel();

	static void moraleCallback(msg_pkg::Morale msg);
	string getActorName();

	int8_t moraleLevel;
	bool giving_morale;

	ros::Subscriber subscriberMorale;

	string residentName;
	int y;
	int x;
	bool first;
	bool first_call;
	bool returningHome;
	bool returningHome_first;
};

#endif
