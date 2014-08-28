#ifndef SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED
#define SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED

#include "Robot.h"
#include <msg_pkg/Morale.h>
#include <msg_pkg/LockStatus.h>

class EntertainmentRobot : public Robot
{
protected:
	virtual void doInitialSetup(); 
	virtual void doExecuteLoop();

	bool checkMoraleLevel();

	static void moraleCallback(msg_pkg::Morale msg);
	static void lockStatusCallback(msg_pkg::LockStatus msg);

	string getActorName();

	int8_t moraleLevel;
	bool entertaining;

	ros::Subscriber subscriberMorale;
	ros::Subscriber subscriberLockStatus;

	string residentName;
	int y;
	int x;
	bool first;
	bool returningHome;
	bool returningHome_first;
	bool waiting_to_entertain;
	bool has_lock;
};

#endif