#ifndef SE306P1_ACTOR_MEDICATIONROBOT_H_DEFINED
#define SE306P1_ACTOR_MEDICATIONROBOT_H_DEFINED

#include "Robot.h"
#include <msg_pkg/Health.h>

class MedicationRobot : public Robot
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
	
	static void healthCallback(msg_pkg::Health msg);

	string getActorName();

	int8_t healthLevel;
	bool health;
	string residentName;

	ros::Subscriber subscriberHealth;

	bool first;
	bool first_call;
    bool healing;
    bool active;
	bool returningHome;
	bool returningHome_first;
	bool travellingToResident;
};

#endif
