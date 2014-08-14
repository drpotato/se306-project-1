#ifndef SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED
#define SE306P1_ACTOR_ENTERTAINMENTROBOT_H_DEFINED

#include "Robot.h"


class EntertainmentRobot : public Robot
{
protected:
	virtual void doInitialSetup(); 
	virtual void doExecuteLoop();
};

#endif