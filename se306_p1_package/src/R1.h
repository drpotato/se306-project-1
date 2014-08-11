#ifndef SE306P1_ACTOR_R1_H_DEFINED
#define SE306P1_ACTOR_R1_H_DEFINED

#include "Robot.h"


class R1 : public Robot
{
protected:
	virtual void doInitialSetup(); 
	virtual void doExecuteLoop();
};

#endif