#ifndef SE306P1_ACTOR_R0_H_DEFINED
#define SE306P1_ACTOR_R0_H_DEFINED

#include "Robot.h"


class R0 : public Robot
{
protected:
	virtual void doInitialSetup(); 
	virtual void doExecuteLoop();
    int status;
   


};

#endif