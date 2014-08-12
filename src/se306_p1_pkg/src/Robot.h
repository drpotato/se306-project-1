#ifndef SE306P1_ACTOR_ROBOT_H_DEFINED
#define SE306P1_ACTOR_ROBOT_H_DEFINED

#include "Actor.h"

class Robot : public Actor
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
};


#endif // #ifndef SE306P1_ACTOR_ROBOT_H_DEFINED

