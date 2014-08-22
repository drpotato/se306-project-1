#ifndef SE306P1_ACTOR_RCACTOR_H_DEFINED
#define SE306P1_ACTOR_RCACTOR_H_DEFINED

#include "Actor.h"

class RCActor : public Actor
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
};


#endif // #ifndef SE306P1_ACTOR_RCACTOR_H_DEFINED

