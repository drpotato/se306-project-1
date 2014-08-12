#ifndef SE306P1_ACTOR_HUMAN_H_DEFINED
#define SE306P1_ACTOR_HUMAN_H_DEFINED

#include "Actor.h"

class Human : public Actor
{
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
};


#endif // #ifndef SE306P1_ACTOR_HUMAN_H_DEFINED

