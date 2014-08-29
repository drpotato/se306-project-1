#ifndef SE306P1_ACTOR_DOG_H_DEFINED
#define SE306P1_ACTOR_DOG_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"

class Dog : public Visitor
{

protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
	string getActorName();

};


#endif // #ifndef SE306P1_ACTOR_DOG_H_DEFINED

