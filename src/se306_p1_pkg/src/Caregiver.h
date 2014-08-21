#ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
#define SE306P1_ACTOR_CAREGIVER_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"

class Caregiver : public Visitor
{
protected:
    virtual void doInitialSetup();
    Virtual void doExecuteLoop();
};


#endif  // #ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
