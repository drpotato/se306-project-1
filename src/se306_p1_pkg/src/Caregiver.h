#ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
#define SE306P1_ACTOR_CAREGIVER_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"
#include <msg_pkg/Fitness.h>
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Hygiene.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Socialness.h>

class Caregiver : public Visitor
{
protected:
    virtual void doInitialSetup();
    Virtual void doExecuteLoop();
    
    
    ros::Subscriber subscriberFitness;
};


#endif  // #ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
