#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"

class Resident : public Human
{
protected:
        virtual void doInitialSetup();
        virtual void doExecuteLoop();
};


#endif

