#ifndef SE306P1_ACTOR_VISITORCONTROLLER_H_DEFINED
#define SE306P1_ACTOR_VISITORCONTROLLER_H_DEFINED

#include "Actor.h"

class VistorController : public Actor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
};

#endif

