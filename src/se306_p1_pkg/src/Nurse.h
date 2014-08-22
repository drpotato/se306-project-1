#ifndef SE306P1_ACTOR_NURSE_H_DEFINED
#define SE306P1_ACTOR_NURSE_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"

class Nurse : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();


};


#endif