#ifndef SE306P1_ACTOR_DOCTOR_H_DEFINED
#define SE306P1_ACTOR_DOCTOR_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"

class Doctor : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();


};


#endif