#ifndef SE306P1_ACTOR_DOCTOR_H_DEFINED
#define SE306P1_ACTOR_DOCTOR_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Health.h>
#include "ActorSpawner.h"

class Doctor : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  void emergency();

  static void healthCallback(msg_pkg::Health msg);

  ros::Subscriber subscriberHealth;

  int8_t healthLevel;


};


#endif