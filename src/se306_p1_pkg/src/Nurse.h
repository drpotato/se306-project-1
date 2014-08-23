#ifndef SE306P1_ACTOR_NURSE_H_DEFINED
#define SE306P1_ACTOR_NURSE_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"
#include <msg_pkg/Nurse.h>

class Nurse : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  ros::Subscriber subscriberNurse;

  static void nurseCallback(msg_pkg::Nurse msg);

  bool assist;

};


#endif