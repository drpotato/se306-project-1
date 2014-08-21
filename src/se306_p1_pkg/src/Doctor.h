#ifndef SE306P1_ACTOR_DOCTOR_H_DEFINED
#define SE306P1_ACTOR_DOCTOR_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Telephone.h>
#include "ActorSpawner.h"

class Doctor : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  void emergency();

  static void telephoneCallback(msg_pkg::Telephone msg);

  ros::Subscriber subscriberHealth;

  bool homeVisit;



};


#endif