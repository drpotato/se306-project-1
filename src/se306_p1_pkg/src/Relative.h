#ifndef SE306P1_ACTOR_RELATIVE_H_DEFINED
#define SE306P1_ACTOR_RELATIVE_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Socialness.h>
#include "ActorSpawner.h"

class Relative : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();

  bool checkSocialnessLevel();

  static void socialnessCallback(msg_pkg::Socialness msg);

  int8_t socialnessLevel;
  bool socialising;

  ros::Subscriber subscriberSocialness;

  string residentName;
  int y;
  int x;
  bool first;
  bool first_call;
  bool returningHome;
  bool returningHome_first;
};


#endif
