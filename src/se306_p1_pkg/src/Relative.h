#ifndef SE306P1_ACTOR_RELATIVE_H_DEFINED
#define SE306P1_ACTOR_RELATIVE_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Socialness.h>
#include "ActorSpawner.h"

class Relative : public Visitor
{

public:
  
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();

  bool checkSocialnessLevel();

  static void socialnessCallback(msg_pkg::Socialness msg);

  bool interacting;
  bool first;
  bool first_call;

  //int status;
  int8_t residentSocialnessLevel;
  
private:
  // Variable for holding whether or not the relative
  // is interacting with the Resident actor
  
  ros::Subscriber subscriberSocialness;

};


#endif
