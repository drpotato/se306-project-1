#ifndef SE306P1_ACTOR_FRIEND_H_DEFINED
#define SE306P1_ACTOR_FRIEND_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/LockStatus.h>
#include <msg_pkg/Telephone.h>
#include "ActorSpawner.h"

class Friend : public Visitor
{
private:
  static Friend* getFriendInstance();
protected:
  /* Hooks */
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  
  /* Callbacks */
  static void socialnessCallback(msg_pkg::Socialness msg);
  static void moraleCallback(msg_pkg::Morale msg);
  static void timeCallback(msg_pkg::Time msg);
  static void telephoneCallback(msg_pkg::Telephone msg);
  
  /* Subscribers */
  ros::Subscriber subscriberSocialness;
  ros::Subscriber subscriberMorale;
  ros::Subscriber subscriberTime;
  ros::Subscriber subscribeTelephone;

  string getActorName();

  int8_t socialnessLevel;
  bool socialising;

  int y;
  int x;
  bool first;
  bool first_call;
  bool returningHome;
  bool returningHome_first;
  bool waiting_to_socialise;
  bool called_by_resident_;
  
  bool has_lock;
};


#endif
