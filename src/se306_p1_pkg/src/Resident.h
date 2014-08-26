#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/RequestLock.h>
#include <msg_pkg/LockStatus.h>
#include "ros/ros.h"
#include <ctime>
#include <time.h>
#include "Actor.h"

class Resident : public Human
{
public:
  virtual bool isLocked();
  virtual void lock(ActorType type, string id);
  virtual void unlock();
  
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg);
  static void timeCallback(msg_pkg::Time msg);
  static void requestLockCallback(msg_pkg::RequestLock msg);
  
  bool lock_;
  ActorType lock_type_;
  string lock_id_;

  // Demo paramters to gradually reduce levels
  int morale_count_;
  int socialness_count_;
  bool m_dropped_;
  bool s_dropped_;
  const static int WAIT_TIME = 50;
  bool m_replenished_;

  // Event hours
  const static int WAKE_TIME = 7;
  const static int BREAKFAST_TIME = 8;
  const static int LUNCH_TIME = 13;
  const static int DINNER_TIME = 18;
  const static int SLEEP_TIME = 23;

  // Has done event values
  bool has_eaten_breakfast_;
  bool has_eaten_lunch_;
  bool has_eaten_dinner_;
  bool has_woken_;
  bool has_gone_to_bed_;

  // Level of social fulfillment: 
  // 1 - bad (lonely)
  // 5 - good (not lonely)
  int socialness_level_;

  // Level of morale:
  // 1 - bad (bored)
  // 5 - good (entertained)	
  int morale_level_;

  // Publisher for socialness
  ros::Publisher publisherSocialness;
  // Publisher for morale
  ros::Publisher publisherMorale;
  // Publisher for lock status
  ros::Publisher publisherLockStatus;

  // Subscriber for interactions
  ros::Subscriber subscriberInteraction;
  // Subscriber for time
  ros::Subscriber subscriberTime;
  // Subscriber for requesting a lock
  ros::Subscriber subscriberRequestLock;


  // Gets a new level with a maximum of 5 and minimum of 1
  static int getNewLevel(int amount, int oldValue);

  // Stop the robots angular velocity
  void stopRobotSpinning();

  // Daily events
  void wakeUp();
  void eat(int hour);
  void goToSleep();
  bool hasWoken();

  // Enum conversions
  ActorType getActorTypeFromString(string actorType);
  string getStringFromActorType(ActorType actorType);
};


#endif