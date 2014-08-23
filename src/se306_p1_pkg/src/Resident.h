#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include "ros/ros.h"
#include <ctime>
#include <time.h>
#include <msg_pkg/Time.h>

class Resident : public Human
{
public:
  virtual bool isLocked();
  virtual void lock();
  virtual void unlock();
  
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg);
  static void timeCallback(msg_pkg::Time msg);
  
  bool lock_;

  // Demo paramters to gradually reduce levels
  int morale_count_;
  int socialness_count_;
  bool m_dropped_;
  bool s_dropped_;
  const static int WAIT_TIME = 50;
  bool m_replenished_;

  // Event hours - c++ inverts 24hr time for some reason? - maybe just my machine does this
  const static int WAKE_TIME = 19;
  const static int BREAKFAST_TIME = 20;
  const static int LUNCH_TIME = 1;
  const static int DINNER_TIME = 6;
  const static int SLEEP_TIME = 11;

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

  // Subscriber for interactions
  ros::Subscriber subscriberInteraction;
  // Subscriber for time
  ros::Subscriber subscriberTime;

  // Gets a new level with a maximum of 5 and minimum of 1
  static int getNewLevel(int amount, int oldValue);

  // Stop the robots angular velocity
  void stopRobotSpinning();

  // Daily events
  void wakeUp();
  void eat();
  void goToSleep();
};


#endif