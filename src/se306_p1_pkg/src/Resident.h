#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include "ros/ros.h"
#include <ctime>

class Resident : public Human
{
public:
  virtual bool isLocked();
  virtual void lock();
  virtual void unlock();
  
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg);
  
private:
  bool lock_;

  // Demo paramters to gradually reduce levels
  int morale_count_;
  int socialness_count_;
  bool m_dropped_;
  bool s_dropped_;
  const static int WAIT_TIME = 50;
  bool m_replenished_;

  // Stores the time of day in the Ultron world
  std::time_t time_of_day;
  // Stores the number of seconds needed to add to time_of_day upon each loop
  double seconds_to_add;

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

  static int getNewLevel(int amount, int oldValue);
  int second_increase_per_loop();
  void stopRobotSpinning();
};


#endif

