#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include "ros/ros.h"

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

  // Demo paramters to graduall reduce levels
  int entertainment_count_;
  int socialness_count_;
  bool e_dropped_;
  bool s_dropped_;

  // Level of social fulfillment: 
  // 1 - bad (lonely)
  // 5 - good (not lonely)
  int socialness_level_;

  // Level of entertainedness:
  // 1 - bad (bored)
  // 5 - good (entertained)	
  int entertainedness_level_;

  // Publisher for socialness
  ros::Publisher publisherSocialness;
  // Publisher for entertainedness
  ros::Publisher publisherEntertainedness;

  // Subscriber for interactions
  ros::Subscriber subscriberInteraction;

  static int getNewLevel(int amount, int oldValue);
};


#endif

