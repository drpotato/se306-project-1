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
  static void interactionCallback(msg_pkg::Interaction msg); //alpha
  static float getRandom(float, float);
  int changeLevel(float, Level); 
	void randomLevelChange(float, float, Level);
  void randomEventLoop();

	unsigned int seed;
	FILE *urandom;

  static void timeCallback(msg_pkg::Time msg);
  static void requestLockCallback(msg_pkg::RequestLock msg);
  static void unlockCallback(msg_pkg::Unlock msg);

  string getActorName();
  bool lock_;

	#define LEVEL_MAX 100 // Final release should be 100
	#define LEVEL_MIN 0 // Final release should be 0
	#define FREQUENCY 10

  // Demo paramters to gradually reduce levels (deprecated)
  int morale_count_;
  int socialness_count_;

  const static int WAIT_TIME;  
	const static float LEVEL_MAX; // Final release should be 100
	const static float LEVEL_MIN; // Final release should be 0
	const static float FREQUENCY;;

	// Deprecated (alpha) - will be replaced by 'm_changed_' for example
  bool m_replenished_;	
  bool m_dropped_;
  bool s_dropped_;

	#define LEVEL_MAX 100 // Final release should be 100
	#define LEVEL_MIN 0 // Final release should be 0
	#define FREQUENCY 10

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
  void eat(int hour);
  void goToSleep();
  bool hasWoken();
};


#endif
