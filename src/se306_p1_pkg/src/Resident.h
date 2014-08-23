#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include "ros/ros.h"
#include <ctime>
#include <time.h>

// Typedef for dbl-precision printing in randomEventLoop()
typedef std::numeric_limits< double > dbl;
// Uncomment this to use Conor's very high-quality (but sometimes blocking) PRNG stream.
//#define USE_DEV_RANDOM

class Resident : public Human
{
public:
  virtual bool isLocked();
  virtual void lock();
  virtual void unlock();
  
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg);
  static float getRandom(float, float);
  void changeLevel(int, char);
  void randomEventLoop();
  
  bool lock_;

  // Demo paramters to gradually reduce levels
  int morale_count_;
  int socialness_count_;
  bool m_dropped_;
  bool s_dropped_;
  const static int WAIT_TIME = 50;
  const static float FREQUENCY = 10;
  bool m_replenished_;

  // Stores the time of day in the Ultron world
  std::time_t time_of_day;
  // Gets updated separate minutes, seconds etc values from the time_of_day
  struct tm *time_of_day_values;
  // Stores the number of seconds needed to add to time_of_day upon each loop
  double seconds_to_add;
  int the_hour;

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

	// Delay measurement variables
	long long msAtPreviousLoop;
	float randNum;


#ifdef USE_DEV_RANDOM
	// Randomness variables	
	int randomData;
	int myRandomInteger;
	size_t randomDataLen;
#endif    

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

  // Gets a new level with a maximum of 5 and minimum of 1
  static int getNewLevel(int amount, int oldValue);

  // Number of seconds to increase Ultron world time by on each ROS loop
  int secondIncreasePerLoop();

  // Stop the robots angular velocity
  void stopRobotSpinning();

  // Daily events
  void wakeUp();
  void eat();
  void goToSleep();
};


#endif
