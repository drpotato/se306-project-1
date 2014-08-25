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

	// Enum for different levels of resident
  enum Level
  {
		MORALE,
		SOCIALNESS,
		HYGIENE,
		HUNGER,
		THIRST,
		HEALTH,
		FITNESS
	};

  virtual bool isLocked();
  virtual void lock();
  virtual void unlock();
  
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg); //alpha
  static float getRandom(float, float);
  void changeLevel(float, Level); 
  void randomEventLoop();
  
  bool lock_;

	// Level variables
	int morale_level_;
  int socialness_level_;
	int health_level_;
	int hygiene_level_;
	int hunger_level_;
	int thirst_level_;
	int fitness_level_;

  // Demo paramters to gradually reduce levels (deprecated)
  int morale_count_;
  int socialness_count_;

  const static int WAIT_TIME;  
	const static int LEVEL_MAX; // Final release should be 100
	const static float FREQUENCY;

	// Deprecated (alpha) - will be replaced by 'm_changed_' for example
  bool m_replenished_;	
  bool m_dropped_;
  bool s_dropped_;

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

	// Publishers for all attributes/levels  
  	ros::Publisher publisherMorale;
	ros::Publisher publisherSocialness;
	ros::Publisher publisherHealth;
	ros::Publisher publisherHygiene;
	ros::Publisher publisherHunger;
	ros::Publisher publisherThirst;
	ros::Publisher publisherFitness;

  // Subscriber for interactions
  ros::Subscriber subscriberInteraction;

  // Gets a new level with a maximum of MAX_LEVEL and minimum of 0
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
