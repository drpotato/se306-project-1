#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/RequestLock.h>
#include <msg_pkg/LockStatus.h>
#include <msg_pkg/Unlock.h>
#include "ros/ros.h"
#include <ctime>
#include <time.h>
#include "Actor.h"

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
  virtual void lock(ActorType type, string id);
  virtual void unlock(std::string robot_id);
  
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  static void interactionCallback(msg_pkg::Interaction msg); //alpha
  static float getRandom(float, float);
  int changeLevel(float, Level); 
  void randomEventLoop();

  static void timeCallback(msg_pkg::Time msg);
  static void requestLockCallback(msg_pkg::RequestLock msg);
  static void unlockCallback(msg_pkg::Unlock msg);
  
  bool lock_;
  ActorType lock_type_;
  string lock_id_;

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
	const static float LEVEL_MAX; // Final release should be 100
	const static float LEVEL_MIN; // Final release should be 0
	const static float FREQUENCY;

	// Deprecated (alpha) - will be replaced by 'm_changed_' for example
  bool m_replenished_;	
  bool m_dropped_;
  bool s_dropped_;

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

  // Publisher for lock status
  ros::Publisher publisherLockStatus;
  // Publisher for telephone
  ros::Publisher publisherTelephone;

  // Subscriber for interactions
  ros::Subscriber subscriberInteraction;
  // Subscriber for time
  ros::Subscriber subscriberTime;
  // Subscriber for requesting a lock
  ros::Subscriber subscriberRequestLock;
  // Subscriber for unlocking
  ros::Subscriber subscriberUnlock;


  // Gets a new level with a maximum of MAX_LEVEL and minimum of 0
  static int getNewLevel(int amount, int oldValue);

  // Stop the robots angular velocity
  void stopRobotSpinning();

  // Daily events
  void wakeUp();
  void eat(int hour);
  void goToSleep();
  bool hasWoken();

  // Phone call
  void call(string personType);

  // Enum conversions
  ActorType getActorTypeFromString(string actorType);
  string getStringFromActorType(ActorType actorType);
};


#endif
