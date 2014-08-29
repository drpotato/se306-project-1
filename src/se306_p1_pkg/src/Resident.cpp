#include "Resident.h"
#include <string.h>
#include <time.h> // seed for rand()
#include <stdlib.h> // rand()
#include <limits> // Get information about max double/float prec.

#ifdef USE_DEV_RANDOM
	#include <fcntl.h> // File access for RNG
#else
	#include <cstdlib>
	#include <ctime>
#endif

// Debug lines for randomness component of code

#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Health.h>
#include <msg_pkg/Hygiene.h>
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Fitness.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/RequestLock.h>
#include <msg_pkg/Telephone.h>
#include "Actor.h"
#include "ActorSpawner.h"
#include <ctime>
#include <time.h>

string Resident::getActorName()
{
  return "Resident";
}

// The person living in our house. 
// Has various attributes representing his needs/wants, which degrade over time.
// When they reach a certain level, messages are published to his assistant Robots and the VisitorController, requesting various services.
void Resident::doInitialSetup()
{
	
	// /dev/urandom
	urandom = fopen("/dev/urandom", "r");
	fread(&seed, sizeof (seed), 1, urandom);
	#ifdef NON_RANDOM
	srand(0);
	#else
	srand(seed);
	#endif

  velLinear = 0;
  velRotational = 0.0;

  // Initially the resident is not locked.
  // Any other Actor can interact with him and acquire the lock.
  lock_ = false;

  // Initialise statuses
  has_eaten_breakfast_ = false;
  has_eaten_lunch_ = false;
  has_eaten_dinner_ = false;
  has_woken_ = hasWoken();
  has_gone_to_bed_ = !hasWoken();

  getOutOfBed = false;
  goToEatingPlace = false;
  getIntoBed = false;

  // Set levels to maximum initially.
  morale_level_ = LEVEL_MAX;
  socialness_level_ = LEVEL_MAX;
  health_level_ = LEVEL_MAX;
  hygiene_level_ = LEVEL_MAX;
  hunger_level_ = LEVEL_MAX;
  fitness_level_ = LEVEL_MAX;

  // Set up publishers.
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherMorale = nodeHandle->advertise<msg_pkg::Morale>("morale", 1000);
  publisherHygiene = nodeHandle->advertise<msg_pkg::Hygiene>("hygiene", 1000);
  publisherHunger = nodeHandle->advertise<msg_pkg::Hunger>("hunger", 1000);
  publisherHealth = nodeHandle->advertise<msg_pkg::Health>("health", 1000);
  publisherFitness = nodeHandle->advertise<msg_pkg::Fitness>("fitness", 1000);

  publisherLockStatus = nodeHandle->advertise<msg_pkg::LockStatus>("lockStatus", 1000);
  publisherTelephone = nodeHandle->advertise<msg_pkg::Telephone>("telephone", 1000);

  // Set up subscriptions.
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Resident::timeCallback);
  subscriberRequestLock = nodeHandle->subscribe("requestLock", 1000, Resident::requestLockCallback);
  subscriberUnlock = nodeHandle->subscribe("unlock", 1000, Resident::unlockCallback);
  
  called_friend_today_ = false;
}

void Resident::doExecuteLoop()
{  
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  if (residentInstance->RCmode == "resident")
  {
    residentInstance->controlRobot();
  }
  else if (getOutOfBed)
  {
    if (!goToNode("nodeBedroomCentre"))
    {
      getOutOfBed = false;
    }
  }
  else if (getIntoBed)
  {
    if (!goToNode("nodeMasterBed"))
    {
      getIntoBed = false;
    }
  }
  else if (goToEatingPlace)
  {
    if (!goToNode("nodeLivingRoomEatingPlace"))
    {
      goToEatingPlace = false;
    }
  }
  
  /* Call a friend if socialness gets too low but only call once per day */
  if (residentInstance->socialness_level_ <= 1 && !called_friend_today_)
  {
    call("friend");
    called_friend_today_ = true;
  }

  if (health_level_ <= 50)
  {
    call("doctor");
  }

  residentInstance->randomEventLoop();
}

// PHONE CALLING -------------------------------------------------------------------------------------------------//
/*
 * personType should be one of the following: doctor, relative, friend (are there more???) (note the lowercase)
 */ 
void Resident::call(string personType)
{
  msg_pkg::Telephone phonecall;
  phonecall.contact = personType;
  publisherTelephone.publish(phonecall);
}


// INTERACTION RELATED --------------------------------------------------------------------------------------------//
/*
 * Upon receiving a message published to the 'interaction' topic, respond appropriately.
 */

void Resident::interactionCallback(msg_pkg::Interaction msg)
{
	
  std::string attribute = msg.attribute;
  int amount = msg.amount;

  // Get the class instance
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 1.0; // Rotate to show is being interacted with

  if (attribute == "socialising")
  {
    // Get new level
    residentInstance->changeLevel(amount, SOCIALNESS);

    if (residentInstance->socialness_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  } 
  else if (attribute == "entertaining")
  {
    residentInstance->changeLevel(amount, MORALE);

    if (residentInstance->morale_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  }
  else if (attribute == "showering")
  {
    residentInstance->changeLevel(amount, HYGIENE);

    if (residentInstance->hygiene_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  }
  else if (attribute == "medicating")
  {
    residentInstance->changeLevel(amount, HEALTH);

    if (residentInstance->health_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  }
  else if (attribute == "exercising")
  {
    residentInstance->changeLevel(amount, FITNESS);

    if (residentInstance->fitness_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  }
  else if (attribute == "eating")
  {
    residentInstance->changeLevel(amount, HUNGER);

    if (residentInstance->hunger_level_ == LEVEL_MAX)
    {
      residentInstance->stopRobotSpinning();
    }
  }
	else if (attribute =="go_to_bath")
	{
		// Make resident walk to bathroom door node
	}
}


// DAILY EVENTS -----------------------------------------------------------------------------------------------------//

void Resident::timeCallback(msg_pkg::Time msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

  // Check if its currently any event times
  if ((msg.hour == residentInstance->WAKE_TIME) && (!residentInstance->has_woken_))
  {
    // WAKE UP
    ROS_INFO("It is %d:00. Wake up!", msg.hour);
    residentInstance->wakeUp();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
  else if ( ((msg.hour == residentInstance->BREAKFAST_TIME) && (!residentInstance->has_eaten_breakfast_)) || ((msg.hour == residentInstance->LUNCH_TIME) && (!residentInstance->has_eaten_lunch_)) || ((msg.hour == residentInstance->DINNER_TIME) && (!residentInstance->has_eaten_dinner_)))
  {
    // Have some food
    ROS_INFO("It is %d:00 - time to eat!", msg.hour);
    residentInstance->eat(msg.hour);
  }
  else if ((msg.hour == residentInstance->SLEEP_TIME) && (!residentInstance->has_gone_to_bed_))
  {
    // Go to sleep
    ROS_INFO("It is %d:00. Sleep time!", msg.hour);
    residentInstance->goToSleep();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
}

void Resident::stopRobotSpinning()
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 0.0; // Stop rotation to show interaction finished
}




void Resident::actOnCriticalNeeds(Level level)
{
	Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

	// Stop the resident
	residentInstance->velRotational = 0.0;
	residentInstance->velLinear = 0.0;
	// Lock?

	if (level == SOCIALNESS) {
		if (!called_friend_today_) {
			// Get a number between 0 and 100.
			randNum = getRandom(0, 100);
			// If 0-25, call first relative
			if (randNum <= 25) {
				call("relative");
			} else if (randNum <= 50) {
				// If 25-50, call second relative
				call("relative");
			} else if (randNum <= 75) {
				// If 50-75, call first friend
				call("friend");
			} else {
				// If 75-100, call second friend
				call("friend");
			} 

		} // else relies on companion robot for socialness

	} else if (level == HEALTH) {
		call("doctor");
	}
}
		




// Function called for each iteration of 'doExecuteLoop()'; emulates random
// human behaviours and needs
void Resident::randomEventLoop()
{

	// DELAY TEST

	///////////////////////
	// Time measurement ///
	struct timeval t_e; 
    gettimeofday(&t_e, NULL); // get current time
    long long milliseconds = t_e.tv_sec * 1000LL + t_e.tv_usec / 1000; // calculate milliseconds
    //printf("milliseconds: %lld\n", milliseconds);
	
	//printf("time since last loop = %lldms\n", (milliseconds - msAtPreviousLoop));
	msAtPreviousLoop = milliseconds;
	///////////////////////


	int drop;
	
	// Description:
	// Entertainedness, morale, health, fitness, and hunger all
	// drop over time, but not linearly.
	
	// == IMPORTANT: Read if you are another person building/running this feature
	// At the moment, this is executed once for every 'doExecuteLoop()' iteration
	// The loop itself is executed every 100+/-10ms (10Hz)
	// This -probably- won't change, but any severe delays in the build that are 
	// introduced after this was initially implemented will affect this.
	// The times described below (in seconds) are running under the assumption
	// that each loop iteration takes 100ms to be executed (e.g. for Morale,
	// an average of 140 loop iterations should pass before the Morale level drops
	// by 1).
	// As we have (unofficially) set a day length at 12 minutes, hunger drop will
	// be based on this.

	// getRandom(float(0), float(14 * FREQUENCY))); explained:
	// The second argument to get random is the maximum value that the randomly
	// generated number can be. This number is the product of a) the amount of 
	// seconds expected (on average) before an event occurs and b) the frequency
	// of the system as a whole, which is seperately defined in Resident.h at the
	// moment (as 'FREQUENCY'). 
	// In this case, an event will occur whenever the random number is between 139
	// and 140 (14 * FREQUENCY {10} - 1), so 1/140 of the time. 140 loops = 14 seconds.

	// Socialness drops fastest and is most affected by randomness.
	// On average, it should drop by 1 every second
	// Every cycle, ANY and ALL levels may change
	randomLevelChange(1, -1, SOCIALNESS);
	
	// Morale also drops quickly but is less affected by randomness than
	// entertainedness.
	// On average, it should drop by 1 every 1.4 seconds
	randomLevelChange(1.4, -1, MORALE);

	// Health drops in two ways, either almost slowly and almost 
	// completely linearly or in a random, drastic fashion.
	// On average, it should drop by 1 every 2.5 seconds. 
	// As well as this, to simulate an 'emergency', the resident 
	// can face a sudden drop in health, which is rare (every 6 minutes
	// on average)
	randomLevelChange(2.5, -1, HEALTH);
	randomLevelChange(360, -95, HEALTH);

	// Hygiene
	// On average, it should drop by 1 every 1.5 seconds
	randomLevelChange(1.5, -1, HYGIENE);

	// Hunger drops almost completely linearly...
	// On average, it should drop by 1 every 3 seconds
	randomLevelChange(3, -1, HUNGER);

	// Fitness drops fairly slowly...
	// On average, it should drop by 1 every 4 seconds
	randomLevelChange(4, -1, FITNESS);
}

// Method used to randomly change resident levels
// Receives frequency and change as arguments
// Input how many seconds (on average) you want a level to change by as the 
// frequency argument, how much you want the level to change (can be positive or negative)
// as the magnitude argument and the level you want to change as the level argument.
// NOTE: To increase/decrease the variance of the change, simply adjust magnitude and
// frequency. For example: for attributes that should conform closely to a mean (i.e. be
// predictable) use a low magnitude and a high frequency. The reverse is true for attributes
// that need to change unpredictably. Each time you halve frequency and double magnitude,
// the mean change over time of the attribute in question remains the same, but the
// variance or 'unpredictability' will increase. This is important to note for some 
// attributes that need to conform tightly to a mean (such as hunger).
void Resident::randomLevelChange(float frequency, float magnitude, Level level) 
{
	randNum = getRandom(float(0), float(frequency * FREQUENCY));
	if (randNum > ((frequency * FREQUENCY) - 1)) {
		changeLevel(magnitude, level);
		std:string LevelNames[7] =  { "Morale", "Socialnesss", "Hygiene", "Health", "Hunger", "Thirst", "Fitness" };
		#ifdef DEBUG_CHANGE_LEVEL
		#ifdef DEBUG_RANDNUM_GENERATION
		ROS_INFO("EVENT: %s drop %.0f (rand = %.2f)\n", LevelNames[level].c_str(), magnitude, randNum);
		#else
		ROS_INFO("EVENT: %s drop %.0f\n", LevelNames[level].c_str(), magnitude);
		#endif
		#endif
	}
}



// Method used to change resident levels 
// Pass an int and a char to change a resident level
// Use example: (-1, 's') will reduce socialness by 1.
// IMPORTANT
// In changing a level, this will also force a message
// to be published in the appropriate topic (morale, soc. etc)
// Return the new value  
int Resident::changeLevel(float change, Level level) {

	// Get the current resident class instance
	Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

	// Change level, publish new level showing new level value
	if (level == MORALE) 
  {
    residentInstance->morale_level_ = residentInstance->getNewLevel(change, residentInstance->morale_level_);

		msg_pkg::Morale moraleMessage;
		moraleMessage.level = residentInstance->morale_level_;
		residentInstance->publisherMorale.publish(moraleMessage);	

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Morale: %d", residentInstance->morale_level_);
		return residentInstance->morale_level_;
		#endif

	} 
  else if (level == SOCIALNESS) 
  {
		residentInstance->socialness_level_ = residentInstance->getNewLevel(change, residentInstance->socialness_level_);

		msg_pkg::Socialness socialnessMessage;
		socialnessMessage.level = residentInstance->socialness_level_;
		residentInstance->publisherSocialness.publish(socialnessMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Socialness: %d", residentInstance->socialness_level_);
		return residentInstance->socialness_level_;
		#endif

	} 
  else if (level == HYGIENE) 
  {
		residentInstance->hygiene_level_ = residentInstance->getNewLevel(change, residentInstance->hygiene_level_);

		msg_pkg::Hygiene hygieneMessage;
		hygieneMessage.level = residentInstance->hygiene_level_;
		residentInstance->publisherHygiene.publish(hygieneMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Hygiene: %d", residentInstance->hygiene_level_);
		return residentInstance->hygiene_level_;
		#endif

	} 
  else if (level == HUNGER) 
  {
		residentInstance->hunger_level_ = residentInstance->getNewLevel(change, residentInstance->hunger_level_);

		msg_pkg::Hunger hungerMessage;
		hungerMessage.level = residentInstance->hunger_level_;
		residentInstance->publisherHunger.publish(hungerMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Hunger: %d", residentInstance->hunger_level_);
		return residentInstance->hunger_level_;
		#endif

	}  
  else if (level == FITNESS) 
  {
		residentInstance->fitness_level_ = residentInstance->getNewLevel(change, residentInstance->fitness_level_);

		msg_pkg::Fitness fitnessMessage;
		fitnessMessage.level = residentInstance->fitness_level_;
		residentInstance->publisherFitness.publish(fitnessMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Fitness: %d", residentInstance->fitness_level_);
		return residentInstance->fitness_level_;
		#endif

	} 
  else if (level == HEALTH) 
  {
		residentInstance->health_level_ = residentInstance->getNewLevel(change, residentInstance->health_level_);

		msg_pkg::Health healthMessage;
		healthMessage.level = residentInstance->health_level_;
		residentInstance->publisherHealth.publish(healthMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Health: %d", residentInstance->health_level_);
		return residentInstance->health_level_;
		#endif
	}  
}


// Alpha (deprecated)
int Resident::getNewLevel(int amount, int oldLevel)
{
  int newLevel = std::min(amount + oldLevel, LEVEL_MAX);
  return newLevel < LEVEL_MIN ? LEVEL_MIN : newLevel;
}

// Basic random number generator with min/range input,
// uses C's 'rand()'
float Resident::getRandom(float minimum, float range) {
	float rn = (float)rand() / (float)RAND_MAX;
    rn = minimum += (rn * range);
	return rn;
}


void Resident::wakeUp()
{
  // Reset sleep value for the day
  has_gone_to_bed_ = false;
  has_woken_ = true;
  // Get out of bed
  getOutOfBed = true;
}
void Resident::eat(int hour)
{
  goToEatingPlace = true;
  if (hour == BREAKFAST_TIME)
  {
    has_eaten_breakfast_ = true;
  }
  else if (hour == LUNCH_TIME)
  {
    has_eaten_lunch_ = true;
  }
    else if (hour == DINNER_TIME)
  {
    has_eaten_dinner_ = true;
  }
}
void Resident::goToSleep()
{
  getIntoBed = true;
  // Reset values for next day
  has_eaten_breakfast_ = false;
  has_eaten_breakfast_ = false;
  has_eaten_lunch_ = false;
  has_eaten_dinner_ = false;
  has_woken_ = false;
  has_gone_to_bed_ = true;
  called_friend_today_ = false;
}


bool Resident::hasWoken()
{
  std::time_t time_of_day = std::time(0);
  int hour = gmtime(&time_of_day)->tm_hour;
  // Hours are inverted by 12 hours, so an hour value of 11 corresponds to 11pm while an hour value of 23 corresponds to 11am
  if ((hour < 11) && (hour > 6))
  {
    return true;
  }
  return false;
}


// LOCK RELATED -------------------------------------------------------------------------------------------------//
void Resident::requestLockCallback(msg_pkg::RequestLock msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

  if (residentInstance->isLocked())
  {

    Resident::ActorType type = residentInstance->getActorTypeFromString(msg.actor_name);

    if (type > residentInstance->lock_type_)
    {
      // The robot requesting the lock has a higher priority than the current one. You can have the lock!
      
      // REMOVE LOCK FROM CURRENT ROBOT
      residentInstance->unlock(residentInstance->lock_id_);

      // SET NEW LOCK
      residentInstance->lock((residentInstance->getActorTypeFromString(msg.actor_name)), msg.robot_id);
    }
    else
    {
      // The robot with the lock has the same or higher priority than the one requesting it, you cannot have the lock
      msg_pkg::LockStatus lockStatusMessage;
      lockStatusMessage.robot_id = msg.robot_id;
      lockStatusMessage.has_lock = false;
      residentInstance->publisherLockStatus.publish(lockStatusMessage);
    }
  }
  else
  {
    // No one has the lock, give it to the requester
    residentInstance->lock(residentInstance->getActorTypeFromString(msg.actor_name), msg.robot_id);
  }
}

/*
 * Unlock the resident when receiving an unlock callback
 */
void Resident::unlockCallback(msg_pkg::Unlock msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->unlock(msg.robot_id);
}

/*
 * A robot should not be able to interact with the resident if the resident is currently locked.
 */
bool Resident::isLocked()
{
  return lock_;
}

/*
 * Robots should only lock the resident if the resident is currently not locked
 */
void Resident::lock(ActorType type, string id)
{
  lock_ = true;
  lock_type_ = type;
  lock_id_ = id;

  msg_pkg::LockStatus lockStatusMessage;

  lockStatusMessage.robot_id = id;
  lockStatusMessage.has_lock = true;

  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->publisherLockStatus.publish(lockStatusMessage);
}

/*
 * Robots should unlock the resident after interation so that another robot may interact with the resident.
 */
void Resident::unlock(string robot_id)
{
  lock_ = false;
  msg_pkg::LockStatus lockStatusMessage;
  lockStatusMessage.robot_id = robot_id;
  lockStatusMessage.has_lock = false;
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->publisherLockStatus.publish(lockStatusMessage);
}



// HELPER FUNCTIONS ---------------------------------------------------------------------------//
Resident::ActorType Resident::getActorTypeFromString(string actorType)
{
  if (actorType == "Doctor")
  {
    return Doctor;
  }
  else if (actorType == "Nurse")
  {
    return Nurse;
  }
  else if (actorType == "Caregiver")
  {
    return Caregiver;
  }
  else if (actorType == "Visitor")
  {
    return Visitor;
  }
  else if (actorType == "Robot")
  {
    return Robot;
  }
}
string Resident::getStringFromActorType(ActorType actorType)
{
  if (actorType == Doctor)
  {
    return "Doctor";
  }
  else if (actorType == Nurse)
  {
    return "Nurse";
  }
  else if (actorType == Caregiver)
  {
    return "Caregiver";
  }
  else if (actorType == Visitor)
  {
    return "Visitor";
  }
  else if (actorType == Robot)
  {
    return "Robot";
  }
}
