#include "Resident.h"
#include <string.h>

/* Old RNG includes
#include <fcntl.h> // File access for RNG
#include <ctime> // as above
#include <limits> // Get information about max double/float prec.
*/

#include <time.h> // seed for rand()
#include <stdlib.h> // rand()
#include <limits> // Get information about max double/float prec.

#ifdef USE_DEV_RANDOM
	#include <fcntl.h> // File access for RNG
#else
	#include <cstdlib>
	#include <ctime>
#endif

// Debug lines for changeLevel()
//#define DEBUG_CHANGE_LEVEL

#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Health.h>
#include <msg_pkg/Hygiene.h>
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Thirst.h>
#include <msg_pkg/Fitness.h>
#include "Actor.h"
#include "ActorSpawner.h"
#include <ctime>
#include <time.h>
#include <msg_pkg/Time.h>

const float Resident::FREQUENCY = 10;
const int Resident::WAIT_TIME = 50;
const int Resident::LEVEL_MAX = 5;

// The person living in our house. 
// Has various attributes representing his needs/wants, which degrade over time.
// When they reach a certain level, messages are published to his assistant Robots and the VisitorController, requesting various services.
void Resident::doInitialSetup()
{


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

  // Set levels to maximum initially.
  morale_level_ = LEVEL_MAX;
  socialness_level_ = LEVEL_MAX;
  health_level_ = LEVEL_MAX;
  hygiene_level_ = LEVEL_MAX;
  hunger_level_ = LEVEL_MAX;
  thirst_level_ = LEVEL_MAX;
  fitness_level_ = LEVEL_MAX;

  //
  morale_count_ = 0;
  socialness_count_ = 0;


  m_dropped_ = false;
  m_replenished_ = false;

  // Set up publishers.
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherMorale = nodeHandle->advertise<msg_pkg::Morale>("morale", 1000);
  publisherHygiene = nodeHandle->advertise<msg_pkg::Hygiene>("hygiene", 1000);
  publisherHunger = nodeHandle->advertise<msg_pkg::Hunger>("hunger", 1000);
  publisherThirst = nodeHandle->advertise<msg_pkg::Thirst>("thirst", 1000);
  publisherHealth = nodeHandle->advertise<msg_pkg::Health>("health", 1000);
  publisherFitness = nodeHandle->advertise<msg_pkg::Fitness>("fitness", 1000);




  // Set up subscriptions.
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);
  subscriberTime = nodeHandle->subscribe("time", 1000, Resident::timeCallback);

  // Reset delay counter
  msAtPreviousLoop = 0;

#ifndef USE_DEV_RANDOM
  // Seed the PRNG
  std::srand(std::time(NULL));
#endif

}

void Resident::doExecuteLoop()
{
	//PathPlannerNode *target = this->pathPlanner.getNode(&node4Name);
  //vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
  //this->goToNode(path);

	//TODO: REMOVE THIS WHEN RANDOMNESS AND DAY LOGIC IS IMPLEMENTED

	if (morale_count_ >= WAIT_TIME && !m_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->morale_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			m_dropped_ = true;
		}
		else 
		{
			// Reduce the level every 1000 counts
			residentInstance->morale_level_--;
			// Create a socialness message to publish
			msg_pkg::Morale moraleMessage;
			// Assign current socialness level to the message
			moraleMessage.level = residentInstance->morale_level_;
			// Publish the message
			residentInstance->publisherMorale.publish(moraleMessage);
		}
		morale_count_ = 0;
	}
	else if (morale_count_ < WAIT_TIME && !m_dropped_) {
		morale_count_++;
	}

	else if (m_replenished_ && (socialness_count_ >= WAIT_TIME) && !s_dropped_) {
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->socialness_level_ <= 1) {
			// don't drop the value any more, it's being tended to or has been already
			s_dropped_ = true;
		}
		else {
			// reduce the level every 1000 counts
			residentInstance->socialness_level_--;
			//Create a socialness message to publish
			msg_pkg::Socialness socialnessMessage;
			//Assign current socialness level to the message
			socialnessMessage.level = residentInstance->socialness_level_;
			//Publish the message
			residentInstance->publisherSocialness.publish(socialnessMessage);
		}
		socialness_count_ = 0;
	}
	else if (m_replenished_ && (socialness_count_ < WAIT_TIME) && !s_dropped_)
	{
		socialness_count_++;
	}

// _REMOVE (when randomness implementation complete) 
////////////////////////////////////////////////////

	// Call random event for each iteration of the execution
	// loop (not sure if this should happen at start or end
	// of each loop; does this even matter?)

	randomEventLoop();
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
void Resident::lock()
{
  lock_ = true;
}

/*
 * Robots should unlock the resident after interation so that another robot may interact with the resident.
 */
void Resident::unlock()
{
  lock_ = false;
}

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
    int newLevel = getNewLevel(amount, residentInstance->socialness_level_);
  // Update the residents socialness level
  residentInstance->socialness_level_ = newLevel;
  //Create a socialness message to publish
  msg_pkg::Socialness socialnessMessage;
  //Assign current socialness level to the message
  socialnessMessage.level = newLevel;

  if (newLevel == LEVEL_MAX)
  {
    residentInstance->stopRobotSpinning();
  }

  //Publish the message
  residentInstance->publisherSocialness.publish(socialnessMessage);
  } 
  else if (attribute == "entertaining")
  {
  // Get new level
  int newLevel = getNewLevel(amount, residentInstance->morale_level_);
  // Update the residents socialness level
  residentInstance->morale_level_ = newLevel;
  //Create a socialness message to publish
  msg_pkg::Morale moraleMessage;
  //Assign current socialness level to the message
  moraleMessage.level = newLevel;

  if (newLevel == LEVEL_MAX)
  {
    residentInstance->stopRobotSpinning();
    residentInstance->m_replenished_ = true;
  }

  //Publish the message
  residentInstance->publisherMorale.publish(moraleMessage);
  }
  // TODO: put others in when implemented 
}


void Resident::timeCallback(msg_pkg::Time msg)
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());

  // Check if its currently any event times
  if ((msg.hour == residentInstance->WAKE_TIME) && (!residentInstance->has_woken_))
  {
    // WAKE THE FK UP
    ROS_INFO("It is %d:00. Wake up!", msg.hour);
    residentInstance->wakeUp();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
  else if ( ((msg.hour == residentInstance->BREAKFAST_TIME) && (!residentInstance->has_eaten_breakfast_)) || ((msg.hour == residentInstance->LUNCH_TIME) && (!residentInstance->has_eaten_lunch_)) || ((msg.hour == residentInstance->DINNER_TIME) && (!residentInstance->has_eaten_dinner_)))
  {
    // Here have some food
    ROS_INFO("It is %d:00 - time to eat!", msg.hour);
    residentInstance->eat(msg.hour);
  }
  else if ((msg.hour == residentInstance->SLEEP_TIME) && (!residentInstance->has_gone_to_bed_))
  {
    // Go to sleep yo
    ROS_INFO("It is %d:00. Sleep time!", msg.hour);
    residentInstance->goToSleep();
    ROS_INFO("%s", residentInstance->has_gone_to_bed_ ? "Sleeping" : "Awake");
  }
}


int Resident::getNewLevel(int amount, int oldLevel)
{
  int newLevel = std::min(amount + oldLevel, 5); // Can only have a maximum level of 5
  return newLevel < 1 ? 1 : newLevel; // Should not be below 1
}

void Resident::stopRobotSpinning()
{
  Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
  residentInstance->velRotational = 0.0; // Stop rotation to show interaction finished
}



// Function called for each iteration of 'doExecuteLoop()'; emulates random
// human behaviours and needs
void Resident::randomEventLoop()
{

	//ROS_INFO("Calculating random event(s)...\n");
	//ROS_INFO("System time: %d\n", msExpiredPrevious);
	
	// - Delay test 
	// Return time in milliseconds to check delay between
	// *full* ROS loops
	// KEEP THIS HERE - This time will change as more load is added
	// to roscore; randomEventLoop() will have to change in response to this


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
	// Entertainedness, morale, health, fitness, hunger and thirst all
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
	// be based on this, as will thirst.

	// getRandom(float(0), float(14 * FREQUENCY))); explained:
	// The second argument to get random is the maximum value that the randomly
	// generated number can be. This number is the product of a) the amount of 
	// seconds expected (on average) before an event occurs and b) the frequency
	// of the system as a whole, which is seperately defined in Resident.h at the
	// moment (as 'FREQUENCY'). 
	// In this case, an event will occur whenever the random number is between 139
	// and 140 (14 * FREQUENCY {10} - 1), so 1/140 of the time. 140 loops = 14 seconds.

	// TODO: Reduce code repetition by moving the first two lines of each section below
	// to the random method

	// Socialness drops fastest and is most affected by randomness.
	// On average, it should drop by 1 every second
	// Every cycle, ANY and ALL levels may change
	randNum = getRandom(float(0), float(1 * FREQUENCY));
	if (randNum > ((1 * FREQUENCY) - 1)) {
		changeLevel(-1, SOCIALNESS);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: socialness drop -1 (rand = %.3f)\n", randNum);
		#endif
	}
	
	// Morale also drops quickly but is less affected by randomness than
	// entertainedness.
	// On average, it should drop by 1 every 1.4 seconds
	randNum = getRandom(float(0), float(1.4 * FREQUENCY));
	if (randNum > ((1.4 * FREQUENCY) - 1)) {
		changeLevel(-1, MORALE);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: morale drop -1 (rand = %.3f)\n", randNum);
		#endif
	}

	// Health drops in two ways, either almost slowly and almost 
	// completely linearly or in a random, drastic fashion.
	// On average, it should drop by 1 every 2.5 seconds
	randNum = getRandom(float(0), float(2.5 * FREQUENCY));
	if (randNum > ((2.5 * FREQUENCY) - 1)) {
		changeLevel(-1, HEALTH);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: slow, near-linear hunger drop here (rand = %.3f)\n", randNum);
		#endif
	}	

	// Hygiene
	// On average, it should drop by 1 every 1.5 seconds
	randNum = getRandom(float(0), float(1.5 * FREQUENCY));
	if (randNum > ((1.5 * FREQUENCY) - 1)) {
		changeLevel(-1, HYGIENE);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: hygiene drop -1 (rand = %.3f)\n", randNum);
		#endif
	}

	// Hunger drops almost completely linearly...
	// On average, it should drop by 1 every 3 seconds (THIS IS NOT EVEN CLOSE TO FINAL)
	randNum = getRandom(float(0), float(3 * FREQUENCY));
	if (randNum > ((3 * FREQUENCY) - 1)) {
		changeLevel(-1, HUNGER);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: hunger drop -1 (rand = %.3f)\n", randNum);
		#endif
	}	

	// ...as does thirst
	// On average, it should drop by 1 every second
	randNum = getRandom(float(0), float(1 * FREQUENCY));
	if (randNum > ((1 * FREQUENCY) - 1)) {
		changeLevel(-1, THIRST);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: thirst drop -1 (rand = %.3f)\n", randNum);
		#endif
	}

	// Fitness drops fairly slowly...
	// On average, it should drop by 1 every second
	randNum = getRandom(float(0), float(1 * FREQUENCY));
	if (randNum > ((1 * FREQUENCY) - 1)) {
		changeLevel(-1, THIRST);
		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("EVENT: thirst drop -1 (rand = %.3f)\n", randNum);
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
	if (level == MORALE) {
		if (residentInstance->morale_level_ + change >= LEVEL_MAX) {
			residentInstance->morale_level_ = LEVEL_MAX;
		} else {
			residentInstance->morale_level_ = residentInstance->morale_level_ + change;
		}
		msg_pkg::Morale moraleMessage;
		moraleMessage.level = residentInstance->morale_level_;
		residentInstance->publisherMorale.publish(moraleMessage);	

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Morale: %d", residentInstance->morale_level_);
		return residentInstance->morale_level_;
		#endif

	} else if (level == SOCIALNESS) {
		if (residentInstance->socialness_level_ + change >= LEVEL_MAX) {
			residentInstance->socialness_level_ = LEVEL_MAX;
		} else {
			residentInstance->socialness_level_ = residentInstance->socialness_level_ + change;
		}
		msg_pkg::Socialness socialnessMessage;
		socialnessMessage.level = residentInstance->socialness_level_;
		residentInstance->publisherSocialness.publish(socialnessMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Socialness: %d", residentInstance->socialness_level_);
		return residentInstance->socialness_level_;
		#endif

	} else if (level == HYGIENE) {
		if (residentInstance->hygiene_level_ + change >= LEVEL_MAX) {
			residentInstance->hygiene_level_ = LEVEL_MAX;
		} else {
			residentInstance->hygiene_level_ = residentInstance->hygiene_level_ + change;
		}
		msg_pkg::Hygiene hygieneMessage;
		hygieneMessage.level = residentInstance->hygiene_level_;
		residentInstance->publisherHygiene.publish(hygieneMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Hygiene: %d", residentInstance->hygiene_level_);
		return residentInstance->hygiene_level_;
		#endif

	} else if (level == HUNGER) {
		if (residentInstance->hunger_level_ + change >= LEVEL_MAX) {
			residentInstance->hunger_level_ = LEVEL_MAX;
		} else {
			residentInstance->hunger_level_ = residentInstance->hunger_level_ + change;
		}
		msg_pkg::Hunger hungerMessage;
		hungerMessage.level = residentInstance->hunger_level_;
		residentInstance->publisherHunger.publish(hungerMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Hunger: %d", residentInstance->hunger_level_);
		return residentInstance->hunger_level_;
		#endif

	} else if (level == THIRST) {
		if (residentInstance->thirst_level_ + change >= LEVEL_MAX) {
			residentInstance->thirst_level_ = LEVEL_MAX;
		} else {
			residentInstance->thirst_level_ = residentInstance->thirst_level_ + change;
		}
		msg_pkg::Thirst thirstMessage;
		thirstMessage.level = residentInstance->thirst_level_;
		residentInstance->publisherThirst.publish(thirstMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Thirst: %d", residentInstance->thirst_level_);
		return residentInstance->thirst_level_;
		#endif

	}  else if (level == FITNESS) {
		if (residentInstance->fitness_level_ + change >= LEVEL_MAX) {
			residentInstance->fitness_level_ = LEVEL_MAX;
		} else {
			residentInstance->fitness_level_ = residentInstance->fitness_level_ + change;
		}
		msg_pkg::Fitness fitnessMessage;
		fitnessMessage.level = residentInstance->fitness_level_;
		residentInstance->publisherFitness.publish(fitnessMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Fitness: %d", residentInstance->fitness_level_);
		return residentInstance->fitness_level_;
		#endif

	} else if (level == HEALTH) {
		if (residentInstance->health_level_ + change >= LEVEL_MAX) {
			residentInstance->health_level_ = LEVEL_MAX;
		} else {
			residentInstance->health_level_ = residentInstance->health_level_ + change;
		}
		msg_pkg::Health healthMessage;
		healthMessage.level = residentInstance->health_level_;
		residentInstance->publisherHealth.publish(healthMessage);

		#ifdef DEBUG_CHANGE_LEVEL
		ROS_INFO("Health: %d", residentInstance->health_level_);
		return residentInstance->health_level_;
		#endif

	}  
}


// Basic random number generator with min/range input,
// uses C's 'rand()'
float Resident::getRandom(float minimum, float range) {
	float rn = (float)rand() / (float)RAND_MAX;
    rn = minimum += (rn * range);
	return rn;
}


void oldRNG()
{
	// True random number generator (uses an entropy pool as opposed to
	// time.h seed, too CPU intensive for the moment)

	// Access/dev/random in read-only for true, random 
	// number generation
	//DELAY TEST
	//clock_t begin = clock();

	/*
	
	#ifdef USE_DEV_RANDOM
	randomData = open("/dev/random", O_RDONLY);
	randomData = open("/dev/random", O_RDONLY);
	// Note: /dev/random blocks when it's empty, and it seems to become empty pretty quickly :(
	// It's probably higher quality than we need, but /dev/urandom sounds like it might be an okay compromise
	// It takes the numbers from /dev/random, but also generates new lower-quality pseudo-random numbers instead of blocking
	// I reckon we can get away with using C's rand(), which is just a simple linear congruential generator, but we're not storing passwords or anything.

	myRandomInteger;
	randomDataLen = 0;
	
	while (randomDataLen < sizeof myRandomInteger)
	{
		ssize_t randomResult = read(randomData, ((char*)&myRandomInteger) + randomDataLen, (sizeof myRandomInteger) - randomDataLen);

		// ssize_t type will return negative if an error occurs while
		// the random number is being generated and assigned to result
		if (randomResult < 0)
		{
			ROS_INFO("Unable to read /dev/random, resorting to alternative RNG");
		}

		randomDataLen += randomResult;
	}

	close(randomData);
	ROS_INFO("Random number generated: %zu", randomDataLen);
	//ROS_INFO("Random number generated: %d", randomDataLen);
	
#else // Use C's rand()

	long myRandomInteger = 0;
	
	// RAND_MAX is guaranteed to be >= 32767, but that's 15 bytes, so to cover 32 bytes completely, we'll need to do it in 3 stages
	for (int shiftVal = 0; shiftVal < 32; shiftVal += 15)
	{
		// Fill out myRandomInteger, 15 bits at a time
		myRandomInteger |= (std::rand() & 0x7fff) << shiftVal;
	}
	
	//ROS_INFO("Random number generated: %d", myRandomInteger);
#endif


	// DELAY TEST
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout.precision(dbl::digits10);
	cout << "Delay: " << fixed << elapsed_secs << endl;

	*/
}


void Resident::wakeUp()
{
  // Reset sleep value for the day
  has_gone_to_bed_ = false;

  has_woken_ = true;
}
void Resident::eat(int hour)
{
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
  // Reset values for next day
  has_eaten_breakfast_ = false;
  has_eaten_breakfast_ = false;
  has_eaten_lunch_ = false;
  has_eaten_dinner_ = false;
  has_woken_ = false;

	has_gone_to_bed_ = true;
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
