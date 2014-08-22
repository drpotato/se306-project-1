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

#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Morale.h>
#include "Actor.h"
#include "ActorSpawner.h"
#include <ctime>
#include <sstream>

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

  time_of_day = std::time(0);

  // Set levels to maximum initially.
  morale_level_ = 5;
  socialness_level_ = 5;

  morale_count_ = 0;
  socialness_count_ = 0;
  m_dropped_ = false;
  m_replenished_ = false;

  // Set up publishers.
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherMorale = nodeHandle->advertise<msg_pkg::Morale>("morale", 1000);

  // Set up subscriptions.
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);

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
    
////////////////////////////////////////////////////
// _REMOVE (when randomness implementation complete) 

	//ROS_INFO("%s", ctime(&time_of_day));
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

	if (newLevel == 5)
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

	if (newLevel == 5)
	{
		residentInstance->stopRobotSpinning();
		residentInstance->m_replenished_ = true;
	}

	//Publish the message
	residentInstance->publisherMorale.publish(moraleMessage);
  }
  // TODO: put others in when implemented 
}

int Resident::getNewLevel(int amount, int oldLevel)
{
	int newLevel = std::min(amount + oldLevel, 5); // Can only have a maximum level of 5
	// Code to check it doesn't go below 1... just incase interactions can reduce levels at some point
	if (newLevel < 1)
	{
		newLevel = 1;
	}
	return newLevel;
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
	
	printf("time since last loop = %lldms\n", (milliseconds - msAtPreviousLoop));
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

	// Socialness drops fastest and is most affected by randomness.
	// On average, it should drop by 1 every 10 seconds
	randNum = getRandom(float(0), float(10 * FREQUENCY));
	if (randNum > ((10 * FREQUENCY) - 1)) {
		printf("Event occured: rng = %.3f", randNum);
	} else {
		//printf("Event not occured: rng = %.3f", randNum);
	}
	
	// Morale also drops quickly but is less affected by randomness than
	// entertainedness.
	// On average, it should drop by 1 every 14 seconds
	randNum = (float)rand() / (float)RAND_MAX;

	// Health drops in two ways, either almost slowly and almost 
	// completely linearly or in a random, drastic fashion.
	// On average, it should drop by 1 every 25 seconds
		

	// Hunger drops almost completely linearly...
	// On average, it should drop by 1 every 10 seconds
	randNum = (float)rand() / (float)RAND_MAX;

	// ...as does thirst
	// On average, it should drop by 1 every 10 seconds
	randNum = (float)rand() / (float)RAND_MAX;

	//ROS_INFO("Calculating random event(s)...");
	

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
