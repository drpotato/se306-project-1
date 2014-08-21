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

#include <msg_pkg/Interaction.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Entertainedness.h>
#include "Actor.h"
#include "ActorSpawner.h"

void Resident::doInitialSetup()
{
  velLinear = 0;
  velRotational = 0.0;

  // Initially the resident is not locked
  lock_ = false;

  // Set levels to maximum initially
  entertainedness_level_ = 5;
  socialness_level_ = 5;

  entertainment_count_ = 0;
  socialness_count_ = 0;
  e_dropped_ = false;
  e_replenished_ = false;

  // Set up a publishers
  publisherSocialness = nodeHandle->advertise<msg_pkg::Socialness>("socialness", 1000);
  publisherEntertainedness = nodeHandle->advertise<msg_pkg::Entertainedness>("entertainedness", 1000);

  // Set up subscribers
  subscriberInteraction = nodeHandle->subscribe("interaction", 1000, Resident::interactionCallback);

  // Reset delay counter
  msAtPreviousLoop = 0;

}

void Resident::doExecuteLoop()
{
	//PathPlannerNode *target = this->pathPlanner.getNode(&node4Name);
    //vector<PathPlannerNode*> path = this->pathPlanner.pathToNode(this->activeNode,target);
    //this->goToNode(path);
    
////////////////////////////////////////////////////
// _REMOVE (when randomness implementation complete) 

	if (entertainment_count_ >= WAIT_TIME && !e_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->entertainedness_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			e_dropped_ = true;
			//ROS_INFO("e_dropped changed");
		}
		else 
		{
			// reduce the level every 1000 counts
			residentInstance->entertainedness_level_--;
			//Create a socialness message to publish
			msg_pkg::Entertainedness entertainednessMessage;
			//Assign current socialness level to the message
			entertainednessMessage.level = residentInstance->entertainedness_level_;
			//Publish the message
			residentInstance->publisherEntertainedness.publish(entertainednessMessage);

		}
		entertainment_count_ = 0;
	}
	else if (entertainment_count_ < WAIT_TIME && !e_dropped_)
	{
		entertainment_count_++;
	}
	else if (e_replenished_ && (socialness_count_ >= WAIT_TIME) && !s_dropped_)
	{
		Resident* residentInstance = dynamic_cast<Resident*>(ActorSpawner::getInstance().getActor());
		if(residentInstance->socialness_level_ <= 1)
		{
			// don't drop the value any more, it's being tended to or has been already
			s_dropped_ = true;
		}
		else 
		{
			
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
	else if (e_replenished_ && (socialness_count_ < WAIT_TIME) && !s_dropped_)
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
 * Robots should unlock the resident after interation * so that another robot may interact with the resident.
 */
void Resident::unlock()
{
  lock_ = false;
}

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
	int newLevel = getNewLevel(amount, residentInstance->entertainedness_level_);
	// Update the residents socialness level
	residentInstance->entertainedness_level_ = newLevel;
	//Create a socialness message to publish
	msg_pkg::Entertainedness entertainednessMessage;
	//Assign current socialness level to the message
	entertainednessMessage.level = newLevel;

	if (newLevel == 5)
	{
		residentInstance->stopRobotSpinning();
		residentInstance->e_replenished_ = true;
	}

	//Publish the message
	residentInstance->publisherEntertainedness.publish(entertainednessMessage);
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

	ROS_INFO("Calculating random event(s)...\n");
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
	
	
	// Entertainedness drops fastest and is most affected by randomness.
	printf("rng = %.3f", getRandom(float(0), float(100)));

	// Morale also drops quickly but is less affected by randomness than
	// entertainedness.
	randNum = (float)rand() / (float)RAND_MAX;

	// Health drops in two ways, either almost completely linearly or in a
	// random, drastic fashion.
	randNum = (float)rand() / (float)RAND_MAX;

	// Hunger drops almost completely linearly...
	randNum = (float)rand() / (float)RAND_MAX;

	// ...as does thirst
	randNum = (float)rand() / (float)RAND_MAX;

	



	

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

	/*randomData = open("/dev/random", O_RDONLY);
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

	// DELAY TEST
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout.precision(dbl::digits10);
	cout << "Delay: " << fixed << elapsed_secs << endl;

	*/
}
