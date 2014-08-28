#include <string.h>
#include <msg_pkg/Time.h>
#include <ctime>
#include <time.h>
#include "Clock.h"
#include "ActorSpawner.h"

// The clock object keeps track of the time of day and publishes it to the time topic

void Clock::doInitialSetup()
{
  // Initialise time of day to current time
  time_of_day = std::time(0);
  invertHours(gmtime(&time_of_day)->tm_hour);

  // Get number of seconds to add on each loop
  seconds_to_add = secondIncreasePerLoop();

  // Set up publisher.
  publisherTime = nodeHandle->advertise<msg_pkg::Time>("time", 1000);
}

void Clock::doExecuteLoop()
{ 
  // Grab out the hour value from the current Ultron world time
  ptm = gmtime(&time_of_day);

	// Increment the time of day by the value calculated previously (in seconds)
  // Increment faster if night time
  if (isNightTime(ptm->tm_hour))
  {
    time_of_day += seconds_to_add*10;
  }
  else
  {
    time_of_day += seconds_to_add; //1800; <-- a good debug time frame is 1800 seconds
  }

  // Create a time message to publish
  msg_pkg::Time timeMessage;

  // Assign current time values to the message
  timeMessage.hour = ptm->tm_hour;
  timeMessage.minutes = ptm->tm_min;
  timeMessage.seconds = ptm->tm_sec;

  // Publish the message
  Clock* clockInstance = dynamic_cast<Clock*>(ActorSpawner::getInstance().getActor());
  clockInstance->publisherTime.publish(timeMessage);
}

int Clock::secondIncreasePerLoop()
{
  Clock* clockInstance = dynamic_cast<Clock*>(ActorSpawner::getInstance().getActor());
  int loop_rate = clockInstance->LOOP_RATE;

  // 1 hr (Ultron world) = 30 seconds (real world)
  // Number of loops needed to pass 30 seconds in the real world: (loop_rate is number loops per second)
  double num_loops_for_real_30_seconds = loop_rate * 30; //e.g. 300

  // Number of loops needed to pass one minute in the Ultron world
  double num_loops_per_ultron_minute = num_loops_for_real_30_seconds / 60; //e.g. 5

  // Number of seconds to add to the time_of_day on each loop in order to pass one Ultron hour in real world 30 seconds
  double num_ultron_seconds_per_loop = 60 / num_loops_per_ultron_minute; //e.g. 12

  return num_ultron_seconds_per_loop;
}

void Clock::invertHours(int hour)
{
  if (hour < 12 )
  {
    // Add 12 hours to the time
    time_of_day = time_of_day += (60*60*12);
  }
  else if (hour >= 12)
  {
    // Minus 12 hours from the time
    time_of_day = time_of_day -= (60*60*12);
  }
}

bool Clock::isNightTime(int hour)
{
  if ((hour < 11) && (hour > 6))
  {
    return false;
  }
  return true;
}